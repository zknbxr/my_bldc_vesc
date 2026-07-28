#include "main.h"

/* 软件相电流保护阈值，单位 mA；任一相连续超限 3 ms 后立即停机，0 表示关闭。 */
volatile u16 gMotorCurrentLimitMa = 2000U;
/* 上电或重新启动时，PWM 使能前的等待时间，单位 ms。 500*/
volatile u16 gMotorStartDelayMs = 10U;
/*
 * 顶层工作模式：上电由MCS_POWER_ON_WORK_MODE选择；学习并保存成功后，
 * 本次运行自动由LEARN切换为CONTROL。
 */
volatile u8 gMotorWorkMode = MCS_POWER_ON_WORK_MODE;

/*
 * 速度环集中状态。前8项是可调参数，后5项是运行监视量。
 * Watch中展开gMotorSpeedControl即可同时观察目标、反馈、误差和iq输出。
 */
volatile motor_speed_control_t gMotorSpeedControl = {
    30000U, /* ramp_erpm_per_s：速度目标斜坡 */
    410,    /* kp_q10：比例增益，Q10 mA/ERPM */
    1024,   /* ki_q10：积分增益，Q10 mA/(ERPM*s) */
    1500U,  /* iq_limit_ma：驱动电流上限 */
    0U,     /* start_current_ma：0表示速度PI直接启动 */
    100U,   /* current_ramp_ma_per_ms：q轴电流斜坡 */
    500U,   /* close_loop_min_erpm：闭环接管最低速度 */
    20U,    /* close_loop_stable_ms：接管稳定时间 */
    0,      /* target_ramp_erpm */
    0,      /* feedback_erpm */
    0,      /* error_erpm */
    0,      /* iq_command_ma */
    0U      /* closed_loop_active */
};

/* 控制模块私有历史量；参数和监视量仍统一放在gMotorSpeedControl中。 */
typedef struct
{
    u16 start_elapsed_ms;      /* 启动等待累计时间 */
    s32 speed_i_term_q10;      /* 速度PI积分，Q10 mA */
    u16 speed_stable_ms;       /* 闭环接管前的稳定时间 */
    u16 speed_ramp_remainder;  /* ERPM斜坡除以1000后的余数 */
} motor_control_runtime_t;

static motor_control_runtime_t s_motorControlRuntime;

static void Motor_SpeedControlReset(bool reset_target_ramp)
{
    s_motorControlRuntime.speed_i_term_q10 = 0L;
    s_motorControlRuntime.speed_stable_ms = 0U;
    gMotorSpeedControl.closed_loop_active = 0U;
    gMotorSpeedControl.error_erpm = 0;
    gMotorSpeedControl.iq_command_ma = 0;

    if(reset_target_ramp)
    {
        gMotorSpeedControl.target_ramp_erpm = 0;
        s_motorControlRuntime.speed_ramp_remainder = 0U;
        gMotorSpeedControl.feedback_erpm = 0;
    }
}

static s32 Motor_SpeedGetIqLimit(const motor_all_state_t *motor)
{
    s32 current_limit;

    current_limit = (s32)gMotorSpeedControl.iq_limit_ma;
    if((motor != 0) && (motor->m_conf != 0))
    {
        if(McsMath_AbsS32((s32)motor->m_conf->lo_current_max) <
           current_limit)
        {
            current_limit = McsMath_AbsS32(
                (s32)motor->m_conf->lo_current_max);
        }
        if(McsMath_AbsS32((s32)motor->m_conf->lo_current_min) <
           current_limit)
        {
            current_limit = McsMath_AbsS32(
                (s32)motor->m_conf->lo_current_min);
        }
    }

    return McsMath_LimitS32(current_limit, 0L, 32767L);
}

/* 对带符号的速度目标做斜坡，余数累积避免低斜率在1ms整数计算中丢失。 */
static s16 Motor_SpeedRampUpdate(s16 target, u16 elapsed_ms)
{
    u32 numerator;
    s32 step;

    if(gMotorSpeedControl.ramp_erpm_per_s == 0U)
    {
        gMotorSpeedControl.target_ramp_erpm = target;
        s_motorControlRuntime.speed_ramp_remainder = 0U;
        return target;
    }

    numerator = (u32)gMotorSpeedControl.ramp_erpm_per_s * (u32)elapsed_ms +
                (u32)s_motorControlRuntime.speed_ramp_remainder;
    step = (s32)(numerator / 1000UL);
    s_motorControlRuntime.speed_ramp_remainder = (u16)(numerator % 1000UL);
    if(step > 32767L)
    {
        step = 32767L;
    }
    if(step > 0L)
    {
        gMotorSpeedControl.target_ramp_erpm = McsMath_StepTowardsS16(
            gMotorSpeedControl.target_ramp_erpm, target, step);
    }

    return gMotorSpeedControl.target_ramp_erpm;
}

bool Motor_IsControlRunning(const motor_all_state_t *motor)
{
    return (motor != 0) &&
           (motor->m_run_state == MOTOR_RUN_STATE_RUNNING) &&
           Motor_IsPwmEnabled();
}

bool Motor_IsControlActive(const motor_all_state_t *motor)
{
    return (motor != 0) &&
           (motor->m_control_mode != CONTROL_MODE_NONE);
}

bool Motor_IsPwmEnabled(void)
{
    return (MCPWM_FAIL012 & MCPWM_MOE_ENABLE_MASK) != 0U;
}

bool Motor_IsRotorMoving(const motor_all_state_t *motor, s16 min_erpm)
{
    s32 speed_abs;
    s32 threshold;

    if(motor == 0)
    {
        return false;
    }

    speed_abs = McsMath_AbsS32((s32)motor->m_pll_speed);
    threshold = McsMath_AbsS32((s32)min_erpm);
    return speed_abs > threshold;
}

void Motor_SetCurrentTarget(motor_all_state_t *motor,
                            s16 id_target_ma,
                            s16 iq_target_ma)
{
    s32 current_limit;

    if((motor == 0) || (motor->m_conf == 0))
    {
        return;
    }

    current_limit = McsMath_AbsS32((s32)motor->m_conf->lo_current_max);
    if(McsMath_AbsS32((s32)motor->m_conf->lo_current_min) > current_limit)
    {
        current_limit = McsMath_AbsS32((s32)motor->m_conf->lo_current_min);
    }
    current_limit = McsMath_LimitS32(current_limit, 0L, 32767L);

    motor->m_id_set_target = (s16)McsMath_LimitS32(
        (s32)id_target_ma, -current_limit, current_limit);
    motor->m_iq_set_target = (s16)McsMath_LimitS32(
        (s32)iq_target_ma, -current_limit, current_limit);
}

/*
 * 电流指令斜坡更新，由 1 ms 慢速任务调用，不放进 ADC 中断。
 *
 * 数据流：
 *   m_id/iq_set_target  应用层或速度环发布的最终目标，单位 mA
 *          ↓ 按 current_ramp_ma_per_ms 逐步逼近
 *   m_id/iq_set         ADC 快速电流环实际使用的指令，单位 mA
 *
 * elapsed_ms 使用调度器累计的实际毫秒数，即使主循环偶尔延迟多个周期，
 * 斜坡速度仍按真实经过时间计算。
 */
void Motor_CurrentCommandUpdate(motor_all_state_t *motor, u16 elapsed_ms)
{
    s32 ramp_ma_per_ms;
    s32 speed_iq_limit;
    s32 step;
    s16 id_next;
    s16 iq_target;
    s16 iq_next;

    if((motor == 0) || (motor->m_conf == 0) || (elapsed_ms == 0U))
    {
        return;
    }

    iq_target = motor->m_iq_set_target;

    /* 速度环需要快速增加转矩；学习和直接电流模式仍使用配置中的慢斜坡。 */
    ramp_ma_per_ms = (s32)motor->m_conf->current_ramp_ma_per_ms;
    if((motor->m_control_mode == CONTROL_MODE_SPEED) &&
       (gMotorSpeedControl.current_ramp_ma_per_ms > 0U))
    {
        ramp_ma_per_ms = (s32)gMotorSpeedControl.current_ramp_ma_per_ms;
    }
    
    if(motor->m_control_mode == CONTROL_MODE_SPEED)
    {
        /* 最后一层钳位，防止任何异常目标绕过速度PI的iq限流。 */
        speed_iq_limit = Motor_SpeedGetIqLimit(motor);
        iq_target = (s16)McsMath_LimitS32(
            (s32)iq_target, -speed_iq_limit, speed_iq_limit);
    }

    /* 本次允许变化的电流量 = 每毫秒斜坡值 * 实际经过毫秒数。 */
    step = ramp_ma_per_ms * (s32)elapsed_ms;
    if(step <= 0L)
    {
        /* 配置为 0 或负数时关闭斜坡，直接使用目标值。 */
        id_next = motor->m_id_set_target;
        iq_next = iq_target;
    }
    else
    {
        /* d/q 两轴分别向目标移动，且不会越过目标值。 */
        step = McsMath_LimitS32(step, 1L, 32767L);
        id_next = McsMath_StepTowardsS16(
            motor->m_id_set, motor->m_id_set_target, step);
        iq_next = McsMath_StepTowardsS16(
            motor->m_iq_set, iq_target, step);
    }
    if(motor->m_control_mode == CONTROL_MODE_SPEED)
    {
        iq_next = (s16)McsMath_LimitS32(
            (s32)iq_next, -speed_iq_limit, speed_iq_limit);
    }

    /* ADC 中断必须看到属于同一个控制周期的 d/q 指令。 */
    __disable_irq();
    motor->m_id_set = id_next;
    motor->m_iq_set = iq_next;
    __enable_irq();
}

void Motor_CurrentCommandReset(motor_all_state_t *motor)
{
    if(motor == 0)
    {
        return;
    }

    motor->m_id_set_target = 0;
    motor->m_iq_set_target = 0;
    __disable_irq();
    motor->m_id_set = 0;
    motor->m_iq_set = 0;
    __enable_irq();
}

/*
 * 1ms速度外环。速度PI只生成q轴电流目标，真正的d/q电流PI仍在ADC中断执行。
 * 学习旋转和控制模式共用本速度环；固定启动电流设为0时，PI从零电流直接接管。
 */
void Motor_SpeedControlUpdate1ms(motor_all_state_t *motor, u16 elapsed_ms)
{
    s32 target_abs;
    s16 target_signed;
    s16 target_ramped;
    s32 feedback;
    s32 error;
    s32 iq_limit;
    s32 brake_limit;
    s32 startup_current;
    s32 entry_speed;
    s32 p_term_q10;
    s32 i_delta_q10;
    s32 i_candidate_q10;
    s32 takeover_iq_q10;
    s32 output_q10;
    s32 output_limit_q10;
    s32 output_min_q10;
    s32 output_max_q10;
    s32 dt_ms;

    if((motor == 0) || (motor->m_conf == 0) || (elapsed_ms == 0U))
    {
        return;
    }

    /* 模式层已经生成带符号速度请求，速度环只关心公共运行状态。 */
    if((motor->m_run_state != MOTOR_RUN_STATE_RUNNING) ||
       (motor->m_control_mode != CONTROL_MODE_SPEED) ||
       !Motor_IsPwmEnabled())
    {
        Motor_SpeedControlReset(true);
        return;
    }

    target_signed = motor->m_speed_pid_set_rpm;
    target_abs = McsMath_AbsS32((s32)target_signed);
    target_abs = McsMath_LimitS32(target_abs, 0L, 32767L);
    // 速度斜坡
    target_ramped = Motor_SpeedRampUpdate(target_signed, elapsed_ms);

    /* Hall测速和无感PLL已经完成平滑，速度PI直接使用该反馈以减少延迟。 */
    feedback = (s32)motor->m_pll_speed;
    feedback = McsMath_LimitS32(feedback, -32768L, 32767L);
    gMotorSpeedControl.feedback_erpm = (s16)feedback;

    iq_limit = Motor_SpeedGetIqLimit(motor);
    if((target_abs == 0L) || (iq_limit == 0L))
    {
        Motor_SpeedControlReset(true);
        Motor_SetCurrentTarget(motor, 0, 0);
        return;
    }
    
    output_limit_q10 = iq_limit << 10;
    
    brake_limit = McsMath_LimitS32(
        MCS_SPEED_BRAKE_IQ_LIMIT_MA, 0L, iq_limit);
    
    if(target_signed > 0)
    {
        /* 正转超速时允许小幅负iq制动，避免零电流滑行造成周期性波动。 */
        output_min_q10 = -(brake_limit << 10);
        output_max_q10 = output_limit_q10;
    }
    else
    {
        output_min_q10 = -output_limit_q10;
        /* 反转超速时，正iq是制动方向。 */
        output_max_q10 = brake_limit << 10;
    }

    /* 低目标速度时把接管门限降到目标值，避免永远停留在固定启动电流。 */
    entry_speed = (s32)gMotorSpeedControl.close_loop_min_erpm;
    if(target_abs < entry_speed)
    {
        entry_speed = target_abs;
    }
    
    if(entry_speed < 50L)
    {
        entry_speed = 50L;
    }

    if(gMotorSpeedControl.closed_loop_active == 0U)
    {
        if(gMotorSpeedControl.start_current_ma == 0U)
        {
            /* 不使用固定启动电流，速度PI从零积分、零电流开始直接接管。 */
            s_motorControlRuntime.speed_stable_ms = 0U;
            s_motorControlRuntime.speed_i_term_q10 = 0L;
            gMotorSpeedControl.closed_loop_active = 1U;
        }
        else
        {
        startup_current = McsMath_LimitS32(
            (s32)gMotorSpeedControl.start_current_ma, 0L, iq_limit);
        if(target_signed < 0)
        {
            startup_current = -startup_current;
        }
        gMotorSpeedControl.iq_command_ma = (s16)startup_current;
        Motor_SetCurrentTarget(motor, 0, (s16)startup_current);

        if(motor->m_phase_control_initialized &&
           (McsMath_AbsS32(feedback) >= entry_speed) &&
           (((target_signed > 0) && (feedback > 0L)) ||
            ((target_signed < 0) && (feedback < 0L))))
        {
            if(s_motorControlRuntime.speed_stable_ms <
               (u16)(65535U - elapsed_ms))
            {
                s_motorControlRuntime.speed_stable_ms += elapsed_ms;
            }
            else
            {
                s_motorControlRuntime.speed_stable_ms = 65535U;
            }
        }
        else
        {
            s_motorControlRuntime.speed_stable_ms = 0U;
        }

        if(s_motorControlRuntime.speed_stable_ms < gMotorSpeedControl.close_loop_stable_ms)
        {
            return;
        }

        /*
         * 欠速时延续当前启动电流；已经超速时让PI从0电流接管。
         * 这样不会在高速接管瞬间继续加速，也不会立即施加反向制动。
         */
        error = (s32)target_ramped - feedback;
        p_term_q10 = error * McsMath_LimitS32(
            (s32)gMotorSpeedControl.kp_q10, 0L, 32767L);
        if(((target_signed > 0) && (error > 0L)) ||
           ((target_signed < 0) && (error < 0L)))
        {
            takeover_iq_q10 = (s32)motor->m_iq_set * 1024L;
            s_motorControlRuntime.speed_i_term_q10 = takeover_iq_q10 - p_term_q10;
        }
        else
        {
            s_motorControlRuntime.speed_i_term_q10 = 0L;
        }
        s_motorControlRuntime.speed_i_term_q10 = McsMath_LimitS32(
            s_motorControlRuntime.speed_i_term_q10, output_min_q10, output_max_q10);
        gMotorSpeedControl.closed_loop_active = 1U;
        }
    }

    error = (s32)target_ramped - feedback;
    gMotorSpeedControl.error_erpm = (s16)McsMath_LimitS32(
        error, -32768L, 32767L);
    p_term_q10 = error * McsMath_LimitS32(
        (s32)gMotorSpeedControl.kp_q10, 0L, 32767L);

    dt_ms = (elapsed_ms > 20U) ? 20L : (s32)elapsed_ms;
    i_delta_q10 = (s32)(((int64_t)error * (int64_t)McsMath_LimitS32(
        (s32)gMotorSpeedControl.ki_q10, 0L, 32767L) * (int64_t)dt_ms) / 1000LL);
    i_candidate_q10 = McsMath_LimitS32(
        s_motorControlRuntime.speed_i_term_q10 + i_delta_q10,
        output_min_q10, output_max_q10);
    output_q10 = p_term_q10 + i_candidate_q10;

    /* 输出饱和且误差仍推动饱和加深时暂停积分，允许反向误差解除饱和。 */
    if(!(((output_q10 > output_max_q10) && (error > 0L)) ||
         ((output_q10 < output_min_q10) && (error < 0L))))
    {
        s_motorControlRuntime.speed_i_term_q10 = i_candidate_q10;
    }
    output_q10 = McsMath_LimitS32(
        p_term_q10 + s_motorControlRuntime.speed_i_term_q10,
        output_min_q10, output_max_q10);
    gMotorSpeedControl.iq_command_ma = (s16)(output_q10 >> 10);
    Motor_SetCurrentTarget(motor, 0, gMotorSpeedControl.iq_command_ma);
}

static void Motor_ControlRequestReset(motor_control_request_t *request)
{
    request->run = false;
    request->control_mode = CONTROL_MODE_NONE;
    request->sensor_mode = FOC_SENSOR_MODE_SENSORLESS;
    request->phase_override = false;
    request->phase_override_q16 = 0;
    request->position.enable = false;
    request->position.target_01mm = 0;
    request->speed.enable = false;
    request->speed.target_erpm = 0;
    request->current.enable = false;
    request->current.id_target_ma = 0;
    request->current.iq_target_ma = 0;
}

/* 正常操作模式只解释应用命令，不直接启停 PWM。 */
static void Motor_OperationBuildControlRequest(
    motor_control_request_t *request)
{
    if((request == 0) || (gMotorWorkMode != MCS_WORK_MODE_CONTROL))
    {
        return;
    }
    
    /*
     * app_height是操作模式命令源。位置层先产生带符号速度请求，
     * 本函数只把应用状态整理成统一控制请求，不重复计算方向和速度。
     */
    request->run = gAppHeight.command.run != 0U;
    request->control_mode = CONTROL_MODE_SPEED;
    request->sensor_mode = FOC_SENSOR_MODE_SENSORLESS;
    request->position.enable =
        gAppHeight.position.mode != APP_POSITION_MODE_IDLE;
    request->position.target_01mm = gAppHeight.position.target_01mm;
    request->speed.enable = request->run;
    request->speed.target_erpm =
        gAppHeight.trajectory.speed_request_erpm;
    
    if((MCS_CONTROL_SENSOR_MODE == FOC_SENSOR_MODE_HALL) &&
       Hall_CalibrationIsValid())
    {
        request->sensor_mode = FOC_SENSOR_MODE_HALL;
    }

}

/* 顶层模式只在这一处选择，后面的公共状态机不需要了解请求来源。 */
static void Motor_ModeBuildControlRequest(motor_control_request_t *request)
{
    /*
     * 每一次都会将状态恢复默认然后重新幅值
     * 在控制模式里选择运动模式和传感器模式，给定目标转速
     */
    Motor_ControlRequestReset(request);

    switch(gMotorWorkMode)
    {
    case MCS_WORK_MODE_LEARN:
        Hall_LearnBuildControlRequest(request);
        break;

    case MCS_WORK_MODE_CONTROL:
        // 传递运动状态，运行模式，传感器模式，目标速度
        Motor_OperationBuildControlRequest(request);
        break;

    default:
        break;
    }
}

static void Motor_RunStateStop(void)
{
    switch(m_motor.m_run_state)
    {
    case MOTOR_RUN_STATE_OFF:
        /* 已经关闭，不再重复写 MOE 或清控制状态。 */
        return;

    case MOTOR_RUN_STATE_START_DELAY:
        /* PWM 尚未打开，取消本次启动即可。 */
        m_motor.m_speed_pid_set_rpm = 0;
        Motor_SetCurrentTarget(&m_motor, 0, 0);
        Motor_SpeedControlReset(true);
        s_motorControlRuntime.start_elapsed_ms = 0U;
        m_motor.m_run_state = MOTOR_RUN_STATE_OFF;
        return;

    case MOTOR_RUN_STATE_RUNNING:
        /* 停止命令只发布一次，后续由 STOPPING 状态等待电流斜坡归零。 */
        m_motor.m_speed_pid_set_rpm = 0;
        Motor_SetCurrentTarget(&m_motor, 0, 0);
        Motor_SpeedControlReset(true);
        m_motor.m_run_state = MOTOR_RUN_STATE_STOPPING;
        return;

    case MOTOR_RUN_STATE_STOPPING:
        if((m_motor.m_id_set != 0) || (m_motor.m_iq_set != 0))
        {
            return;
        }

        /* 电流已经归零，只在本次状态转换中关闭一次 MOE。 */
        __disable_irq();
        m_motor.m_control_mode = CONTROL_MODE_NONE;
        m_motor.m_phase_override = false;
        PwmAOutputs(DISABLE);
        __enable_irq();
        m_motor.m_run_state = MOTOR_RUN_STATE_OFF;
        s_motorControlRuntime.start_elapsed_ms = 0U;
        return;

    case MOTOR_RUN_STATE_FAULT:
    default:
        /* 故障关断由 Motor_FaultTrip() 负责。 */
        return;
    }
}

/*
 * 公共功率运行状态机只执行 motor_control_request_t。
 * 它不知道请求来自串口操作模式还是霍尔学习模式。
 */
static void Motor_RunStateTask1ms(
    const motor_control_request_t *request,
    u16 elapsed_ms)
{
    bool control_changed;

    if((request == 0) || (elapsed_ms == 0U) ||
       Motor_FaultIsActive())
    {
        return;
    }

    if(!request->run)
    {
        Motor_RunStateStop();
        return;
    }

    /*
     * 运行中切换传感器或控制算法会造成角度/积分突变。要求模式层先请求停止，
     * 电流回零并关闭 PWM 后，再以新请求重新启动。
     * 电机停止使能或者切换了传感器控制模式，先关闭PWM
     */
    control_changed = Motor_IsControlActive(&m_motor) &&
        ((m_motor.m_control_mode != request->control_mode) ||
         (m_motor.m_conf->foc_sensor_mode != request->sensor_mode));
    if(control_changed)
    {
        Motor_RunStateStop();
        return;
    }

    if((m_motor.m_run_state == MOTOR_RUN_STATE_OFF) ||
       (m_motor.m_run_state == MOTOR_RUN_STATE_START_DELAY))
    {
        m_motor.m_run_state = MOTOR_RUN_STATE_START_DELAY;
        if(s_motorControlRuntime.start_elapsed_ms < gMotorStartDelayMs)
        {
            s_motorControlRuntime.start_elapsed_ms += elapsed_ms;
            return;
        }

        m_motor.m_conf->foc_sensor_mode = request->sensor_mode;
        m_motor.m_phase_override = request->phase_override;
        if(request->phase_override)
        {
            m_motor.m_motor_state.phase = request->phase_override_q16;
        }

        m_motor.m_speed_pid_set_rpm = request->speed.target_erpm;

        __disable_irq();
        m_motor.m_control_mode = request->control_mode;
        PwmAOutputs(ENABLE);
        __enable_irq();
        m_motor.m_run_state = MOTOR_RUN_STATE_RUNNING;
    }

    if(m_motor.m_run_state == MOTOR_RUN_STATE_STOPPING)
    {
        /* 停机斜坡尚未结束时收到新的同模式命令，允许平滑恢复运行。 */
        m_motor.m_run_state = MOTOR_RUN_STATE_RUNNING;
    }

    if(m_motor.m_run_state != MOTOR_RUN_STATE_RUNNING)
    {
        return;
    }

    m_motor.m_phase_override = request->phase_override;
    if(request->phase_override)
    {
        m_motor.m_motor_state.phase = request->phase_override_q16;
    }
    m_motor.m_speed_pid_set_rpm = request->speed.target_erpm;

    if((request->control_mode == CONTROL_MODE_CURRENT) &&
       request->current.enable)
    {
        Motor_SetCurrentTarget(&m_motor,
                               request->current.id_target_ma,
                               request->current.iq_target_ma);
    }
}

void Motor_ControlInit(void)
{
    __disable_irq();
    m_motor.m_phase_override = false;
    m_motor.m_control_mode = CONTROL_MODE_NONE;
    PwmAOutputs(DISABLE);
    __enable_irq();

    Motor_CurrentCommandReset(&m_motor);
    Motor_SpeedControlReset(true);
    Motor_FaultInit();

    /* 应用命令由AppHeight_Init初始化；MCS初始化只整理电机实时状态。 */
    m_motor.m_run_state = MOTOR_RUN_STATE_OFF;
    m_motor.m_fault_code = MOTOR_FAULT_NONE;
    m_motor.m_speed_pid_set_rpm = 0;
    s_motorControlRuntime.start_elapsed_ms = 0U;
}

/*
 * 1 ms 模式入口。学习模式和操作模式分别生成统一请求，公共状态机执行请求。
 * 故障判断已独立到 Motor_FaultTask1ms()。
 */
void Motor_ControlTask1ms(u16 elapsed_ms)
{
    /*
     * gAppHeight：应用层长期状态；
     * request：本次1ms生成的分层请求快照；
     * m_motor：已经作用于控制器的电机实时状态。
     */
    motor_control_request_t request;

    if(elapsed_ms == 0U)
    {
        return;
    }

    Motor_ModeBuildControlRequest(&request);
    Motor_RunStateTask1ms(&request, elapsed_ms);
}



void StopMotorImmdly(void)
{
    Motor_FaultTrip(MOTOR_FAULT_HARDWARE_SHORT);
}

