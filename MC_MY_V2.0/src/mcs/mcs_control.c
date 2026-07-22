#include "main.h"

#define MCS_CONTROL_STATE_OFF               (0U)
#define MCS_CONTROL_STATE_START_DELAY       (1U)
#define MCS_CONTROL_STATE_RUNNING           (2U)
#define MCS_CONTROL_STATE_STOPPING          (3U)
#define MCS_CONTROL_STATE_FAULT             (4U)
#define MCS_CONTROL_CURRENT_TRIP_MS          (3U)
#define MCS_CONTROL_MOE_MASK                 (0x0040U)

extern volatile UINT16 gShortFaultCount;

/*
 * 正式控制命令，可由应用层修改，也保留为全局变量方便 Keil Watch 观察。
 * 电流目标只表示幅值，最终 q 轴符号由 gMotorDirection 决定。
 */
/* 运行命令：1 允许延时启动和运行；0 将电流斜坡降到 0 后关闭 PWM。 */
volatile u8 gMotorRunEnable = 0U;
/* 机械方向：MCS_MOTOR_DIRECTION_FORWARD(+1) 或 REVERSE(-1)。 */
volatile s8 gMotorDirection = MCS_MOTOR_DIRECTION_DEFAULT;
/* 正式运行的 q 轴电流目标幅值，单位 mA，当前默认 600 mA。 */
volatile s16 gMotorCurrentTargetMa = 600;
/* 软件相电流保护阈值，单位 mA；任一相连续超限 3 ms 后立即停机，0 表示关闭。 */
volatile u16 gMotorCurrentLimitMa = 2000U;
/* 上电或重新启动时，PWM 使能前的等待时间，单位 ms。 */
volatile u16 gMotorStartDelayMs = 500U;
/* 操作模式角度源：默认霍尔；改为FOC_SENSOR_MODE_SENSORLESS可使用无感。 */
volatile u8 gMotorOperationSensorMode = FOC_SENSOR_MODE_HALL;
/* 操作模式外环：默认速度环；切换为CURRENT后恢复串口方向+固定电流控制。 */
volatile u8 gMotorOperationControlMode = MCS_OPERATION_CONTROL_SPEED;
/* 控制状态：0 关闭，1 启动延时，2 运行，3 斜坡停止中，4 故障停机。 */
volatile u8 gMotorControlState;
/* 故障码：0 无故障，1 软件过流，2 MCPWM 硬件故障，3 MOE 意外关闭。 */
volatile u8 gMotorControlFaultCode;

/* 速度环目标只保存幅值，正负方向仍由gMotorDirection决定，单位ERPM。 */
volatile s16 gMotorSpeedTargetErpm = MCS_SPEED_TARGET_DEFAULT_ERPM;
/* 速度目标斜坡，单位ERPM/s；4000表示从0升到3000约需0.75秒。 */
volatile u16 gMotorSpeedRampErpmPerS = 4000U;
/* 速度PI参数，Q10单位分别为mA/ERPM和mA/(ERPM*s)。 */
volatile s16 gMotorSpeedKpQ10 = 205;
volatile s16 gMotorSpeedKiQ10 = 64;
/* 速度PI输出限幅与可选固定启动电流，单位mA；启动电流为0时由PI直接起步。 */
volatile u16 gMotorSpeedIqLimitMa = 1500U;
volatile u16 gMotorSpeedStartCurrentMa = 0U;
/* 速度模式单独使用更快的电流斜坡，使负载突变时能及时增加转矩。 */
volatile u16 gMotorSpeedCurrentRampMaPerMs = 10U;
/* 无感/Hall启动达到该速度并稳定一段时间后，速度PI才接管。 */
volatile u16 gMotorSpeedCloseLoopMinErpm = 500U;
volatile u16 gMotorSpeedCloseLoopStableMs = 20U;
/* 速度环运行诊断量，均可直接放入Keil Watch。 */
volatile s16 gMotorSpeedTargetRampErpm;
volatile s16 gMotorSpeedFeedbackErpm;
volatile s16 gMotorSpeedErrorErpm;
volatile s16 gMotorSpeedIqCommandMa;
volatile u8 gMotorSpeedClosedLoopActive;

/* 已累计的启动等待时间，单位 ms；达到 gMotorStartDelayMs 后使能 PWM。 */
static u16 s_motorStartElapsedMs;
/* 相电流连续超限时间，单位 ms；正常一次便清零，用于滤除单次采样毛刺。 */
static u16 s_motorCurrentOverMs;
/* PWM 使能瞬间保存的硬件短路故障计数，用于判断运行后是否出现新故障。 */
static u16 s_shortFaultCountAtArm;
/* PWM 已正式使能标志；用于区分“尚未启动”和“运行中 MOE 被硬件关闭”。 */
static bool s_motorPwmArmed;
/* 速度反馈使用Q8低通状态，积分项使用Q10 mA，避免慢速变化被整数截断。 */
static s32 s_motorSpeedFeedbackQ8;
static s32 s_motorSpeedITermQ10;
static u16 s_motorSpeedStableMs;
static u16 s_motorSpeedRampRemainder;

static s32 Motor_ControlAbsS32(s32 value)
{
    return (value >= 0L) ? value : -value;
}

static s32 Motor_ControlLimitS32(s32 value, s32 min, s32 max)
{
    if(value > max)
    {
        return max;
    }
    if(value < min)
    {
        return min;
    }
    return value;
}

static s16 Motor_ControlStepTowards(s16 value, s16 target, s32 step)
{
    s32 next;

    next = (s32)value;
    if(next < (s32)target)
    {
        next += step;
        if(next > (s32)target)
        {
            next = target;
        }
    }
    else if(next > (s32)target)
    {
        next -= step;
        if(next < (s32)target)
        {
            next = target;
        }
    }

    return (s16)next;
}

static void Motor_SpeedControlReset(bool reset_target_ramp)
{
    s_motorSpeedITermQ10 = 0L;
    s_motorSpeedStableMs = 0U;
    gMotorSpeedClosedLoopActive = 0U;
    gMotorSpeedErrorErpm = 0;
    gMotorSpeedIqCommandMa = 0;

    if(reset_target_ramp)
    {
        gMotorSpeedTargetRampErpm = 0;
        s_motorSpeedRampRemainder = 0U;
        s_motorSpeedFeedbackQ8 = 0L;
        gMotorSpeedFeedbackErpm = 0;
    }
}

static s32 Motor_SpeedGetIqLimit(const motor_all_state_t *motor)
{
    s32 current_limit;

    current_limit = (s32)gMotorSpeedIqLimitMa;
    if((motor != 0) && (motor->m_conf != 0))
    {
        if(Motor_ControlAbsS32((s32)motor->m_conf->lo_current_max) <
           current_limit)
        {
            current_limit = Motor_ControlAbsS32(
                (s32)motor->m_conf->lo_current_max);
        }
        if(Motor_ControlAbsS32((s32)motor->m_conf->lo_current_min) <
           current_limit)
        {
            current_limit = Motor_ControlAbsS32(
                (s32)motor->m_conf->lo_current_min);
        }
    }

    return Motor_ControlLimitS32(current_limit, 0L, 32767L);
}

/* 对带符号的速度目标做斜坡，余数累积避免低斜率在1ms整数计算中丢失。 */
static s16 Motor_SpeedRampUpdate(s16 target, u16 elapsed_ms)
{
    u32 numerator;
    s32 step;

    if(gMotorSpeedRampErpmPerS == 0U)
    {
        gMotorSpeedTargetRampErpm = target;
        s_motorSpeedRampRemainder = 0U;
        return target;
    }

    numerator = (u32)gMotorSpeedRampErpmPerS * (u32)elapsed_ms +
                (u32)s_motorSpeedRampRemainder;
    step = (s32)(numerator / 1000UL);
    s_motorSpeedRampRemainder = (u16)(numerator % 1000UL);
    if(step > 32767L)
    {
        step = 32767L;
    }
    if(step > 0L)
    {
        gMotorSpeedTargetRampErpm = Motor_ControlStepTowards(
            gMotorSpeedTargetRampErpm, target, step);
    }

    return gMotorSpeedTargetRampErpm;
}

static bool Motor_ControlCurrentExceeded(void)
{
    s32 current_max;

    if(gMotorCurrentLimitMa == 0U)
    {
        return false;
    }

    current_max = Motor_ControlAbsS32((s32)ADC_curr_norm_value[0]);
    if(Motor_ControlAbsS32((s32)ADC_curr_norm_value[1]) > current_max)
    {
        current_max = Motor_ControlAbsS32((s32)ADC_curr_norm_value[1]);
    }
    if(Motor_ControlAbsS32((s32)ADC_curr_norm_value[2]) > current_max)
    {
        current_max = Motor_ControlAbsS32((s32)ADC_curr_norm_value[2]);
    }

    return current_max > (s32)gMotorCurrentLimitMa;
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

    current_limit = Motor_ControlAbsS32((s32)motor->m_conf->lo_current_max);
    if(Motor_ControlAbsS32((s32)motor->m_conf->lo_current_min) > current_limit)
    {
        current_limit = Motor_ControlAbsS32((s32)motor->m_conf->lo_current_min);
    }
    current_limit = Motor_ControlLimitS32(current_limit, 0L, 32767L);

    motor->m_id_set_target = (s16)Motor_ControlLimitS32(
        (s32)id_target_ma, -current_limit, current_limit);
    motor->m_iq_set_target = (s16)Motor_ControlLimitS32(
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
       (gMotorSpeedCurrentRampMaPerMs > 0U))
    {
        ramp_ma_per_ms = (s32)gMotorSpeedCurrentRampMaPerMs;
    }
    if(motor->m_control_mode == CONTROL_MODE_SPEED)
    {
        /* 最后一层钳位，防止任何异常目标绕过速度PI的iq限流。 */
        speed_iq_limit = Motor_SpeedGetIqLimit(motor);
        iq_target = (s16)Motor_ControlLimitS32(
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
        step = Motor_ControlLimitS32(step, 1L, 32767L);
        id_next = Motor_ControlStepTowards(
            motor->m_id_set, motor->m_id_set_target, step);
        iq_next = Motor_ControlStepTowards(
            motor->m_iq_set, iq_target, step);
    }
    if(motor->m_control_mode == CONTROL_MODE_SPEED)
    {
        iq_next = (s16)Motor_ControlLimitS32(
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
 * 启动阶段先使用固定q轴电流建立可靠角度和速度，达到门限并稳定后再无扰接管。
 */
void Motor_SpeedControlUpdate1ms(motor_all_state_t *motor, u16 elapsed_ms)
{
    s32 target_abs;
    s16 target_signed;
    s16 target_ramped;
    s32 feedback_q8_target;
    s32 filter_elapsed;
    s32 feedback;
    s32 error;
    s32 iq_limit;
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

    /* 学习模式和电流模式都不允许速度环覆盖各自发布的电流目标。 */
    if((gHallWorkMode != HALL_WORK_MODE_NORMAL) ||
       (gMotorOperationControlMode != MCS_OPERATION_CONTROL_SPEED) ||
       (gMotorRunEnable == 0U) || !s_motorPwmArmed ||
       (motor->m_control_mode != CONTROL_MODE_SPEED))
    {
        Motor_SpeedControlReset(true);
        return;
    }

    target_abs = Motor_ControlAbsS32((s32)gMotorSpeedTargetErpm);
    target_abs = Motor_ControlLimitS32(target_abs, 0L, 32767L);
    target_signed = (s16)target_abs;
    if(gMotorDirection == MCS_MOTOR_DIRECTION_REVERSE)
    {
        target_signed = (s16)(-target_abs);
    }
    // 速度斜坡
    target_ramped = Motor_SpeedRampUpdate(target_signed, elapsed_ms);

    /* m_pll_speed 已经统一为ERPM，这里再做约8ms的一阶低通供速度PI使用。 */
    filter_elapsed = (elapsed_ms > 8U) ? 8L : (s32)elapsed_ms;
    feedback_q8_target = (s32)motor->m_pll_speed << 8;
    s_motorSpeedFeedbackQ8 +=
        ((feedback_q8_target - s_motorSpeedFeedbackQ8) * filter_elapsed) >> 3;
    feedback = s_motorSpeedFeedbackQ8 >> 8;
    feedback = Motor_ControlLimitS32(feedback, -32768L, 32767L);
    gMotorSpeedFeedbackErpm = (s16)feedback;

    iq_limit = Motor_SpeedGetIqLimit(motor);
    if((target_abs == 0L) || (iq_limit == 0L))
    {
        Motor_SpeedControlReset(true);
        Motor_SetCurrentTarget(motor, 0, 0);
        return;
    }
    output_limit_q10 = iq_limit << 10;
    if(target_signed > 0)
    {
        output_min_q10 = 0L;
        output_max_q10 = output_limit_q10;
    }
    else
    {
        output_min_q10 = -output_limit_q10;
        output_max_q10 = 0L;
    }

    /* 低目标速度时把接管门限降到目标值，避免永远停留在固定启动电流。 */
    entry_speed = (s32)gMotorSpeedCloseLoopMinErpm;
    if(target_abs < entry_speed)
    {
        entry_speed = target_abs;
    }
    if(entry_speed < 50L)
    {
        entry_speed = 50L;
    }

    if(gMotorSpeedClosedLoopActive == 0U)
    {
        if(gMotorSpeedStartCurrentMa == 0U)
        {
            /* 不使用固定启动电流，速度PI从零积分、零电流开始直接接管。 */
            s_motorSpeedStableMs = 0U;
            s_motorSpeedITermQ10 = 0L;
            gMotorSpeedClosedLoopActive = 1U;
        }
        else
        {
        startup_current = Motor_ControlLimitS32(
            (s32)gMotorSpeedStartCurrentMa, 0L, iq_limit);
        if(target_signed < 0)
        {
            startup_current = -startup_current;
        }
        gMotorSpeedIqCommandMa = (s16)startup_current;
        Motor_SetCurrentTarget(motor, 0, (s16)startup_current);

        if(motor->m_phase_control_initialized &&
           (Motor_ControlAbsS32(feedback) >= entry_speed) &&
           (((target_signed > 0) && (feedback > 0L)) ||
            ((target_signed < 0) && (feedback < 0L))))
        {
            if(s_motorSpeedStableMs <
               (u16)(65535U - elapsed_ms))
            {
                s_motorSpeedStableMs += elapsed_ms;
            }
            else
            {
                s_motorSpeedStableMs = 65535U;
            }
        }
        else
        {
            s_motorSpeedStableMs = 0U;
        }

        if(s_motorSpeedStableMs < gMotorSpeedCloseLoopStableMs)
        {
            return;
        }

        /*
         * 欠速时延续当前启动电流；已经超速时让PI从0电流接管。
         * 这样不会在高速接管瞬间继续加速，也不会立即施加反向制动。
         */
        error = (s32)target_ramped - feedback;
        p_term_q10 = error * Motor_ControlLimitS32(
            (s32)gMotorSpeedKpQ10, 0L, 32767L);
        if(((target_signed > 0) && (error > 0L)) ||
           ((target_signed < 0) && (error < 0L)))
        {
            takeover_iq_q10 = (s32)motor->m_iq_set << 10;
            s_motorSpeedITermQ10 = takeover_iq_q10 - p_term_q10;
        }
        else
        {
            s_motorSpeedITermQ10 = 0L;
        }
        s_motorSpeedITermQ10 = Motor_ControlLimitS32(
            s_motorSpeedITermQ10, -output_limit_q10, output_limit_q10);
        gMotorSpeedClosedLoopActive = 1U;
        }
    }

    error = (s32)target_ramped - feedback;
    gMotorSpeedErrorErpm = (s16)Motor_ControlLimitS32(
        error, -32768L, 32767L);
    p_term_q10 = error * Motor_ControlLimitS32(
        (s32)gMotorSpeedKpQ10, 0L, 32767L);

    dt_ms = (elapsed_ms > 20U) ? 20L : (s32)elapsed_ms;
    i_delta_q10 = (s32)(((int64_t)error * (int64_t)Motor_ControlLimitS32(
        (s32)gMotorSpeedKiQ10, 0L, 32767L) * (int64_t)dt_ms) / 1000LL);
    i_candidate_q10 = Motor_ControlLimitS32(
        s_motorSpeedITermQ10 + i_delta_q10,
        -output_limit_q10, output_limit_q10);
    output_q10 = p_term_q10 + i_candidate_q10;

    /* 输出饱和且误差仍推动饱和加深时暂停积分，允许反向误差解除饱和。 */
    if(!(((output_q10 > output_max_q10) && (error > 0L)) ||
         ((output_q10 < output_min_q10) && (error < 0L))))
    {
        s_motorSpeedITermQ10 = i_candidate_q10;
    }
    output_q10 = Motor_ControlLimitS32(
        p_term_q10 + s_motorSpeedITermQ10,
        output_min_q10, output_max_q10);
    gMotorSpeedIqCommandMa = (s16)(output_q10 >> 10);
    Motor_SetCurrentTarget(motor, 0, gMotorSpeedIqCommandMa);
}

static void Motor_ControlFaultStop(u8 fault_code)
{
    __disable_irq();
    m_motor.m_control_mode = CONTROL_MODE_NONE;
    PwmAOutputs(DISABLE);
    __enable_irq();
    
    Motor_CurrentCommandReset(&m_motor);
    Motor_SpeedControlReset(true);
    
    gMotorRunEnable = 0U;
    gMotorControlFaultCode = fault_code;
    gMotorControlState = MCS_CONTROL_STATE_FAULT;
    s_motorPwmArmed = false;
}

void Motor_ControlInit(void)
{
    __disable_irq();
    m_motor.m_phase_override = false;
    m_motor.m_control_mode = CONTROL_MODE_NONE;
    PwmAOutputs(DISABLE);
    __enable_irq();
    // 电流环参数初始化
    Motor_CurrentCommandReset(&m_motor);
    Motor_SpeedControlReset(true);
    /* 操作模式默认关闭；学习模式由霍尔学习状态机自动申请运行。 */
    gMotorRunEnable = 0U;
    gMotorControlState = MCS_CONTROL_STATE_OFF;
    gMotorControlFaultCode = 0U;
    s_motorStartElapsedMs = 0U;
    s_motorCurrentOverMs = 0U;
    s_shortFaultCountAtArm = gShortFaultCount;
    s_motorPwmArmed = false;
}

/*
 * 正式电机控制状态机，由主循环中的 1 ms 调度任务调用。
 * 本函数只处理运行命令、启动延时、PWM 启停、保护和最终电流目标；
 * 无感/Hall 角度、电流采样、电流 PI 以及 PWM 更新仍由 ADC 快速环完成。
 *
 * 执行顺序：
 *   1. 检查 MCPWM/MOE 硬件故障；
 *   2. 检查相电流连续超限；
 *   3. 处理停止命令并等待电流斜坡回零；
 *   4. 处理上电启动延时和 PWM 使能；
 *   5. 根据机械方向发布带符号的 q 轴电流目标。
 */
void Motor_ControlTask1ms(u16 elapsed_ms)
{
    hall_learn_drive_mode_t learn_drive_mode;
    bool learning_control;
    bool run_requested;
    bool publish_current_target;
    s32 id_target;
    s32 iq_target;

    if(elapsed_ms == 0U)
    {
        return;
    }

    learning_control = gHallWorkMode == HALL_WORK_MODE_LEARN;
    publish_current_target = true;
    learn_drive_mode = Hall_LearnGetDriveMode();
    if(learning_control)
    {
        run_requested = (learn_drive_mode != HALL_LEARN_DRIVE_STOP) &&
                        (gMotorControlFaultCode == 0U);
    }
    else
    {
        run_requested = gMotorRunEnable != 0U;
    }

    /* PWM 运行后，新增硬件短路事件或 MOE 意外关闭都必须立即停机。 */
    if(s_motorPwmArmed &&
       ((gShortFaultCount != s_shortFaultCountAtArm) ||
        ((MCPWM_FAIL012 & MCS_CONTROL_MOE_MASK) == 0U)))
    {
        Motor_ControlFaultStop(
            (gShortFaultCount != s_shortFaultCountAtArm) ? 2U : 3U);
        return;
    }

    /* 软件过流要求连续超限 3 ms，避免单次 ADC 毛刺造成误停机。 */
    if(s_motorPwmArmed && Motor_ControlCurrentExceeded())
    {
        s_motorCurrentOverMs += elapsed_ms;
        if(s_motorCurrentOverMs >= MCS_CONTROL_CURRENT_TRIP_MS)
        {
            Motor_ControlFaultStop(1U);
            return;
        }
    }
    else
    {
        s_motorCurrentOverMs = 0U;
    }

    /* 正常停止先把目标设为 0，让独立电流斜坡平滑卸载，再关闭 PWM。 */
    if(!run_requested)
    {
        Motor_SetCurrentTarget(&m_motor, 0, 0);
        Motor_SpeedControlReset(true);
        gMotorControlState = MCS_CONTROL_STATE_STOPPING;

        if((m_motor.m_id_set == 0) && (m_motor.m_iq_set == 0))
        {
            __disable_irq();
            m_motor.m_control_mode = CONTROL_MODE_NONE;
            PwmAOutputs(DISABLE);
            __enable_irq();
            gMotorControlState = MCS_CONTROL_STATE_OFF;
            s_motorPwmArmed = false;
            s_motorStartElapsedMs = 0U;
        }
        return;
    }

    /* 尚未启动时先等待功率级和采样稳定，再以零电流使能 PWM。 */
    if(!s_motorPwmArmed)
    {
        gMotorControlState = MCS_CONTROL_STATE_START_DELAY;
        if(s_motorStartElapsedMs < gMotorStartDelayMs)
        {
            s_motorStartElapsedMs += elapsed_ms;
            return;
        }

        if(learning_control)
        {
            m_motor.m_conf->foc_sensor_mode = FOC_SENSOR_MODE_SENSORLESS;
            m_motor.m_phase_override =
                learn_drive_mode == HALL_LEARN_DRIVE_ALIGN;
            if(m_motor.m_phase_override)
            {
                m_motor.m_motor_state.phase = gHallLearnAlignPhase;
            }
        }
        else
        {
            m_motor.m_phase_override = false;
            if((gMotorOperationSensorMode == FOC_SENSOR_MODE_HALL) &&
               Hall_CalibrationIsValid())
            {
                m_motor.m_conf->foc_sensor_mode = FOC_SENSOR_MODE_HALL;
            }
            else
            {
                m_motor.m_conf->foc_sensor_mode = FOC_SENSOR_MODE_SENSORLESS;
            }
        }
        s_shortFaultCountAtArm = gShortFaultCount;
        __disable_irq();
        if(learning_control ||
           (gMotorOperationControlMode == MCS_OPERATION_CONTROL_CURRENT))
        {
            m_motor.m_control_mode = CONTROL_MODE_CURRENT;
        }
        else
        {
            m_motor.m_control_mode = CONTROL_MODE_SPEED;
        }
        PwmAOutputs(ENABLE);
        __enable_irq();
        s_motorPwmArmed = true;
    }

    if(learning_control &&
       (learn_drive_mode == HALL_LEARN_DRIVE_ALIGN))
    {
        /* 固定电角度只施加d轴电流，使转子吸附到已知磁场方向。 */
        m_motor.m_phase_override = true;
        m_motor.m_motor_state.phase = gHallLearnAlignPhase;
        id_target = Motor_ControlAbsS32((s32)gHallLearnAlignCurrentMa);
        iq_target = 0L;
    }
    else
    {
        m_motor.m_phase_override = false;
        id_target = 0L;
        if(learning_control)
        {
            /* 学习模式固定正向无感旋转，不接受串口方向和停止命令。 */
            iq_target = Motor_ControlAbsS32((s32)gHallLearnSpinCurrentMa);
        }
        else
        {
            if(gMotorOperationControlMode == MCS_OPERATION_CONTROL_SPEED)
            {
                /* 速度模式的iq目标在随后的1ms速度外环中发布。 */
                iq_target = 0L;
                publish_current_target = false;
            }
            else
            {
                /* 电流模式由串口方向和固定电流幅值直接生成q轴目标。 */
                iq_target = Motor_ControlAbsS32((s32)gMotorCurrentTargetMa);
                if(gMotorDirection == MCS_MOTOR_DIRECTION_REVERSE)
                {
                    iq_target = -iq_target;
                }
            }
        }
    }
    id_target = Motor_ControlLimitS32(id_target, -32768L, 32767L);
    iq_target = Motor_ControlLimitS32(iq_target, -32768L, 32767L);
    if(publish_current_target)
    {
        Motor_SetCurrentTarget(&m_motor, (s16)id_target, (s16)iq_target);
    }
    gMotorControlState = MCS_CONTROL_STATE_RUNNING;
}



void StopMotorImmdly(void)
{
    /* Emergency-style stop used by legacy control code.
     * Keep it simple: disable PWM first, then mark the MCS motor sub-state
     * as BRAKE so APP/reporting code can observe that the drive is no longer
     * producing torque.
     */
    Motor_ControlFaultStop(0U);
		
}

