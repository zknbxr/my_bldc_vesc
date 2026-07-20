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
/* 控制状态：0 关闭，1 启动延时，2 运行，3 斜坡停止中，4 故障停机。 */
volatile u8 gMotorControlState;
/* 故障码：0 无故障，1 软件过流，2 MCPWM 硬件故障，3 MOE 意外关闭。 */
volatile u8 gMotorControlFaultCode;

/* 已累计的启动等待时间，单位 ms；达到 gMotorStartDelayMs 后使能 PWM。 */
static u16 s_motorStartElapsedMs;
/* 相电流连续超限时间，单位 ms；正常一次便清零，用于滤除单次采样毛刺。 */
static u16 s_motorCurrentOverMs;
/* PWM 使能瞬间保存的硬件短路故障计数，用于判断运行后是否出现新故障。 */
static u16 s_shortFaultCountAtArm;
/* PWM 已正式使能标志；用于区分“尚未启动”和“运行中 MOE 被硬件关闭”。 */
static bool s_motorPwmArmed;

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
    s32 step;
    s16 id_next;
    s16 iq_next;

    if((motor == 0) || (motor->m_conf == 0) || (elapsed_ms == 0U))
    {
        return;
    }

    /* 本次允许变化的电流量 = 每毫秒斜坡值 * 实际经过毫秒数。 */
    step = (s32)motor->m_conf->current_ramp_ma_per_ms * (s32)elapsed_ms;
    if(step <= 0L)
    {
        /* 配置为 0 或负数时关闭斜坡，直接使用目标值。 */
        id_next = motor->m_id_set_target;
        iq_next = motor->m_iq_set_target;
    }
    else
    {
        /* d/q 两轴分别向目标移动，且不会越过目标值。 */
        step = Motor_ControlLimitS32(step, 1L, 32767L);
        id_next = Motor_ControlStepTowards(
            motor->m_id_set, motor->m_id_set_target, step);
        iq_next = Motor_ControlStepTowards(
            motor->m_iq_set, motor->m_iq_set_target, step);
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

static void Motor_ControlFaultStop(u8 fault_code)
{
    __disable_irq();
    m_motor.m_control_mode = CONTROL_MODE_NONE;
    PwmAOutputs(DISABLE);
    __enable_irq();

    Motor_CurrentCommandReset(&m_motor);
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

    Motor_CurrentCommandReset(&m_motor);
    /* 上电默认保持关闭，只有串口运行命令才能进入启动流程。 */
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
    s32 iq_target;

    if(elapsed_ms == 0U)
    {
        return;
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
    if(gMotorRunEnable == 0U)
    {
        Motor_SetCurrentTarget(&m_motor, 0, 0);
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

        m_motor.m_phase_override = false;
        s_shortFaultCountAtArm = gShortFaultCount;
        __disable_irq();
        m_motor.m_control_mode = CONTROL_MODE_CURRENT;
        PwmAOutputs(ENABLE);
        __enable_irq();
        s_motorPwmArmed = true;
    }

    /* 应用层只给电流幅值，这里根据机械方向生成最终带符号的 iq 目标。 */
    iq_target = Motor_ControlAbsS32((s32)gMotorCurrentTargetMa);
    if(gMotorDirection == MCS_MOTOR_DIRECTION_REVERSE)
    {
        iq_target = -iq_target;
    }
    iq_target = Motor_ControlLimitS32(iq_target, -32768L, 32767L);
    Motor_SetCurrentTarget(&m_motor, 0, (s16)iq_target);
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

