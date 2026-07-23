#include "main.h"

#define MCS_CONTROL_CURRENT_TRIP_MS          (3U)
#define MCS_FAULT_RECOVERY_STABLE_MS          (500U)

/* 软件过流必须连续超限，计时只由本模块维护。 */
static u16 s_motorCurrentOverMs;
/* 故障源连续恢复正常的时间，达到门限后只解锁到 OFF，不自动启动。 */
static u16 s_motorFaultRecoveryMs;

static s32 Motor_FaultAbsS32(s32 value)
{
    return (value >= 0L) ? value : -value;
}

static bool Motor_FaultCurrentExceeded(void)
{
    s32 current_max;

    if(gMotorCurrentLimitMa == 0U)
    {
        return false;
    }

    current_max = Motor_FaultAbsS32((s32)ADC_curr_norm_value[0]);
    if(Motor_FaultAbsS32((s32)ADC_curr_norm_value[1]) > current_max)
    {
        current_max = Motor_FaultAbsS32((s32)ADC_curr_norm_value[1]);
    }
    if(Motor_FaultAbsS32((s32)ADC_curr_norm_value[2]) > current_max)
    {
        current_max = Motor_FaultAbsS32((s32)ADC_curr_norm_value[2]);
    }

    return current_max > (s32)gMotorCurrentLimitMa;
}

static bool Motor_FaultRecoveryIsSafe(void)
{
    /* 故障恢复判断期间功率输出必须保持关闭。 */
    if(Motor_IsPwmEnabled())
    {
        return false;
    }

    /* MCPWM Fail 仍有待处理事件时，硬件故障源尚未稳定消失。 */
    if((MCPWM_EIF & (BIT4 | BIT5)) != 0U)
    {
        return false;
    }

    /* ADC 采样电流也必须回到软件保护门限以内。 */
    return !Motor_FaultCurrentExceeded();
}

void Motor_FaultInit(void)
{
    m_motor.m_fault_code = MOTOR_FAULT_NONE;
    s_motorCurrentOverMs = 0U;
    s_motorFaultRecoveryMs = 0U;
}

bool Motor_FaultIsActive(void)
{
    return m_motor.m_fault_code != MOTOR_FAULT_NONE;
}

/*
 * 所有软件故障最终都从这里关断功率级并锁存状态。
 * 硬件比较器仍会先通过 MCPWM Fail 链路关闭 MOE，本函数负责整理软件状态。
 */
void Motor_FaultTrip(motor_fault_t fault)
{
    __disable_irq();
    m_motor.m_control_mode = CONTROL_MODE_NONE;
    m_motor.m_speed_pid_set_rpm = 0;
    m_motor.m_id_set_target = 0;
    m_motor.m_iq_set_target = 0;
    m_motor.m_id_set = 0;
    m_motor.m_iq_set = 0;
    m_motor.m_fault_code = fault;
    m_motor.m_run_state = MOTOR_RUN_STATE_FAULT;
    PwmAOutputs(DISABLE);
    __enable_irq();

    gMotorCommand.run = 0U;
    s_motorCurrentOverMs = 0U;
    s_motorFaultRecoveryMs = 0U;
}

/*
 * 1 ms 故障监控。只负责判断和锁存故障，不处理学习模式、速度环或正常启停。
 */
void Motor_FaultTask1ms(u16 elapsed_ms)
{
    if(elapsed_ms == 0U)
    {
        return;
    }

    if(Motor_FaultIsActive())
    {
        if(Motor_FaultRecoveryIsSafe())
        {
            if((elapsed_ms < MCS_FAULT_RECOVERY_STABLE_MS) &&
               (s_motorFaultRecoveryMs <
                (u16)(MCS_FAULT_RECOVERY_STABLE_MS - elapsed_ms)))
            {
                s_motorFaultRecoveryMs += elapsed_ms;
            }
            else
            {
                /*
                 * 只解除故障锁存并回到 OFF。gMotorCommand.run 在故障发生时
                 * 已经清零，因此没有新的串口命令变化就不会重新开启 PWM。
                 */
                __disable_irq();
                m_motor.m_control_mode = CONTROL_MODE_NONE;
                m_motor.m_phase_override = false;
                m_motor.m_fault_code = MOTOR_FAULT_NONE;
                m_motor.m_run_state = MOTOR_RUN_STATE_OFF;
                __enable_irq();
                s_motorFaultRecoveryMs = 0U;
            }
        }
        else
        {
            s_motorFaultRecoveryMs = 0U;
        }
        return;
    }

    s_motorFaultRecoveryMs = 0U;

    /*
     * PWM 未输出是 OFF、START_DELAY 和 FAULT 等状态的正常结果，不单独
     * 定义为故障。没有活动控制或 MOE 已关闭时，只清除软件过流计时。
     */
    if(!Motor_IsControlActive(&m_motor) ||
       !Motor_IsPwmEnabled())
    {
        s_motorCurrentOverMs = 0U;
        return;
    }

    /* 软件相电流必须连续超限 3 ms，避免单次 ADC 毛刺造成误停机。 */
    if(Motor_FaultCurrentExceeded())
    {
        if(s_motorCurrentOverMs < (u16)(65535U - elapsed_ms))
        {
            s_motorCurrentOverMs += elapsed_ms;
        }
        else
        {
            s_motorCurrentOverMs = 65535U;
        }

        if(s_motorCurrentOverMs >= MCS_CONTROL_CURRENT_TRIP_MS)
        {
            Motor_FaultTrip(MOTOR_FAULT_SOFTWARE_OVERCURRENT);
        }
    }
    else
    {
        s_motorCurrentOverMs = 0U;
    }
}

