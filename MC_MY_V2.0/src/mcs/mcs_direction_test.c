#include "main.h"

extern volatile UINT16 gMcpwmEifAtShort;
extern volatile UINT16 gMcpwmFail012AtShort;
extern volatile UINT16 gCmpDataAtShort;
extern volatile UINT16 gShortFaultCount;

/*
 * 电机方向及基础闭环测试任务，按 1 ms 周期运行。
 *
 *   0：关闭 PWM
 *   1：固定角度，直接给定 d 轴电压
 *   2：固定角度，直接给定 q 轴电压
 *   3：固定角度，测试 d 轴电流环
 *   4：固定角度，测试 q 轴电流环
 *   5：无感角度 + q 轴电流环
 *
 * 本任务只负责启停、测试命令和电流斜坡。无感观测器、PLL、角度选择、
 * 电流 PI 与 PWM 更新全部在 ADC 快速环中完成。
 */
volatile u8 gDirectionTestMode = 0U;
volatile u16 gDirectionTestAngle = 0U;
volatile s16 gDirectionTestDcmd = 20;
volatile s16 gDirectionTestQcmd = 20;
volatile s16 gDirectionTestIdRefMa = 100;
volatile s16 gDirectionTestIqRefMa = 600;

volatile s32 gDirectionTestAlphaMa;
volatile s32 gDirectionTestBetaMa;
volatile s32 gDirectionTestIdMa;
volatile s32 gDirectionTestIqMa;
volatile s32 gDirectionTestIdFilteredMa;
volatile s32 gDirectionTestIqFilteredMa;
volatile s32 gDirectionTestIqErrFilteredMa;
volatile s16 gDirectionTestVdCmd;
volatile s16 gDirectionTestVqCmd;
volatile s32 gDirectionTestIqErrMa;
volatile s32 gDirectionTestIqPiOut;

/*
 * 运行状态：0 停止，1 上电延时，3 运行，4 软件过流，5 硬件 PWM 故障。
 * 故障代码：0 无故障，1 软件过流，2 MCPWM 故障，3 MOE 意外关闭。
 */
volatile u8 gDirectionTestRunState;
volatile u8 gDirectionTestFaultCode;
volatile u8 gDirectionTestMoeEnabled;
volatile u32 gDirectionTestTaskCount;
volatile u16 gDirectionTestPwmEnableCount;
volatile u16 gDirectionTestMoeRegister;
volatile u16 gDirectionTestEifRegister;
volatile s16 gDirectionTestPwmA;
volatile s16 gDirectionTestPwmB;
volatile s16 gDirectionTestPwmC;
volatile u16 gDirectionTestShortFaultCount;

/* 模式 5 的命令、保护和上电自动启动状态。 */
volatile u16 gSensorlessCurrentLimitMa = 2000U;
volatile u8 gSensorlessCurrentTrip;
volatile u8 gSensorlessAutoStartEnable;
volatile u16 gSensorlessAutoStartDelayMs = 500U;
volatile u16 gSensorlessAutoStartElapsedMs;
volatile u16 gSensorlessCurrentOverMs;
volatile u8 gSensorlessCurrentControlActive;

static u8 s_directionTestLastMode = 0xFFU;
static u16 s_shortFaultCountAtArm;
static bool s_pwmArmAttempted;

#define DIRECTION_TEST_SENSORLESS_MODE       (5U)
#define SENSORLESS_CURRENT_TRIP_DEBOUNCE_MS  (3U)
#define DIRECTION_TEST_MOE_MASK               (0x0040U)

void Direction_Init(void)
{
    gSensorlessCurrentLimitMa = 2000U;
    gSensorlessAutoStartElapsedMs = 0U;
    gSensorlessAutoStartEnable = 1U;
    gSensorlessCurrentTrip = 0U;
    gSensorlessCurrentOverMs = 0U;
    gSensorlessCurrentControlActive = 0U;

    gDirectionTestMode = 0U;
    gDirectionTestRunState = 1U;
    gDirectionTestFaultCode = 0U;
    gDirectionTestMoeEnabled = 0U;
    gDirectionTestTaskCount = 0U;
    gDirectionTestPwmEnableCount = 0U;
    gDirectionTestMoeRegister = MCPWM_FAIL012;
    gDirectionTestEifRegister = MCPWM_EIF;
    gDirectionTestPwmA = (s16)MCPWM_TH01;
    gDirectionTestPwmB = (s16)MCPWM_TH11;
    gDirectionTestPwmC = (s16)MCPWM_TH21;
    gDirectionTestShortFaultCount = gShortFaultCount;

    s_shortFaultCountAtArm = gShortFaultCount;
    s_pwmArmAttempted = false;
}

static void DirectionTest_UpdatePwmDiagnostics(void)
{
    gDirectionTestMoeRegister = MCPWM_FAIL012;
    gDirectionTestEifRegister = MCPWM_EIF;
    gDirectionTestMoeEnabled =
        ((MCPWM_FAIL012 & DIRECTION_TEST_MOE_MASK) != 0U) ? 1U : 0U;
    gDirectionTestPwmA = (s16)MCPWM_TH01;
    gDirectionTestPwmB = (s16)MCPWM_TH11;
    gDirectionTestPwmC = (s16)MCPWM_TH21;
    gDirectionTestShortFaultCount = gShortFaultCount;
}

static void DirectionTest_ResetPi(void)
{
    m_motor.m_motor_state.vd_int = 0;
    m_motor.m_motor_state.vq_int = 0;
    m_motor.m_motor_state.vd_int_residual = 0;
    m_motor.m_motor_state.vq_int_residual = 0;
    m_motor.m_motor_state.vd = 0;
    m_motor.m_motor_state.vq = 0;
    m_motor.m_motor_state.id_error = 0;
    m_motor.m_motor_state.iq_error = 0;
}

static u32 DirectionTest_AbsS32(s32 value)
{
    if(value >= 0L)
    {
        return (u32)value;
    }

    return (u32)(-(value + 1L)) + 1UL;
}

static s32 DirectionTest_ApplyMotorDirection(s32 magnitude)
{
    if(magnitude < 0L)
    {
        magnitude = -magnitude;
    }

    if(gMotorDirection == MCS_MOTOR_DIRECTION_REVERSE)
    {
        return -magnitude;
    }

    return magnitude;
}

static void DirectionTest_ResetSensorlessCommand(void)
{
    gSensorlessCurrentTrip = 0U;
    gSensorlessCurrentOverMs = 0U;
    gSensorlessCurrentControlActive = 0U;
    gDirectionTestIdFilteredMa = 0;
    gDirectionTestIqFilteredMa = 0;
    gDirectionTestIqErrFilteredMa = 0;
}

static bool DirectionTest_SensorlessCurrentExceeded(void)
{
    u32 currentMax;

    if(gSensorlessCurrentLimitMa == 0U)
    {
        return false;
    }

    currentMax = DirectionTest_AbsS32((s32)ADC_curr_norm_value[0]);
    if(DirectionTest_AbsS32((s32)ADC_curr_norm_value[1]) > currentMax)
    {
        currentMax = DirectionTest_AbsS32((s32)ADC_curr_norm_value[1]);
    }
    if(DirectionTest_AbsS32((s32)ADC_curr_norm_value[2]) > currentMax)
    {
        currentMax = DirectionTest_AbsS32((s32)ADC_curr_norm_value[2]);
    }

    return currentMax > (u32)gSensorlessCurrentLimitMa;
}

static void DirectionTest_UpdateCurrentMonitor(void)
{
    gDirectionTestAlphaMa = m_motor.m_motor_state.i_alpha;
    gDirectionTestBetaMa = m_motor.m_motor_state.i_beta;
    gDirectionTestIdMa = m_motor.m_motor_state.id;
    gDirectionTestIqMa = m_motor.m_motor_state.iq;
    gDirectionTestIdFilteredMa +=
        (gDirectionTestIdMa - gDirectionTestIdFilteredMa) / 8L;
    gDirectionTestIqFilteredMa +=
        (gDirectionTestIqMa - gDirectionTestIqFilteredMa) / 8L;
}

static void DirectionTest_UpdateClosedLoopMonitor(void)
{
    gDirectionTestVdCmd = m_motor.m_motor_state.vd;
    gDirectionTestVqCmd = m_motor.m_motor_state.vq;
    gDirectionTestIqErrMa = m_motor.m_motor_state.iq_error;
    gDirectionTestIqErrFilteredMa =
        (s32)m_motor.m_iq_set - gDirectionTestIqFilteredMa;
    gDirectionTestIqPiOut = m_motor.m_motor_state.vq;
}

void Motor_DirectionTest_Stop(void)
{
    __disable_irq();
    m_motor.m_phase_override = false;
    m_motor.m_control_mode = CONTROL_MODE_NONE;
    PwmAOutputs(DISABLE);
    __enable_irq();
    Motor_CurrentCommandReset(&m_motor);

    gDirectionTestVdCmd = 0;
    gDirectionTestVqCmd = 0;
    gDirectionTestIqErrMa = 0;
    gDirectionTestIqErrFilteredMa = 0;
    gDirectionTestIqPiOut = 0;
    DirectionTest_ResetSensorlessCommand();
    DirectionTest_ResetPi();
    Motor_WriteDqVector(gDirectionTestAngle, 0, 0);
}

void Motor_DirectionTest_Task(void)
{
    bool modeChanged;
    mc_control_mode controlModeNext;
    s32 qCommand;

    gDirectionTestTaskCount++;
    DirectionTest_UpdatePwmDiagnostics();

    if(gDirectionTestMode > DIRECTION_TEST_SENSORLESS_MODE)
    {
        gDirectionTestMode = 0U;
    }

    modeChanged = (gDirectionTestMode != s_directionTestLastMode);

    /* 硬件故障会关闭 MOE，但不会主动修改测试模式。 */
    if((gDirectionTestMode == DIRECTION_TEST_SENSORLESS_MODE) &&
       s_pwmArmAttempted &&
       ((gShortFaultCount != s_shortFaultCountAtArm) ||
        (gDirectionTestMoeEnabled == 0U)))
    {
        gDirectionTestFaultCode =
            (gShortFaultCount != s_shortFaultCountAtArm) ? 2U : 3U;
        gDirectionTestRunState = 5U;
        gSensorlessAutoStartEnable = 0U;
        gDirectionTestMode = 0U;
        s_directionTestLastMode = 0U;
        s_pwmArmAttempted = false;
        Motor_DirectionTest_Stop();
        DirectionTest_UpdatePwmDiagnostics();
        return;
    }

    if(gDirectionTestMode == 0U)
    {
        s_directionTestLastMode = 0U;
        Motor_DirectionTest_Stop();

        if(gSensorlessAutoStartEnable != 0U)
        {
            gDirectionTestRunState = 1U;
            if(gSensorlessAutoStartElapsedMs < gSensorlessAutoStartDelayMs)
            {
                gSensorlessAutoStartElapsedMs++;
            }
            else
            {
                gSensorlessAutoStartEnable = 0U;
                gDirectionTestMode = DIRECTION_TEST_SENSORLESS_MODE;
            }
        }
        else if(gDirectionTestFaultCode == 0U)
        {
            gDirectionTestRunState = 0U;
        }
        return;
    }

    if(modeChanged)
    {
        /* 新模式准备完成前先关闭 PWM，并让 ADC 快速环退出控制。 */
        __disable_irq();
        PwmAOutputs(DISABLE);
        m_motor.m_control_mode = CONTROL_MODE_NONE;
        __enable_irq();

        DirectionTest_ResetPi();
        Motor_CurrentCommandReset(&m_motor);
        if(gDirectionTestMode == DIRECTION_TEST_SENSORLESS_MODE)
        {
            gDirectionTestFaultCode = 0U;
            s_pwmArmAttempted = false;
            DirectionTest_ResetSensorlessCommand();
        }
        else
        {
            gSensorlessCurrentControlActive = 0U;
        }
        s_directionTestLastMode = gDirectionTestMode;
    }

    if(gDirectionTestMode == DIRECTION_TEST_SENSORLESS_MODE)
    {
        if(DirectionTest_SensorlessCurrentExceeded())
        {
            if(gSensorlessCurrentOverMs < SENSORLESS_CURRENT_TRIP_DEBOUNCE_MS)
            {
                gSensorlessCurrentOverMs++;
            }

            if(gSensorlessCurrentOverMs >= SENSORLESS_CURRENT_TRIP_DEBOUNCE_MS)
            {
                gSensorlessCurrentTrip = 1U;
                gDirectionTestFaultCode = 1U;
                gDirectionTestRunState = 4U;
                gSensorlessAutoStartEnable = 0U;
                gDirectionTestMode = 0U;
                s_directionTestLastMode = 0U;
                s_pwmArmAttempted = false;
                Motor_DirectionTest_Stop();
                DirectionTest_UpdatePwmDiagnostics();
                return;
            }
        }
        else
        {
            gSensorlessCurrentOverMs = 0U;
        }
    }

    /* 模式 1 至 4 使用固定测试角；模式 5 由 ADC 中断发布无感角。 */
    m_motor.m_phase_override =
        (gDirectionTestMode != DIRECTION_TEST_SENSORLESS_MODE);
    if(m_motor.m_phase_override)
    {
        m_motor.m_motor_state.phase = (s16)gDirectionTestAngle;
    }

    DirectionTest_UpdateCurrentMonitor();
    gDirectionTestRunState = 3U;
    controlModeNext = CONTROL_MODE_NONE;
    gSensorlessCurrentControlActive = 0U;
    gDirectionTestIqErrFilteredMa = 0;

    switch(gDirectionTestMode)
    {
        case 1U:
            controlModeNext = CONTROL_MODE_OPENLOOP_DUTY_PHASE;
            Motor_SetCurrentTarget(&m_motor, 0, 0);
            gDirectionTestVdCmd = gDirectionTestDcmd;
            gDirectionTestVqCmd = 0;
            gDirectionTestIqErrMa = 0;
            gDirectionTestIqPiOut = 0;
            Motor_WriteDqVector(gDirectionTestAngle, gDirectionTestDcmd, 0);
            break;

        case 2U:
            controlModeNext = CONTROL_MODE_OPENLOOP_DUTY_PHASE;
            Motor_SetCurrentTarget(&m_motor, 0, 0);
            qCommand = DirectionTest_ApplyMotorDirection(gDirectionTestQcmd);
            gDirectionTestVdCmd = 0;
            gDirectionTestVqCmd = (s16)qCommand;
            gDirectionTestIqErrMa = 0;
            gDirectionTestIqPiOut = 0;
            Motor_WriteDqVector(gDirectionTestAngle, 0, (s16)qCommand);
            break;

        case 3U:
            controlModeNext = CONTROL_MODE_CURRENT;
            Motor_SetCurrentTarget(&m_motor, gDirectionTestIdRefMa, 0);
            DirectionTest_UpdateClosedLoopMonitor();
            break;

        case 4U:
            controlModeNext = CONTROL_MODE_CURRENT;
            Motor_SetCurrentTarget(&m_motor, 0,
                (s16)DirectionTest_ApplyMotorDirection(gDirectionTestIqRefMa));
            DirectionTest_UpdateClosedLoopMonitor();
            break;

        case DIRECTION_TEST_SENSORLESS_MODE:
            /* 测试任务只发布目标，通用 1 ms 控制层负责电流斜坡。 */
            controlModeNext = CONTROL_MODE_CURRENT;
            Motor_SetCurrentTarget(&m_motor, 0,
                (s16)DirectionTest_ApplyMotorDirection(gDirectionTestIqRefMa));
            gSensorlessCurrentControlActive = 1U;
            gDirectionTestRunState = 3U;
            DirectionTest_UpdateClosedLoopMonitor();
            break;

        default:
            gDirectionTestMode = 0U;
            s_directionTestLastMode = 0U;
            Motor_DirectionTest_Stop();
            return;
    }

    if(modeChanged)
    {
        s_shortFaultCountAtArm = gShortFaultCount;
        __disable_irq();
        m_motor.m_control_mode = controlModeNext;
        PwmAOutputs(ENABLE);
        gDirectionTestPwmEnableCount++;
        s_pwmArmAttempted = true;
        __enable_irq();
        DirectionTest_UpdatePwmDiagnostics();
    }
    else
    {
        m_motor.m_control_mode = controlModeNext;
    }
}
