#include "main.h"

extern volatile UINT16 gMcpwmEifAtShort;
extern volatile UINT16 gMcpwmFail012AtShort;
extern volatile UINT16 gCmpDataAtShort;
extern volatile UINT16 gShortFaultCount;

/* 置 1：跳过转子吸附和开环拖动，直接让观测器接管角度。 能满足，可以使用 */
#define DIRECTION_TEST_DIRECT_SENSORLESS   (1U)

/*
 * 电机方向及启动测试任务。
 *
 * 主要观察变量：
 *   gDirectionTestMode
 *     0：关闭 PWM，写入中性矢量
 *     1：直接给定 d 轴电压
 *     2：直接给定 q 轴电压
 *     3：d 轴电流环测试
 *     4：q 轴电流环测试
 *     5：自动无感电流控制启动
 *
 * 本文件按 1 ms 周期运行，只负责准备控制模式、角度归属和电流目标值；
 * 真正的电流 PI 和 PWM 更新在 ADC 中断中执行。
 */
volatile u8 gDirectionTestMode = 0U;
volatile u16 gDirectionTestAngle = 0U;
volatile s16 gDirectionTestDcmd = 20;
volatile s16 gDirectionTestQcmd = 20;
volatile s16 gDirectionTestIdRefMa = 100;


/* 直接无感启动的转矩电流目标，以 2 mA/ms 的斜率逐步增加。 */
volatile s16 gDirectionTestIqRefMa = 400;

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
 * 自动启动诊断状态：
 *   运行状态 0：停止，1：启动延时，2：转子吸附，3：开环旋转，
 *            4：软件过流停机，5：硬件 PWM 故障。
 *   故障代码 0：无故障，1：软件电流限制，2：MCPWM 短路/故障中断，
 *            3：MOE 被意外清除。
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

volatile s16 gOpenloopVoltageCmd = 30;
volatile s16 gOpenloopStartVoltageCmd = 30;
volatile s16 gOpenloopAlignVoltageCmd = 20;
volatile s16 gOpenloopTargetErpm = 800;
volatile u16 gOpenloopRampErpmPerSecond = 600U;
volatile u16 gOpenloopAlignTimeMs = 1000U;
volatile u16 gOpenloopCurrentLimitMa = 1500U;
volatile s16 gOpenloopActualErpm;
volatile u16 gOpenloopAngle;
volatile s16 gOpenloopAngleStep;
volatile u16 gOpenloopAlignElapsedMs;
volatile u8 gOpenloopCurrentTrip;
volatile u8 gOpenloopAutoStartEnable;
volatile u16 gOpenloopAutoStartDelayMs = 500U;
volatile u16 gOpenloopAutoStartElapsedMs;
volatile u16 gOpenloopCurrentOverMs;
volatile u16 gOpenloopObserverAngle;
volatile s16 gOpenloopObserverPhaseError;
volatile s16 gOpenloopObserverPhaseErrorFiltered;
volatile u16 gOpenloopObserverPhaseJitter;
volatile u16 gOpenloopObserverFluxUwb;
volatile s16 gOpenloopObserverSpeedErpm;
volatile u16 gOpenloopObserverStableMs;
volatile u8 gOpenloopObserverTracking;
volatile u16 gOpenloopPullElapsedMs;
volatile u16 gOpenloopFluxErrorUwb;
volatile u16 gOpenloopObserverPredictedAngle;
volatile s16 gOpenloopObserverPredictionAdvance;
volatile u8 gOpenloopSensorlessEnable = 1U;
volatile u8 gOpenloopSensorlessActive;
volatile u16 gOpenloopSensorlessAngle;
volatile s16 gOpenloopSensorlessPhaseOffset;
volatile u16 gOpenloopSensorlessDropMs;
volatile u16 gOpenloopSensorlessEnterCount;
volatile u8 gOpenloopCurrentControlActive;
volatile u16 gOpenloopSensorlessTransitionMs;
volatile s16 gOpenloopCurrentIqCommandMa;

static u8 s_directionTestLastMode = 0xFFU;
#if !DIRECTION_TEST_DIRECT_SENSORLESS
static s32 s_openloopPhaseNumerator;
static u32 s_openloopRampNumerator;
#endif
static bool s_openloopObserverSampleValid;
static u16 s_shortFaultCountAtArm;
static bool s_pwmArmAttempted;

#define DIRECTION_TEST_OPENLOOP_MODE       (5U)
#define OPENLOOP_MAX_ABS_ERPM              (3000)
#define OPENLOOP_MAX_VOLTAGE_CMD           ((s16)(PWM_PERIOD / 4U))
#define OPENLOOP_MIN_PULL_TIME_MS            (1000U)
#define OPENLOOP_MAX_FLUX_ERROR_UWB          (100U)
#define OPENLOOP_OBSERVER_STABLE_TIME_MS     (100U)
#define OPENLOOP_OBSERVER_PREDICT_US         (750L)
#define OPENLOOP_CURRENT_TRIP_DEBOUNCE_MS   (3U)
#define OPENLOOP_SENSORLESS_TRANSITION_MS   (200U)
#define OPENLOOP_IQ_RAMP_MA_PER_MS          (2L)
#define DIRECTION_TEST_MOE_MASK              (0x0040U)

void Direction_Init(void)
{
    gOpenloopVoltageCmd = 30;
    gOpenloopStartVoltageCmd = 30;
    gOpenloopAlignVoltageCmd = 20;
    gOpenloopTargetErpm = 800;
    gOpenloopRampErpmPerSecond = 600;
#if DIRECTION_TEST_DIRECT_SENSORLESS
    gOpenloopAlignTimeMs = 0U;
#else
    gOpenloopAlignTimeMs = 1000U;
#endif
    gOpenloopCurrentLimitMa = 2000U;
    gOpenloopAutoStartElapsedMs = 0U;
    gOpenloopAutoStartEnable = 1U;
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
    if(value >= 0)
    {
        return (u32)value;
    }

    return (u32)(-(value + 1L)) + 1UL;
}

static s16 DirectionTest_ClampS16(s32 value, s32 min, s32 max)
{
    if(value > max)
    {
        value = max;
    }
    else if(value < min)
    {
        value = min;
    }

    return (s16)value;
}

static void DirectionTest_ResetOpenloop(void)
{
    gOpenloopActualErpm = 0;
    gOpenloopAngle = gDirectionTestAngle;
    gOpenloopAngleStep = 0;
    gOpenloopAlignElapsedMs = 0U;
    gOpenloopCurrentTrip = 0U;
    gOpenloopCurrentOverMs = 0U;
    gOpenloopObserverAngle = 0U;
    gOpenloopObserverPhaseError = 0;
    gOpenloopObserverPhaseErrorFiltered = 0;
    gOpenloopObserverPhaseJitter = 0U;
    gOpenloopObserverFluxUwb = 0U;
    gOpenloopObserverSpeedErpm = 0;
    gOpenloopObserverStableMs = 0U;
    gOpenloopObserverTracking = 0U;
    gOpenloopPullElapsedMs = 0U;
    gOpenloopFluxErrorUwb = 0U;
    gOpenloopObserverPredictedAngle = 0U;
    gOpenloopObserverPredictionAdvance = 0;
    gOpenloopSensorlessActive = 0U;
    gOpenloopSensorlessAngle = gOpenloopAngle;
    gOpenloopSensorlessPhaseOffset = 0;
    gOpenloopSensorlessDropMs = 0U;
    gOpenloopSensorlessEnterCount = 0U;
    gOpenloopCurrentControlActive = 0U;
    gOpenloopSensorlessTransitionMs = 0U;
    gOpenloopCurrentIqCommandMa = 0;
    gDirectionTestIdFilteredMa = 0;
    gDirectionTestIqFilteredMa = 0;
    gDirectionTestIqErrFilteredMa = 0;
#if !DIRECTION_TEST_DIRECT_SENSORLESS
    s_openloopPhaseNumerator = 0;
    s_openloopRampNumerator = 0U;
#endif
    s_openloopObserverSampleValid = false;
    foc_observer_reset(&m_motor.m_observer_state);

#if DIRECTION_TEST_DIRECT_SENSORLESS
    /*
     * 直接无感启动实验：复位后立即认为角度接管已经完成。
     * 不施加 d 轴吸附电压，也不生成旋转的开环角度。零速时观测角可能不确定，
     * 因此在转子运动产生可测反电动势之前，可能出现短暂的来回摆动。
     * q轴电流>=400ma时可以满足直接启动
     */
    gOpenloopSensorlessActive = 1U;
    gOpenloopSensorlessAngle = (u16)m_motor.m_phase_now_observer;
    gOpenloopSensorlessTransitionMs = OPENLOOP_SENSORLESS_TRANSITION_MS;
    gOpenloopSensorlessEnterCount = 1U;
#endif
}

static bool DirectionTest_OpenloopCurrentExceeded(void)
{
    u32 currentMax;

    if(gOpenloopCurrentLimitMa == 0U)
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

    return currentMax > (u32)gOpenloopCurrentLimitMa;
}

#if !DIRECTION_TEST_DIRECT_SENSORLESS
static void DirectionTest_UpdateOpenloopSpeed(void)
{
    s32 targetErpm;
    s32 speedStep;
    s32 speedError;

    targetErpm = (s32)DirectionTest_ClampS16(
        (s32)gOpenloopTargetErpm,
        -OPENLOOP_MAX_ABS_ERPM,
        OPENLOOP_MAX_ABS_ERPM);

    if(gOpenloopRampErpmPerSecond == 0U)
    {
        gOpenloopActualErpm = (s16)targetErpm;
        s_openloopRampNumerator = 0U;
        return;
    }

    s_openloopRampNumerator += (u32)gOpenloopRampErpmPerSecond;
    speedStep = (s32)(s_openloopRampNumerator / 1000U);
    s_openloopRampNumerator %= 1000U;
    if(speedStep == 0)
    {
        return;
    }

    speedError = targetErpm - (s32)gOpenloopActualErpm;
    if(speedError > 0)
    {
        if(speedStep > speedError)
        {
            speedStep = speedError;
        }
        gOpenloopActualErpm = (s16)((s32)gOpenloopActualErpm + speedStep);
    }
    else if(speedError < 0)
    {
        if(speedStep > -speedError)
        {
            speedStep = -speedError;
        }
        gOpenloopActualErpm = (s16)((s32)gOpenloopActualErpm - speedStep);
    }
}

static void DirectionTest_AdvanceOpenloopAngle(void)
{
    s32 angleStep;

    /* 一个电气周期为 65536 个角度计数，本任务每 1 ms 执行一次。 */
    s_openloopPhaseNumerator += (s32)gOpenloopActualErpm * 65536L;
    angleStep = s_openloopPhaseNumerator / 60000L;
    s_openloopPhaseNumerator -= angleStep * 60000L;
    gOpenloopAngleStep = DirectionTest_ClampS16(
        angleStep, -32768L, 32767L);
    gOpenloopAngle = (u16)((s32)gOpenloopAngle + angleStep);
}
#endif

static void DirectionTest_UpdateObserverMonitor(void)
{
    s32 phaseFilterStep;
    s32 phaseAdvance;
    s32 fluxError;
    u32 fluxSquare;

    /* 仅用于诊断显示，这些变量不直接参与快速电流环计算。 */
    gOpenloopObserverAngle = (u16)m_motor.m_phase_now_observer;
    gOpenloopObserverPhaseError =
        (s16)(gOpenloopObserverAngle - gOpenloopAngle);

    fluxSquare = (u32)((int64_t)m_motor.m_observer_state.x1 *
                       m_motor.m_observer_state.x1 +
                       (int64_t)m_motor.m_observer_state.x2 *
                       m_motor.m_observer_state.x2);
    gOpenloopObserverFluxUwb = (u16)utils_sqrt_u32(fluxSquare);
    fluxError = (s32)gOpenloopObserverFluxUwb -
                m_motor.m_conf->foc_motor_flux_linkage;
    gOpenloopFluxErrorUwb = (u16)DirectionTest_AbsS32(fluxError);

    gOpenloopObserverSpeedErpm = m_motor.m_pll_speed;
    phaseAdvance = (s32)((int64_t)gOpenloopObserverSpeedErpm *
        65536LL * OPENLOOP_OBSERVER_PREDICT_US / 60000000LL);
    gOpenloopObserverPredictionAdvance = DirectionTest_ClampS16(
        phaseAdvance, -32768L, 32767L);
    /* 对 PLL 角度进行超前补偿，以抵消计算和 PWM 更新延时。 */
    gOpenloopObserverPredictedAngle = (u16)(
        (u16)m_motor.m_pll_phase +
        (s32)gOpenloopObserverPredictionAdvance);

    if(!s_openloopObserverSampleValid)
    {
        gOpenloopObserverPhaseErrorFiltered = gOpenloopObserverPhaseError;
        gOpenloopObserverPhaseJitter = 0U;
        s_openloopObserverSampleValid = true;
    }
    else
    {
        phaseFilterStep = (s16)(gOpenloopObserverPhaseError -
                                      gOpenloopObserverPhaseErrorFiltered);
        gOpenloopObserverPhaseErrorFiltered = (s16)(
            (s32)gOpenloopObserverPhaseErrorFiltered + phaseFilterStep / 8L);
        gOpenloopObserverPhaseJitter = (u16)DirectionTest_AbsS32((s16)(
            gOpenloopObserverPhaseError - gOpenloopObserverPhaseErrorFiltered));
    }

    if((gOpenloopPullElapsedMs >= OPENLOOP_MIN_PULL_TIME_MS) &&
       (gOpenloopFluxErrorUwb <= OPENLOOP_MAX_FLUX_ERROR_UWB))
    {
        if(gOpenloopObserverStableMs < OPENLOOP_OBSERVER_STABLE_TIME_MS)
        {
            gOpenloopObserverStableMs++;
        }
    }
    else
    {
        gOpenloopObserverStableMs = 0U;
    }
    gOpenloopObserverTracking =
        (gOpenloopObserverStableMs >=
         OPENLOOP_OBSERVER_STABLE_TIME_MS) ? 1U : 0U;
}

static void DirectionTest_UpdateSensorlessAngle(void)
{
    s32 phaseOffset;
    u16 transitionRemaining;

    if(gOpenloopSensorlessEnable == 0U)
    {
        gOpenloopSensorlessActive = 0U;
        gOpenloopSensorlessAngle = gOpenloopAngle;
        gOpenloopSensorlessTransitionMs = 0U;
        return;
    }

    /* 该进入条件仅供可选的开环启动路径使用。 */
    if((gOpenloopSensorlessActive == 0U) &&
       (gOpenloopObserverTracking != 0U))
    {
        /* 接管瞬间保留原旋转电压矢量角度，避免角度突变。 */
        gOpenloopSensorlessPhaseOffset = (s16)(
            gOpenloopAngle - gOpenloopObserverPredictedAngle);
        gOpenloopSensorlessTransitionMs = 0U;
        gOpenloopSensorlessActive = 1U;
        gOpenloopSensorlessEnterCount++;
    }

    gOpenloopSensorlessDropMs = 0U;
    if(gOpenloopSensorlessActive != 0U)
    {
        if(gOpenloopSensorlessTransitionMs <
           OPENLOOP_SENSORLESS_TRANSITION_MS)
        {
            transitionRemaining = OPENLOOP_SENSORLESS_TRANSITION_MS -
                                  gOpenloopSensorlessTransitionMs;
            phaseOffset = gOpenloopSensorlessPhaseOffset;
            phaseOffset -= phaseOffset / (s32)transitionRemaining;
            gOpenloopSensorlessPhaseOffset = (s16)phaseOffset;
            gOpenloopSensorlessTransitionMs++;
        }
        else
        {
            gOpenloopSensorlessPhaseOffset = 0;
        }

        /* 最终指令角度 = PLL 预测角度 + 逐渐衰减的接管偏差。 */
        gOpenloopSensorlessAngle = (u16)(
            gOpenloopObserverPredictedAngle +
            gOpenloopSensorlessPhaseOffset);
    }
    else
    {
        gOpenloopSensorlessAngle = gOpenloopAngle;
    }
}

static void DirectionTest_CalcPark(u16 angle)
{
    s32 alpha;
    s32 beta;

    (void)angle;
    alpha = m_motor.m_motor_state.i_alpha;
    beta = m_motor.m_motor_state.i_beta;

    gDirectionTestAlphaMa = alpha;
    gDirectionTestBetaMa = beta;
    gDirectionTestIdMa = m_motor.m_motor_state.id;
    gDirectionTestIqMa = m_motor.m_motor_state.iq;
    gDirectionTestIdFilteredMa +=
        (gDirectionTestIdMa - gDirectionTestIdFilteredMa) / 8L;
    gDirectionTestIqFilteredMa +=
        (gDirectionTestIqMa - gDirectionTestIqFilteredMa) / 8L;
}

void Motor_DirectionTest_Stop(void)
{
    PwmAOutputs(DISABLE);
    m_motor.m_phase_override = false;
    m_motor.m_control_mode = CONTROL_MODE_NONE;
    m_motor.m_id_set = 0;
    m_motor.m_iq_set = 0;
    gDirectionTestVdCmd = 0;
    gDirectionTestVqCmd = 0;
    gDirectionTestIqErrFilteredMa = 0;
    gOpenloopCurrentControlActive = 0U;
    gOpenloopSensorlessTransitionMs = 0U;
    gOpenloopCurrentIqCommandMa = 0;
    gOpenloopActualErpm = 0;
    gOpenloopAlignElapsedMs = 0U;
#if !DIRECTION_TEST_DIRECT_SENSORLESS
    s_openloopPhaseNumerator = 0;
    s_openloopRampNumerator = 0U;
#endif
    DirectionTest_ResetPi();
    Motor_WriteDqVector(gDirectionTestAngle, 0, 0);
    m_motor.m_state = MC_STATE_OFF;
}

void Motor_DirectionTest_Task(void)
{
    bool modeChanged;
    bool openloopAligning;
    bool sensorlessCurrentReady;
    mc_control_mode controlModeNext;
    u16 controlAngle;
    s16 openloopVoltage;
    s32 iqCommand;
    s32 iqTarget;

    /*
     * 这是 1 ms 启动/外层任务。在电流模式下不会自行计算 SVM，只向下一次
     * ADC 快速环发布 iq 目标和角度归属。
     */
    gDirectionTestTaskCount++;
    DirectionTest_UpdatePwmDiagnostics();

    if(gDirectionTestMode > DIRECTION_TEST_OPENLOOP_MODE) // 目前只有5次
    {
        gDirectionTestMode = 0U;
    }

    modeChanged = (gDirectionTestMode != s_directionTestLastMode);

    /* 硬件故障中断会关闭 MOE，但不会主动修改测试模式。 */
    if((gDirectionTestMode == DIRECTION_TEST_OPENLOOP_MODE) &&
       s_pwmArmAttempted &&
       ((gShortFaultCount != s_shortFaultCountAtArm) ||
        (gDirectionTestMoeEnabled == 0U)))
    {
        gDirectionTestFaultCode =
            (gShortFaultCount != s_shortFaultCountAtArm) ? 2U : 3U;
        gDirectionTestRunState = 5U;
        gOpenloopAutoStartEnable = 0U;
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

        if(gOpenloopAutoStartEnable != 0U)
        {
            gDirectionTestRunState = 1U;
            if(gOpenloopAutoStartElapsedMs < gOpenloopAutoStartDelayMs)
            {
                gOpenloopAutoStartElapsedMs++;
            }
            else
            {
                gOpenloopAutoStartEnable = 0U;
                gDirectionTestMode = DIRECTION_TEST_OPENLOOP_MODE;
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
        /* 新控制矢量准备完成之前，保持 ADC 快速控制路径停止。 */
        __disable_irq();
        PwmAOutputs(DISABLE);
        m_motor.m_control_mode = CONTROL_MODE_NONE;
        __enable_irq();

        DirectionTest_ResetPi();
        if(gDirectionTestMode == DIRECTION_TEST_OPENLOOP_MODE)
        {
            gDirectionTestFaultCode = 0U;
            s_pwmArmAttempted = false;
            DirectionTest_ResetOpenloop();
        }
        else
        {
            gOpenloopActualErpm = 0;
            gOpenloopAlignElapsedMs = 0U;
        }
        s_directionTestLastMode = gDirectionTestMode;
    }

    controlAngle = gDirectionTestAngle;
    openloopAligning = false;
    sensorlessCurrentReady = false;
    if(gDirectionTestMode == DIRECTION_TEST_OPENLOOP_MODE)
    {
        if(DirectionTest_OpenloopCurrentExceeded())
        {
            if(gOpenloopCurrentOverMs < OPENLOOP_CURRENT_TRIP_DEBOUNCE_MS)
            {
                gOpenloopCurrentOverMs++;
            }
            if(gOpenloopCurrentOverMs >= OPENLOOP_CURRENT_TRIP_DEBOUNCE_MS)
            {
                gOpenloopCurrentTrip = 1U;
                gDirectionTestFaultCode = 1U;
                gDirectionTestRunState = 4U;
                gOpenloopAutoStartEnable = 0U;
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
            gOpenloopCurrentOverMs = 0U;
        }

        openloopAligning = gOpenloopAlignElapsedMs < gOpenloopAlignTimeMs;
        if(openloopAligning)
        {
            gDirectionTestRunState = 2U;
            gOpenloopAlignElapsedMs++;
            gOpenloopActualErpm = 0;
            gOpenloopPullElapsedMs = 0U;
        }
        else
        {
            gDirectionTestRunState = 3U;
#if DIRECTION_TEST_DIRECT_SENSORLESS
            gOpenloopActualErpm = 0;
#else
            DirectionTest_UpdateOpenloopSpeed();
            DirectionTest_AdvanceOpenloopAngle();
#endif
            if(gOpenloopPullElapsedMs < 0xFFFFU)
            {
                gOpenloopPullElapsedMs++;
            }
        }
        /* 读取 ADC 中断异步更新的观测器和 PLL 状态。 */
        DirectionTest_UpdateObserverMonitor();
        DirectionTest_UpdateSensorlessAngle();
        controlAngle = gOpenloopSensorlessAngle;
        sensorlessCurrentReady =
            (gOpenloopSensorlessActive != 0U) &&
            (gOpenloopSensorlessTransitionMs >=
             OPENLOOP_SENSORLESS_TRANSITION_MS);
    }

    /* false 表示允许 AdcSampleCal() 使用观测角覆盖当前 phase。 */
    m_motor.m_phase_override = !sensorlessCurrentReady;
    m_motor.m_motor_state.phase = (s16)controlAngle;
    DirectionTest_CalcPark(controlAngle);

    m_motor.m_state = MC_STATE_RUNNING;
    controlModeNext = CONTROL_MODE_NONE;
    gOpenloopCurrentControlActive = 0U;
    gDirectionTestIqErrFilteredMa = 0;

    switch(gDirectionTestMode)
    {
        case 1U:
            controlModeNext = CONTROL_MODE_OPENLOOP_DUTY_PHASE;
            m_motor.m_id_set = 0;
            m_motor.m_iq_set = 0;
            gDirectionTestVdCmd = gDirectionTestDcmd;
            gDirectionTestVqCmd = 0;
            gDirectionTestIqErrMa = 0;
            gDirectionTestIqPiOut = 0;
            Motor_WriteDqVector(gDirectionTestAngle, gDirectionTestDcmd, 0);
            break;

        case 2U:
            controlModeNext = CONTROL_MODE_OPENLOOP_DUTY_PHASE;
            m_motor.m_id_set = 0;
            m_motor.m_iq_set = 0;
            gDirectionTestVdCmd = 0;
            gDirectionTestVqCmd = gDirectionTestQcmd;
            gDirectionTestIqErrMa = 0;
            gDirectionTestIqPiOut = 0;
            Motor_WriteDqVector(gDirectionTestAngle, 0, gDirectionTestQcmd);
            break;

        case 3U:
            controlModeNext = CONTROL_MODE_CURRENT;
            m_motor.m_id_set = gDirectionTestIdRefMa;
            m_motor.m_iq_set = 0;
            gDirectionTestVdCmd = m_motor.m_motor_state.vd;
            gDirectionTestVqCmd = m_motor.m_motor_state.vq;
            gDirectionTestIqErrMa = m_motor.m_motor_state.iq_error;
            gDirectionTestIqErrFilteredMa =
                (s32)m_motor.m_iq_set - gDirectionTestIqFilteredMa;
            gDirectionTestIqPiOut = m_motor.m_motor_state.vq;
            break;

        case 4U:
            controlModeNext = CONTROL_MODE_CURRENT;
            m_motor.m_id_set = 0;
            m_motor.m_iq_set = gDirectionTestIqRefMa;
            gDirectionTestVdCmd = m_motor.m_motor_state.vd;
            gDirectionTestVqCmd = m_motor.m_motor_state.vq;
            gDirectionTestIqErrMa = m_motor.m_motor_state.iq_error;
            gDirectionTestIqErrFilteredMa =
                (s32)m_motor.m_iq_set - gDirectionTestIqFilteredMa;
            gDirectionTestIqPiOut = m_motor.m_motor_state.vq;
            break;

        case DIRECTION_TEST_OPENLOOP_MODE:
            if(!openloopAligning)
            {
                /*
                 * 转矩电流软启动：目标 600 mA、斜率 2 mA/ms，约 300 ms 到达
                 * 目标值，避免一步给定造成转矩突变。
                 */
                iqCommand = gOpenloopCurrentIqCommandMa;
                iqTarget = gDirectionTestIqRefMa;
                if(iqCommand < iqTarget)
                {
                    iqCommand += OPENLOOP_IQ_RAMP_MA_PER_MS;
                    if(iqCommand > iqTarget)
                    {
                        iqCommand = iqTarget;
                    }
                }
                else if(iqCommand > iqTarget)
                {
                    iqCommand -= OPENLOOP_IQ_RAMP_MA_PER_MS;
                    if(iqCommand < iqTarget)
                    {
                        iqCommand = iqTarget;
                    }
                }
                gOpenloopCurrentIqCommandMa = (s16)iqCommand;

                controlModeNext = CONTROL_MODE_CURRENT;
                m_motor.m_id_set = 0;
                m_motor.m_iq_set = gOpenloopCurrentIqCommandMa;
                gOpenloopCurrentControlActive = 1U;
                gDirectionTestVdCmd = m_motor.m_motor_state.vd;
                gDirectionTestVqCmd = m_motor.m_motor_state.vq;
                gDirectionTestIqErrMa = m_motor.m_motor_state.iq_error;
                gDirectionTestIqErrFilteredMa =
                    (s32)m_motor.m_iq_set - gDirectionTestIqFilteredMa;
                gDirectionTestIqPiOut = m_motor.m_motor_state.vq;
            }
            else
            {
                openloopVoltage = DirectionTest_ClampS16(
                    (s32)gOpenloopAlignVoltageCmd,
                    -(s32)OPENLOOP_MAX_VOLTAGE_CMD,
                    (s32)OPENLOOP_MAX_VOLTAGE_CMD);
                controlModeNext = CONTROL_MODE_OPENLOOP_DUTY_PHASE;
                m_motor.m_id_set = 0;
                m_motor.m_iq_set = 0;
                gOpenloopCurrentIqCommandMa = 0;
                gOpenloopCurrentControlActive = 0U;
                gDirectionTestVdCmd = openloopVoltage;
                gDirectionTestVqCmd = 0;
                gDirectionTestIqErrMa = 0;
                gDirectionTestIqErrFilteredMa = 0;
                gDirectionTestIqPiOut = 0;
                Motor_WriteDqVector(controlAngle, openloopVoltage, 0);
            }
            break;

        default:
            gDirectionTestMode = 0U;
            s_directionTestLastMode = 0U;
            Motor_DirectionTest_Stop();
            return;
    }

    /* 在不可中断区内一次性发布控制模式并使能 PWM。 */
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
