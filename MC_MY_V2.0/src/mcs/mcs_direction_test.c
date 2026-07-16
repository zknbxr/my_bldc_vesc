#include "main.h"

/*
 * Direction test task.
 *
 * Watch variables:
 *   gDirectionTestMode
 *     0: PWM off, write neutral vector
 *     1: direct d-axis voltage command
 *     2: direct q-axis voltage command
 *     3: d-axis current-loop command
 *     4: q-axis current-loop command
 *     5: rotating open-loop voltage vector
 */
volatile u8 gDirectionTestMode = 0U;
volatile u16 gDirectionTestAngle = 0U;
volatile s16 gDirectionTestDcmd = 20;
volatile s16 gDirectionTestQcmd = 20;
volatile s16 gDirectionTestIdRefMa = 100;
volatile s16 gDirectionTestIqRefMa = 100;

volatile s32 gDirectionTestAlphaMa;
volatile s32 gDirectionTestBetaMa;
volatile s32 gDirectionTestIdMa;
volatile s32 gDirectionTestIqMa;
volatile s16 gDirectionTestVdCmd;
volatile s16 gDirectionTestVqCmd;
volatile s32 gDirectionTestIqErrMa;
volatile s32 gDirectionTestIqPiOut;

volatile s16 gOpenloopVoltageCmd = 30;
volatile s16 gOpenloopTargetErpm = 60;
volatile u16 gOpenloopRampErpmPerSecond = 30U;
volatile u16 gOpenloopAlignTimeMs = 1000U;
volatile u16 gOpenloopCurrentLimitMa = 1500U;
volatile s16 gOpenloopActualErpm;
volatile u16 gOpenloopAngle;
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

static u8 s_directionTestLastMode = 0xFFU;
static s32 s_openloopPhaseNumerator;
static u32 s_openloopRampNumerator;
static u16 s_openloopObserverAngleLast;
static bool s_openloopObserverSampleValid;

#define DIRECTION_TEST_OPENLOOP_MODE       (5U)
#define OPENLOOP_MAX_ABS_ERPM              (3000)
#define OPENLOOP_MAX_VOLTAGE_CMD           ((s16)(PWM_PERIOD / 4U))
#define OPENLOOP_OBSERVER_MIN_ERPM          (60)
#define OPENLOOP_OBSERVER_MAX_SPEED_ERR     (30)
#define OPENLOOP_OBSERVER_MAX_JITTER        (4096U)
#define OPENLOOP_OBSERVER_STABLE_TIME_MS    (300U)
#define OPENLOOP_CURRENT_TRIP_DEBOUNCE_MS   (3U)

void Direction_Init(void)
{
    gOpenloopVoltageCmd = 30;
    gOpenloopTargetErpm = 180;
    gOpenloopRampErpmPerSecond = 120;
    gOpenloopAlignTimeMs = 500U;
    gOpenloopCurrentLimitMa = 1000U;
    gOpenloopAutoStartElapsedMs = 0U;
    gOpenloopAutoStartEnable = 1U;
    gDirectionTestMode = 5U;
}

static void DirectionTest_ResetPi(void)
{
    m_motor.m_motor_state.vd_int = 0;
    m_motor.m_motor_state.vq_int = 0;
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
    s_openloopPhaseNumerator = 0;
    s_openloopRampNumerator = 0U;
    s_openloopObserverAngleLast = 0U;
    s_openloopObserverSampleValid = false;
    foc_observer_reset(&m_motor.m_observer_state);
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

    /* One electrical turn is 65536 angle counts; this task runs every 1 ms. */
    s_openloopPhaseNumerator += (s32)gOpenloopActualErpm * 65536L;
    angleStep = s_openloopPhaseNumerator / 60000L;
    s_openloopPhaseNumerator -= angleStep * 60000L;
    gOpenloopAngle = (u16)((s32)gOpenloopAngle + angleStep);
}

static void DirectionTest_UpdateObserverMonitor(void)
{
    s32 phaseDiff;
    s32 speedInstant;
    s32 speedFiltered;
    s32 speedError;
    s32 phaseFilterStep;
    s32 fluxMin;
    u32 fluxSquare;
    u32 actualSpeedAbs;
    bool sameDirection;

    gOpenloopObserverAngle = (u16)m_motor.m_phase_now_observer;
    gOpenloopObserverPhaseError =
        (s16)(gOpenloopObserverAngle - gOpenloopAngle);

    fluxSquare = (u32)((int64_t)m_motor.m_observer_state.x1 *
                       m_motor.m_observer_state.x1 +
                       (int64_t)m_motor.m_observer_state.x2 *
                       m_motor.m_observer_state.x2);
    gOpenloopObserverFluxUwb = (u16)utils_sqrt_u32(fluxSquare);

    if(!s_openloopObserverSampleValid)
    {
        s_openloopObserverAngleLast = gOpenloopObserverAngle;
        gOpenloopObserverPhaseErrorFiltered = gOpenloopObserverPhaseError;
        s_openloopObserverSampleValid = true;
        return;
    }

    phaseDiff = (s16)(gOpenloopObserverAngle - s_openloopObserverAngleLast);
    s_openloopObserverAngleLast = gOpenloopObserverAngle;
    speedInstant = (phaseDiff * 60000L) / 65536L;
    speedFiltered = (s32)gOpenloopObserverSpeedErpm +
        (speedInstant - (s32)gOpenloopObserverSpeedErpm) / 8L;
    gOpenloopObserverSpeedErpm = DirectionTest_ClampS16(
        speedFiltered, -32768L, 32767L);

    phaseFilterStep = (s16)(gOpenloopObserverPhaseError -
                                  gOpenloopObserverPhaseErrorFiltered);
    gOpenloopObserverPhaseErrorFiltered = (s16)(
        (s32)gOpenloopObserverPhaseErrorFiltered + phaseFilterStep / 8L);
    gOpenloopObserverPhaseJitter = (u16)DirectionTest_AbsS32((s16)(
        gOpenloopObserverPhaseError - gOpenloopObserverPhaseErrorFiltered));

    actualSpeedAbs = DirectionTest_AbsS32((s32)gOpenloopActualErpm);
    speedError = (s32)gOpenloopObserverSpeedErpm -
                 (s32)gOpenloopActualErpm;
    sameDirection = ((gOpenloopActualErpm > 0) &&
                     (gOpenloopObserverSpeedErpm > 0)) ||
                    ((gOpenloopActualErpm < 0) &&
                     (gOpenloopObserverSpeedErpm < 0));
    fluxMin = m_motor.m_conf->foc_motor_flux_linkage / 2L;

    if((actualSpeedAbs >= OPENLOOP_OBSERVER_MIN_ERPM) &&
       sameDirection &&
       (DirectionTest_AbsS32(speedError) <= OPENLOOP_OBSERVER_MAX_SPEED_ERR) &&
       ((s32)gOpenloopObserverFluxUwb >= fluxMin) &&
       (gOpenloopObserverPhaseJitter <= OPENLOOP_OBSERVER_MAX_JITTER))
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
        (gOpenloopObserverStableMs >= OPENLOOP_OBSERVER_STABLE_TIME_MS) ? 1U : 0U;
}

static void DirectionTest_CalcPark(u16 angle)
{
    MCS_TRIG_Q15 trig;
    s32 alpha;
    s32 beta;

    trig = Motor_GetSinCosQ15(angle);
    alpha = m_motor.m_motor_state.i_alpha;
    beta = m_motor.m_motor_state.i_beta;

    gDirectionTestAlphaMa = alpha;
    gDirectionTestBetaMa = beta;
    gDirectionTestIdMa = ((alpha * trig.cos) + (beta * trig.sin)) >> 15;
    gDirectionTestIqMa = ((beta * trig.cos) - (alpha * trig.sin)) >> 15;
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
    gOpenloopActualErpm = 0;
    gOpenloopAlignElapsedMs = 0U;
    s_openloopPhaseNumerator = 0;
    s_openloopRampNumerator = 0U;
    DirectionTest_ResetPi();
    Motor_WriteDqVector(gDirectionTestAngle, 0, 0);
    m_motor.m_state = MC_STATE_OFF;
}

void Motor_DirectionTest_Task(void)
{
    bool modeChanged;
    u16 controlAngle;
    s16 openloopVoltage;

    if(gDirectionTestMode > DIRECTION_TEST_OPENLOOP_MODE)
    {
        gDirectionTestMode = 0U;
    }

    modeChanged = (gDirectionTestMode != s_directionTestLastMode);

    if(gDirectionTestMode == 0U)
    {
        s_directionTestLastMode = 0U;
        Motor_DirectionTest_Stop();

        if(gOpenloopAutoStartEnable != 0U)
        {
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
        return;
    }

    if(modeChanged)
    {
        /* Disable outputs while changing test type and clear the old PI state. */
        PwmAOutputs(DISABLE);
        DirectionTest_ResetPi();
        if(gDirectionTestMode == DIRECTION_TEST_OPENLOOP_MODE)
        {
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
                gDirectionTestMode = 0U;
                s_directionTestLastMode = 0U;
                Motor_DirectionTest_Stop();
                return;
            }
        }
        else
        {
            gOpenloopCurrentOverMs = 0U;
        }

        if(gOpenloopAlignElapsedMs < gOpenloopAlignTimeMs)
        {
            gOpenloopAlignElapsedMs++;
            gOpenloopActualErpm = 0;
        }
        else
        {
            DirectionTest_UpdateOpenloopSpeed();
            DirectionTest_AdvanceOpenloopAngle();
        }
        DirectionTest_UpdateObserverMonitor();
        controlAngle = gOpenloopAngle;
    }

    m_motor.m_phase_override = true;
    m_motor.m_motor_state.phase = (s16)controlAngle;
    DirectionTest_CalcPark(controlAngle);

    m_motor.m_state = MC_STATE_RUNNING;

    switch(gDirectionTestMode)
    {
        case 1U:
            m_motor.m_control_mode = CONTROL_MODE_OPENLOOP_DUTY_PHASE;
            m_motor.m_id_set = 0;
            m_motor.m_iq_set = 0;
            gDirectionTestVdCmd = gDirectionTestDcmd;
            gDirectionTestVqCmd = 0;
            gDirectionTestIqErrMa = 0;
            gDirectionTestIqPiOut = 0;
            Motor_WriteDqVector(gDirectionTestAngle, gDirectionTestDcmd, 0);
            break;

        case 2U:
            m_motor.m_control_mode = CONTROL_MODE_OPENLOOP_DUTY_PHASE;
            m_motor.m_id_set = 0;
            m_motor.m_iq_set = 0;
            gDirectionTestVdCmd = 0;
            gDirectionTestVqCmd = gDirectionTestQcmd;
            gDirectionTestIqErrMa = 0;
            gDirectionTestIqPiOut = 0;
            Motor_WriteDqVector(gDirectionTestAngle, 0, gDirectionTestQcmd);
            break;

        case 3U:
            m_motor.m_control_mode = CONTROL_MODE_CURRENT;
            m_motor.m_id_set = gDirectionTestIdRefMa;
            m_motor.m_iq_set = 0;
            gDirectionTestVdCmd = m_motor.m_motor_state.vd;
            gDirectionTestVqCmd = m_motor.m_motor_state.vq;
            gDirectionTestIqErrMa = m_motor.m_motor_state.iq_error;
            gDirectionTestIqPiOut = m_motor.m_motor_state.vq;
            break;

        case 4U:
            m_motor.m_control_mode = CONTROL_MODE_CURRENT;
            m_motor.m_id_set = 0;
            m_motor.m_iq_set = gDirectionTestIqRefMa;
            gDirectionTestVdCmd = m_motor.m_motor_state.vd;
            gDirectionTestVqCmd = m_motor.m_motor_state.vq;
            gDirectionTestIqErrMa = m_motor.m_motor_state.iq_error;
            gDirectionTestIqPiOut = m_motor.m_motor_state.vq;
            break;

        case DIRECTION_TEST_OPENLOOP_MODE:
            openloopVoltage = DirectionTest_ClampS16(
                (s32)gOpenloopVoltageCmd,
                -(s32)OPENLOOP_MAX_VOLTAGE_CMD,
                (s32)OPENLOOP_MAX_VOLTAGE_CMD);
            m_motor.m_control_mode = CONTROL_MODE_OPENLOOP_DUTY_PHASE;
            m_motor.m_id_set = 0;
            m_motor.m_iq_set = 0;
            gDirectionTestVdCmd = openloopVoltage;
            gDirectionTestVqCmd = 0;
            gDirectionTestIqErrMa = 0;
            gDirectionTestIqPiOut = 0;
            Motor_WriteDqVector(controlAngle, openloopVoltage, 0);
            break;

        default:
            gDirectionTestMode = 0U;
            s_directionTestLastMode = 0U;
            Motor_DirectionTest_Stop();
            return;
    }

    /* Only a deliberate mode transition can arm PWM after it was disabled. */
    if(modeChanged)
    {
        PwmAOutputs(ENABLE);
    }
}
