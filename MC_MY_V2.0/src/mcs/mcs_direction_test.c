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
 */
volatile u8 gDirectionTestMode = 4U;
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
static u8 s_directionTestLastMode = 0xFFU;

static void DirectionTest_ResetPi(void)
{
    m_motor.m_motor_state.vd_int = 0;
    m_motor.m_motor_state.vq_int = 0;
    m_motor.m_motor_state.vd = 0;
    m_motor.m_motor_state.vq = 0;
    m_motor.m_motor_state.id_error = 0;
    m_motor.m_motor_state.iq_error = 0;
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
    m_motor.m_phase_override = false;
    m_motor.m_control_mode = CONTROL_MODE_NONE;
    m_motor.m_id_set = 0;
    m_motor.m_iq_set = 0;
    gDirectionTestVdCmd = 0;
    gDirectionTestVqCmd = 0;
    DirectionTest_ResetPi();
    Motor_WriteDqVector(gDirectionTestAngle, 0, 0);
    PwmAOutputs(DISABLE);
    m_motor.m_state = MC_STATE_OFF;
}

void Motor_DirectionTest_Task(void)
{
    /* 方向测试使用固定电角度，不允许无感观测角接管。 */
    m_motor.m_phase_override = true;
    DirectionTest_CalcPark(gDirectionTestAngle);

    if(gDirectionTestMode != s_directionTestLastMode)
    {
        DirectionTest_ResetPi();
        s_directionTestLastMode = gDirectionTestMode;
    }

    if(gDirectionTestMode == 0U)
    {
        Motor_DirectionTest_Stop();
        return;
    }

    m_motor.m_state = MC_STATE_RUNNING;
    PwmAOutputs(ENABLE);

    switch(gDirectionTestMode)
    {
        case 1U:
            m_motor.m_control_mode = CONTROL_MODE_NONE;
            DirectionTest_ResetPi();
            gDirectionTestVdCmd = gDirectionTestDcmd;
            gDirectionTestVqCmd = 0;
            Motor_WriteDqVector(gDirectionTestAngle, gDirectionTestDcmd, 0);
            break;

        case 2U:
            m_motor.m_control_mode = CONTROL_MODE_NONE;
            DirectionTest_ResetPi();
            gDirectionTestVdCmd = 0;
            gDirectionTestVqCmd = gDirectionTestQcmd;
            Motor_WriteDqVector(gDirectionTestAngle, 0, gDirectionTestQcmd);
            break;

        case 3U:
            m_motor.m_control_mode = CONTROL_MODE_CURRENT;
            m_motor.m_id_set = gDirectionTestIdRefMa;
            m_motor.m_iq_set = 0;
            m_motor.m_motor_state.id_target = m_motor.m_id_set;
            m_motor.m_motor_state.iq_target = m_motor.m_iq_set;
            Motor_FocLoopRun(gDirectionTestAngle);
            gDirectionTestIdMa = m_motor.m_motor_state.id;
            gDirectionTestIqMa = m_motor.m_motor_state.iq;
            gDirectionTestVdCmd = m_motor.m_motor_state.vd;
            gDirectionTestVqCmd = m_motor.m_motor_state.vq;
            gDirectionTestIqErrMa = m_motor.m_motor_state.iq_error;
            gDirectionTestIqPiOut = m_motor.m_motor_state.vq;
            break;

        case 4U:
            m_motor.m_control_mode = CONTROL_MODE_CURRENT;
            m_motor.m_id_set = 0;
            m_motor.m_iq_set = gDirectionTestIqRefMa;
            m_motor.m_motor_state.id_target = m_motor.m_id_set;
            m_motor.m_motor_state.iq_target = m_motor.m_iq_set;
            Motor_FocLoopRun(gDirectionTestAngle);
            gDirectionTestIdMa = m_motor.m_motor_state.id;
            gDirectionTestIqMa = m_motor.m_motor_state.iq;
            gDirectionTestVdCmd = m_motor.m_motor_state.vd;
            gDirectionTestVqCmd = m_motor.m_motor_state.vq;
            gDirectionTestIqErrMa = m_motor.m_motor_state.iq_error;
            gDirectionTestIqPiOut = m_motor.m_motor_state.vq;
            break;

        default:
            gDirectionTestMode = 0U;
            Motor_DirectionTest_Stop();
            break;
    }
}
