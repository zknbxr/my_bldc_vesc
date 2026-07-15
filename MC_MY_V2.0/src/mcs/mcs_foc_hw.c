#include "main.h"

/*
 * ADC results are left aligned by the LKS32 ADC. The coefficients below
 * convert one ADC count to the integer units used by the FOC state.
 */
#define MCS_CURRENT_ADC_TO_MA_Q15          (19802L)
#define MCS_BUS_ADC_TO_MV_Q15              (75600L)
#define MCS_BUS_FILTER_Q15                 (8192L)
#define MCS_PHASE_CURRENT_OVER_ADC         (20295U)
#define MCS_PHASE_CURRENT_RELEASE_ADC      (MCS_PHASE_CURRENT_OVER_ADC >> 1)
#define FOC_TWO_BY_THREE_Q15               (21845L)
#define FOC_CONTROL_DT_US                   ((u16)(1000000UL / PWM_FREQ))

volatile s16 gBUS_Vol_ADC;
volatile u32 gBusVoltageMv;
volatile s16 ADC_curr_raw[3];
volatile s16 ADC_curr_norm_value[3];
volatile s16 gPhaseCurrentAAdc;
volatile s16 gPhaseCurrentBAdc;
volatile u16 gPhaseCurrentPeakAbsAdc;
volatile u32 gPhaseCurrentPeakAbsMa;
volatile u32 gFocAdcNormalCount;
volatile u32 gFocAdcBlockCount;
volatile s16 gOverCurrentPhaseAAdc;
volatile s16 gOverCurrentPhaseBAdc;
volatile u16 gOverCurrentPeakAdc;
volatile u16 gOverCurrentPwmA;
volatile u16 gOverCurrentPwmB;
volatile u16 gOverCurrentPwmC;
volatile u8 gOverCurrentStartState;

INT16 hal1;
INT16 hal2;

static s16 FocHw_SatS16(s32 value)
{
    if(value > 32767L)
    {
        return 32767;
    }

    if(value < -32768L)
    {
        return -32768;
    }

    return (s16)value;
}

static u16 FocHw_AbsS32ToU16(s32 value)
{
    u32 abs_value;

    if(value >= 0)
    {
        abs_value = (u32)value;
    }
    else
    {
        abs_value = (u32)(-(value + 1L)) + 1UL;
    }

    if(abs_value > 32767UL)
    {
        abs_value = 32767UL;
    }

    return (u16)abs_value;
}

static s16 FocHw_AdcToCurrentMa(s32 adc_value)
{
    int64_t current_ma;

    current_ma = ((int64_t)adc_value * MCS_CURRENT_ADC_TO_MA_Q15) >> 15;
    return FocHw_SatS16((s32)current_ma);
}

static u32 FocHw_AdcToBusMv(s16 adc_value)
{
    if(adc_value <= 0)
    {
        return 0UL;
    }

    return (u32)(((int64_t)adc_value * MCS_BUS_ADC_TO_MV_Q15) >> 15);
}

/* Q15 SVM 指令为 1.0 时，alpha-beta 电压等于母线电压的 2/3。 */
static s32 FocHw_ModToVoltageMv(s16 modulation, s32 bus_voltage)
{
    s32 max_alpha_beta_voltage;

    if(bus_voltage <= 0L)
    {
        return 0L;
    }

    max_alpha_beta_voltage = (s32)(
        ((int64_t)bus_voltage * FOC_TWO_BY_THREE_Q15) >> 15);
    return (s32)(((int64_t)modulation * max_alpha_beta_voltage) >> 15);
}

INT16 AcqAdcSampDatPhaseU(void)
{
    return GET_CURRENT_U_SAMPLE_RESULT();
}

INT16 AcqAdcSampDatPhaseV(void)
{
    return GET_CURRENT_V_SAMPLE_RESULT();
}

INT16 AcqAdcSampDatUdc(void)
{
    return GET_UDC_SAMPLE_RESULT();
}

/*
 * ADC fast path used by the basic fixed-point FOC implementation.
 * HFI, MTPA and field weakening remain outside this function. The fixed-point
 * sensorless observer consumes the previous PWM voltage and current sample.
 */
void AdcSampleCal(void)
{
    motor_all_state_t *motor_now;
    motor_state_t *state_now;
    mc_configuration *conf_now;
    MCS_TRIG_Q15 trig;
    s32 raw0;
    s32 raw1;
    s32 curr0_adc;
    s32 curr1_adc;
    s32 curr2_adc;
    s32 curr0;
    s32 curr1;
    s32 curr2;
    s32 id_set_tmp;
    s32 iq_set_tmp;
    s32 current_max_abs;
    s32 iq_max_abs;
    u32 iq_limit_square;

    Motor_FocInit();
    motor_now = &m_motor;
    state_now = &motor_now->m_motor_state;
    conf_now = motor_now->m_conf;

    raw0 = (s32)AcqAdcSampDatPhaseU();
    raw1 = (s32)AcqAdcSampDatPhaseV();
    iAdcRes1 = FocHw_SatS16(raw0);
    iAdcRes2 = FocHw_SatS16(raw1);

    /* VESC naming: raw ADC first, then offset-corrected ADC counts. */
    // adc直接采样值
    motor_now->m_currents_adc[0] = iAdcRes1;
    motor_now->m_currents_adc[1] = iAdcRes2;
    motor_now->m_currents_adc[2] = 0;
    // adc采样值减去零偏
    curr0_adc = raw0 - (s32)conf_now->foc_offsets_current[0];
    curr1_adc = raw1 - (s32)conf_now->foc_offsets_current[1];
    curr2_adc = -(curr0_adc + curr1_adc);

    ADC_curr_raw[0] = FocHw_SatS16(curr0_adc);
    ADC_curr_raw[1] = FocHw_SatS16(curr1_adc);
    ADC_curr_raw[2] = FocHw_SatS16(curr2_adc);

    // adc采样值减去零偏后转换为mA
    curr0 = (s32)FocHw_AdcToCurrentMa(curr0_adc);
    curr1 = (s32)FocHw_AdcToCurrentMa(curr1_adc);
    curr2 = -(curr0 + curr1);

    ADC_curr_norm_value[0] = FocHw_SatS16(curr0);
    ADC_curr_norm_value[1] = FocHw_SatS16(curr1);
    ADC_curr_norm_value[2] = FocHw_SatS16(curr2);

    /* Clarke transform for two-shunt sampling with balanced phase currents. */
    // mA克拉克变换
    state_now->i_alpha = ADC_curr_norm_value[0];
    state_now->i_beta = FocHw_SatS16(
        ((s32)ONE_BY_SQRT3 * ADC_curr_norm_value[0] +
         (s32)TWO_BY_SQRT3 * ADC_curr_norm_value[1]) >> 15);

    /*
     * 电流采样对应上一 PWM 周期，因此这里也使用上一周期的调制电压。
     * 这与 VESC 在电流中断中先更新 observer、再计算新 PWM 的顺序一致。
     */
    state_now->v_alpha = FocHw_ModToVoltageMv(
        state_now->mod_alpha_raw, state_now->v_bus);
    state_now->v_beta = FocHw_ModToVoltageMv(
        state_now->mod_beta_raw, state_now->v_bus);

    if(motor_now->m_control_mode != CONTROL_MODE_NONE)
    {
        foc_observer_update(state_now->v_alpha, state_now->v_beta,
                            state_now->i_alpha, state_now->i_beta,
                            FOC_CONTROL_DT_US,
                            &motor_now->m_observer_state,
                            &motor_now->m_phase_now_observer,
                            motor_now);

        /* 开环启动或调试时由 m_phase_override 保留外部给定角度。 */
        if(motor_now->m_phase_override == false)
        {
            state_now->phase = motor_now->m_phase_now_observer;
        }
    }
    else
    {
        /* 停机时只同步电流历史，避免再次启动时出现很大的 L*delta_i。 */
        motor_now->m_observer_state.i_alpha_last = state_now->i_alpha;
        motor_now->m_observer_state.i_beta_last = state_now->i_beta;
    }

    /* The selected angle source updates state_now->phase before this point. */
    // 相位
    trig = Motor_GetSinCosQ15((u16)state_now->phase);
    state_now->phase_sin = trig.sin;
    state_now->phase_cos = trig.cos;

    id_set_tmp = (s32)motor_now->m_id_set;
    iq_set_tmp = (s32)motor_now->m_iq_set;
    current_max_abs = utils_max_abs((s32)conf_now->lo_current_max,
                                    (s32)conf_now->lo_current_min);
    if(current_max_abs < 0)
    {
        current_max_abs = -current_max_abs;
    }

    if(current_max_abs > 0)
    {
        utils_truncate_number_abs(&id_set_tmp, current_max_abs);
        iq_limit_square = (u32)((int64_t)current_max_abs * current_max_abs -
                                (int64_t)id_set_tmp * id_set_tmp);
        iq_max_abs = (s32)utils_sqrt_u32(iq_limit_square);
        utils_truncate_number_abs(&iq_set_tmp, iq_max_abs);
    }

    state_now->id_target = FocHw_SatS16(id_set_tmp);
    state_now->iq_target = FocHw_SatS16(iq_set_tmp);
    state_now->max_duty = conf_now->l_max_duty;

    if(motor_now->m_control_mode == CONTROL_MODE_NONE)
    {
        motor_now->m_state = MC_STATE_OFF;
    }
    else
    {
        motor_now->m_state = MC_STATE_RUNNING;
    }

    Motor_CurrentLoopRun(1U);
}

void PhaseCurrent_CheckFast(void)
{
    s32 phaseA;
    s32 phaseB;
    u16 absA;
    u16 absB;
    u16 peakAbs;
    u16 absMaA;
    u16 absMaB;
    u32 peakAbsMa;

    phaseA = (s32)GET_CURRENT_U_SAMPLE_RESULT() - (s32)hPhaseAOffset;
    phaseB = (s32)GET_CURRENT_V_SAMPLE_RESULT() - (s32)hPhaseBOffset;
    gPhaseCurrentAAdc = FocHw_SatS16(phaseA);
    gPhaseCurrentBAdc = FocHw_SatS16(phaseB);

    absA = FocHw_AbsS32ToU16(phaseA);
    absB = FocHw_AbsS32ToU16(phaseB);
    peakAbs = (absA > absB) ? absA : absB;

    absMaA = FocHw_AbsS32ToU16((s32)ADC_curr_norm_value[0]);
    absMaB = FocHw_AbsS32ToU16((s32)ADC_curr_norm_value[1]);
    peakAbsMa = (absMaA > absMaB) ? (u32)absMaA : (u32)absMaB;

    if(peakAbs > gPhaseCurrentPeakAbsAdc)
    {
        gPhaseCurrentPeakAbsAdc = peakAbs;
    }

    if(peakAbsMa > gPhaseCurrentPeakAbsMa)
    {
        gPhaseCurrentPeakAbsMa = peakAbsMa;
    }

    /* Keep the threshold values available for the next protection step. */
    (void)MCS_PHASE_CURRENT_OVER_ADC;
    (void)MCS_PHASE_CURRENT_RELEASE_ADC;
}

void AdcEocHandler(void)
{
    s16 bus_adc;
    u32 bus_mv;

    AdcSampleCal();
    PhaseCurrent_CheckFast();

    bus_adc = AcqAdcSampDatUdc();
    bus_mv = FocHw_AdcToBusMv(bus_adc);
    gBUS_Vol_ADC = FocHw_SatS16((s32)gBUS_Vol_ADC +
        (((s32)bus_adc - (s32)gBUS_Vol_ADC) * MCS_BUS_FILTER_Q15 >> 15));
    gBusVoltageMv = (u32)((s32)gBusVoltageMv +
        (s32)(((int64_t)((s32)bus_mv - (s32)gBusVoltageMv) *
               MCS_BUS_FILTER_Q15) >> 15));
    m_motor.m_motor_state.v_bus = (s32)gBusVoltageMv;

    hal1 = GET_HALLA_SAMPLE_RESULT();
    hal2 = GET_HALLB_SAMPLE_RESULT();
}

void PwmAOutputs(FuncState t_state)
{
    MCPWM_PRT = 0x0000DEAD;

    if(t_state == ENABLE)
    {
        MCPWM_FAIL012 |= MCPWM_MOE_ENABLE_MASK;
    }
    else
    {
        MCPWM_FAIL012 &= MCPWM_MOE_DISABLE_MASK;
    }

    MCPWM_PRT = 0x0000CAFE;
}
