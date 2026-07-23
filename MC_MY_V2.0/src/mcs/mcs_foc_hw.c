#include "main.h"

/*
 * FOC快速环在ADC转换完成中断中执行：
 * ADC电流 -> 零偏/极性 -> Clarke -> 角度更新 -> Park/电流PI/反Park -> SVM。
 * 电流单位为mA，电压单位为mV，时间单位为us，电角度一圈对应0...65535。
 */



volatile s16 gBUS_Vol_ADC;
volatile u32 gBusVoltageMv;
volatile s16 ADC_curr_norm_value[3];
volatile u32 gAdcCurrentSpikeRejectCount;


INT16 hal1;
INT16 hal2;



static volatile s16 s_busAdcLatest;

s16 FocHw_PhaseDifference(s16 phase, s16 reference)
{
    return (s16)((u16)phase - (u16)reference);
}

s16 FocHw_SatS16(s32 value)
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

static s16 FocHw_AdcToCurrentMa(s32 adc_value)
{
    /* 最坏情况下的乘积仍不会超出有符号32位范围。 */
    return FocHw_SatS16(
        (adc_value * MCS_CURRENT_ADC_TO_MA_Q15) >> 15);
}

static u32 FocHw_AdcToBusMv(s16 adc_value)
{
    if(adc_value <= 0)
    {
        return 0UL;
    }

    return (u32)(((int64_t)adc_value * MCS_BUS_ADC_TO_MV_Q15) >> 15);
}

/* Q15 SVM指令为1.0时，alpha-beta电压等于母线电压的2/3。 */
s32 FocHw_ModToVoltageMv(s16 modulation, s32 bus_voltage)
{
    s32 max_alpha_beta_voltage;

    if(bus_voltage <= 0L)
    {
        return 0L;
    }

    /* 本项目母线电压不超过ADC换算上限，两个乘积都在s32范围内。 */
    max_alpha_beta_voltage =
        (bus_voltage * FOC_TWO_BY_THREE_Q15) >> 15;
    return ((s32)modulation * max_alpha_beta_voltage) >> 15;
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
 * 基础定点FOC的ADC快速路径。
 * 当前未包含HFI、MTPA和弱磁；无感观测器使用上一PWM周期电压和本次电流更新。
 */
void AdcSampleCal(void)
{
    motor_all_state_t *motor_now;
    motor_state_t *state_now;
    mc_configuration *conf_now;
    MCS_TRIG_Q15 trig;
    s32 raw0;
    s32 raw1;
    s32 curr0;
    s32 curr1;
    s32 curr2;
    s32 id_set_tmp;
    s32 iq_set_tmp;
    s32 current_max_abs;


    motor_now = &m_motor;
    state_now = &motor_now->m_motor_state;
    conf_now = motor_now->m_conf;

    /* 第1步：读取PWM定时触发的两路下桥臂采样电阻ADC值。 */
    raw0 = (s32)AcqAdcSampDatPhaseU();
    raw1 = (s32)AcqAdcSampDatPhaseV();

    /* 根据采样极性减去零偏，直接换算为单位mA的相电流。 */
    curr0 = (s32)FocHw_AdcToCurrentMa(
        (s32)conf_now->foc_offsets_current[0] - raw0);
    curr1 = (s32)FocHw_AdcToCurrentMa(
        (s32)conf_now->foc_offsets_current[1] - raw1);

    /* 第三相没有独立ADC通道，根据ia+ib+ic=0重构。 */
    curr2 = -(curr0 + curr1);
    ADC_curr_norm_value[0] =curr0;//FocHw_SatS16(curr0)
    ADC_curr_norm_value[1] =curr1;//FocHw_SatS16(curr1)
    // curr2需要经过一次s16饱和。
    ADC_curr_norm_value[2] = FocHw_SatS16(curr2);

    /* 对满足三相电流平衡的两电阻采样结果进行Clarke变换。 */
    state_now->i_alpha = ADC_curr_norm_value[0];
    state_now->i_beta = FocHw_SatS16(
        ((s32)ONE_BY_SQRT3 * ADC_curr_norm_value[0] +
         (s32)TWO_BY_SQRT3 * ADC_curr_norm_value[1]) >> 15);

    /*
     * 第3步：仅在无感模式运行磁链观测器。
     * 磁链积分、幅值校正和CORDIC/PLL分散执行，完整结果每四个采样更新一次。
     */
    foc_sensorless_update(motor_now);


    /* 第4步：只计算一次sin/cos，保证电流PI使用同一角度的一对值。 */
    trig = Motor_GetSinCosQ15((u16)state_now->phase);
    state_now->phase_sin = trig.sin;
    state_now->phase_cos = trig.cos;

    /* 第5步：将慢速任务给出的电流指令复制到快速环目标值。 */
    if(motor_now->m_control_mode == CONTROL_MODE_OPENLOOP_DUTY_PHASE)
    {
        /* 直接电压模式不使用电流目标，也不执行电流矢量限幅。 */
        state_now->id_target = 0;
        state_now->iq_target = 0;
    }
    else
    {
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
            utils_truncate_number_abs(&iq_set_tmp, current_max_abs);
        }

        state_now->id_target = FocHw_SatS16(id_set_tmp);
        state_now->iq_target = FocHw_SatS16(iq_set_tmp);
    }
    /* 第6步：Park -> 电流PI -> 反Park -> SVM -> 更新PWM比较值。 */
    Motor_CurrentLoopRun(1U);
}

void AdcEocHandler(void)
{
    /*
     * 正常霍尔模式先更新控制角，使本次电流环立即使用最新预测角。
     * 学习模式下该函数只做条件判断，不执行CORDIC。
     */
    hal1 = GET_HALLA_SAMPLE_RESULT();
    hal2 = GET_HALLB_SAMPLE_RESULT();
    Hall_FastUpdate(&m_motor, hal1, hal2);

    /* 角度准备完成后立即执行电流控制。 */
    AdcSampleCal();

    /* 中断内只保存最新慢速ADC数据，换算和滤波放到1ms任务。 */
    s_busAdcLatest = AcqAdcSampDatUdc();
    if(gMotorWorkMode == MCS_WORK_MODE_LEARN)
    {
        Hall_CaptureSample(hal1, hal2, m_motor.m_motor_state.phase);
    }
}

void Motor_FocSlowUpdate1ms(void)
{
    s16 bus_adc;
    u32 bus_mv;
    s32 bus_mv_filtered;

    bus_adc = s_busAdcLatest;
    bus_mv = FocHw_AdcToBusMv(bus_adc);
    gBUS_Vol_ADC = FocHw_SatS16((s32)gBUS_Vol_ADC +
        (((s32)bus_adc - (s32)gBUS_Vol_ADC) * MCS_BUS_FILTER_Q15 >> 15));
    bus_mv_filtered = (s32)gBusVoltageMv +
        (((s32)bus_mv - (s32)gBusVoltageMv) * MCS_BUS_FILTER_Q15 >> 15);
    if(bus_mv_filtered < 0L)
    {
        bus_mv_filtered = 0L;
    }
    gBusVoltageMv = (u32)bus_mv_filtered;
    m_motor.m_motor_state.v_bus = (s32)gBusVoltageMv;
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
