#include "main.h"

/*
 * FOC 快速环数据流（在 ADC 转换完成中断中执行）：
 *
 *   ADC 电流 -> 零偏/极性 -> Clarke (i_alpha/i_beta)
 *       -> 磁链观测器 + PLL -> 选择电角度
 *       -> Park/电流 PI/反 Park -> SVM -> 下一周期 PWM 比较值
 *
 * 中断内全部使用整数物理量：电流单位 mA，电压单位 mV，时间单位 us，
 * 电角度一圈对应 0...65535。
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
    /* 最坏情况下的乘积仍不会超出有符号 32 位范围。 */
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

/* Q15 SVM 指令为 1.0 时，alpha-beta 电压等于母线电压的 2/3。 */
s32 FocHw_ModToVoltageMv(s16 modulation, s32 bus_voltage)
{
    s32 max_alpha_beta_voltage;

    if(bus_voltage <= 0L)
    {
        return 0L;
    }

    /* 本项目母线电压不超过 ADC 换算上限，两个乘积都在 s32 范围内。 */
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
 * 基础定点 FOC 的 ADC 快速路径。
 * 当前未包含 HFI、MTPA 和弱磁；定点无感观测器使用上一 PWM 周期的电压
 * 与本次电流采样进行更新。
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

    /* 运行状态只由快速环维护，避免主循环和 ADC 中断同时写 m_state。 */
    if(motor_now->m_control_mode == CONTROL_MODE_NONE)
    {
        motor_now->m_state = MC_STATE_OFF;
    }
    else
    {
        motor_now->m_state = MC_STATE_RUNNING;
    }


    /* 第 1 步：读取 PWM 定时触发的两路下桥臂采样电阻 ADC 值。 */
    raw0 = (s32)AcqAdcSampDatPhaseU();
    raw1 = (s32)AcqAdcSampDatPhaseV();

    /* 根据采样极性减去零偏，直接换算为单位 mA 的相电流。 */
    curr0 = (s32)FocHw_AdcToCurrentMa(
        (s32)conf_now->foc_offsets_current[0] - raw0);
    curr1 = (s32)FocHw_AdcToCurrentMa(
        (s32)conf_now->foc_offsets_current[1] - raw1);

    /* 第三相没有独立 ADC 通道，根据 ia+ib+ic=0 重构。 */
    curr2 = -(curr0 + curr1);
    ADC_curr_norm_value[0] =curr0;//FocHw_SatS16(curr0)
    ADC_curr_norm_value[1] =curr1;//FocHw_SatS16(curr1)
    // 只保留 curr2
    ADC_curr_norm_value[2] = FocHw_SatS16(curr2);

    /* 对满足三相电流平衡的两电阻采样结果进行 Clarke 变换。 */
    // mA克拉克变换
    state_now->i_alpha = ADC_curr_norm_value[0];
    state_now->i_beta = FocHw_SatS16(
        ((s32)ONE_BY_SQRT3 * ADC_curr_norm_value[0] +
         (s32)TWO_BY_SQRT3 * ADC_curr_norm_value[1]) >> 15);

    /*
     * 第 3 步：配置为无感且 PWM 工作时运行观测器。
     * 磁链积分、幅值矫正和 CORDIC/PLL 分散到不同中断时隙中执行，避免单次
     * 中断时间超过一个 PWM 周期；完整观测结果每四个采样更新一次。
     */
    foc_sensorless_update(motor_now);


    /* 第 4 步：只计算一次 sin/cos，保证电流 PI 使用同一角度的一对值。 */
    trig = Motor_GetSinCosQ15((u16)state_now->phase);
    state_now->phase_sin = trig.sin;
    state_now->phase_cos = trig.cos;

    /* 第 5 步：将慢速任务给出的电流指令复制到快速环目标值。 */
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
    /* 第 6 步：Park -> 电流 PI -> 反 Park -> SVM -> 更新 PWM 比较值。 */
    Motor_CurrentLoopRun(1U);
}

void AdcEocHandler(void)
{
    /* 电流控制对时序最敏感，因此放在 ADC 中断最前面执行。 */
    AdcSampleCal();

    /* 中断内只保存最新慢速 ADC 数据，换算和滤波放到 1 ms 任务。 */
    s_busAdcLatest = AcqAdcSampDatUdc();
    hal1 = GET_HALLA_SAMPLE_RESULT();
    hal2 = GET_HALLB_SAMPLE_RESULT();
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
