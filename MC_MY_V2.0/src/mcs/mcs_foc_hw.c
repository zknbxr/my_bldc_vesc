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

/*
 * LKS32 ADC 结果为左对齐。下面的系数将 ADC 计数转换为 FOC 状态使用的整数单位。
 */
#define MCS_CURRENT_ADC_TO_MA_Q15          (19802L)
#define MCS_BUS_ADC_TO_MV_Q15              (75600L)
#define MCS_BUS_FILTER_Q15                 (8192L)
#define FOC_TWO_BY_THREE_Q15               (21845L)
#define FOC_CONTROL_DT_US                   ((u16)(1000000UL / PWM_FREQ))

/*
    * 无感观测器部分
    * 观测器运行周期
    * 相位SLOT
    * 无感计算4阶段
*/ 
#define FOC_OBSERVER_FLUX_DIV               (4U)
#define FOC_OBSERVER_MAGNITUDE_SLOT         (1U)
#define FOC_OBSERVER_PHASE_SLOT             (2U)
#define FOC_OBSERVER_STAGE_FLUX              (1U)
#define FOC_OBSERVER_STAGE_MAGNITUDE         (2U)
#define FOC_OBSERVER_STAGE_PHASE             (3U)
#define FOC_OBSERVER_STAGE_CLOSED_LOOP       (4U)
/* 按当前 ADC 电流换算系数，单个采样点允许约 2.5 A 的变化。 */
#define MCS_CURRENT_MAX_STEP_MA             (2500L)

volatile s16 gBUS_Vol_ADC;
volatile u32 gBusVoltageMv;
volatile s16 ADC_curr_norm_value[3];
volatile u32 gAdcCurrentSpikeRejectCount;
volatile u8 gObserverStage = FOC_OBSERVER_STAGE_CLOSED_LOOP;
volatile s16 gObserverPhase;
volatile s16 gObserverPhaseError;

INT16 hal1;
INT16 hal2;

static s32 s_currentAcceptedMaU;
static s32 s_currentAcceptedMaV;
static u8 s_currentRejectStreakU;
static u8 s_currentRejectStreakV;
static bool s_currentFilterInitialized;
static s32 s_observerVAlphaSum;
static s32 s_observerVBetaSum;
static s32 s_observerIAlphaSum;
static s32 s_observerIBetaSum;
static u8 s_observerFluxSamples;
static u8 s_observerPostFluxSlot;

static s16 FocHw_PhaseDifference(s16 phase, s16 reference)
{
    return (s16)((u16)phase - (u16)reference);
}

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

static u32 FocHw_AbsS32(s32 value)
{
    if(value >= 0L)
    {
        return (u32)value;
    }

    return (u32)(-(value + 1L)) + 1UL;
}

// 去除单次毛刺
static s32 FocHw_RejectSingleCurrentSpikeMa(s32 sample, s32 *accepted,
                                            u8 *reject_streak)
{
    if((FocHw_AbsS32(sample - *accepted) > MCS_CURRENT_MAX_STEP_MA) &&
       (*reject_streak == 0U))
    {
        *reject_streak = 1U;
        gAdcCurrentSpikeRejectCount++;
        return *accepted;
    }

    *reject_streak = 0U;
    *accepted = sample;
    return sample;
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
    s32 observerVAlphaAvg;
    s32 observerVBetaAvg;
    s32 observerIAlphaAvg;
    s32 observerIBetaAvg;

    Motor_FocInit();
    motor_now = &m_motor;
    state_now = &motor_now->m_motor_state;
    conf_now = motor_now->m_conf;

    /* 第 1 步：读取 PWM 定时触发的两路下桥臂采样电阻 ADC 值。 */
    raw0 = (s32)AcqAdcSampDatPhaseU();
    raw1 = (s32)AcqAdcSampDatPhaseV();

    /* 根据采样极性减去零偏，直接换算为单位 mA 的相电流。 */
    curr0 = (s32)FocHw_AdcToCurrentMa(
        (s32)conf_now->foc_offsets_current[0] - raw0);
    curr1 = (s32)FocHw_AdcToCurrentMa(
        (s32)conf_now->foc_offsets_current[1] - raw1);

    // 滤除电流突变毛刺
    /* 拒绝单次孤立毛刺；若连续两个采样都发生变化，则认为是真实电流。 */
    if(!s_currentFilterInitialized)
    {
        s_currentAcceptedMaU = curr0;
        s_currentAcceptedMaV = curr1;
        s_currentFilterInitialized = true;
    }
    else
    {
        curr0 = FocHw_RejectSingleCurrentSpikeMa(
            curr0, &s_currentAcceptedMaU, &s_currentRejectStreakU);
        curr1 = FocHw_RejectSingleCurrentSpikeMa(
            curr1, &s_currentAcceptedMaV, &s_currentRejectStreakV);
    }

    /* 第三相没有独立 ADC 通道，根据 ia+ib+ic=0 重构。 */
    curr2 = -(curr0 + curr1);
    ADC_curr_norm_value[0] = FocHw_SatS16(curr0);
    ADC_curr_norm_value[1] = FocHw_SatS16(curr1);
    ADC_curr_norm_value[2] = FocHw_SatS16(curr2);

    /* 对满足三相电流平衡的两电阻采样结果进行 Clarke 变换。 */
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

    /*
     * 第 3 步：PWM 工作时运行无感观测器。
     * 磁链积分、幅值矫正和 CORDIC/PLL 分散到不同中断时隙中执行，避免单次
     * 中断时间超过一个 PWM 周期；完整观测结果每四个采样更新一次。
     * 计算分步，4个采样更新一次，更新时在4个采样周期分步计算完成
     */
    if((motor_now->m_control_mode != CONTROL_MODE_NONE) &&
       (gObserverStage >= FOC_OBSERVER_STAGE_FLUX))
    {
        /* 磁链更新前先对四个 PWM 周期的数据求平均，以降低计算频率。
         * 区间末端的瞬时电流单独保留，用于计算 L*di 项。
         */
        s_observerVAlphaSum += state_now->v_alpha;
        s_observerVBetaSum += state_now->v_beta;
        s_observerIAlphaSum += state_now->i_alpha;
        s_observerIBetaSum += state_now->i_beta;
        s_observerFluxSamples++;

        if(s_observerFluxSamples >= FOC_OBSERVER_FLUX_DIV)
        {
            observerVAlphaAvg = s_observerVAlphaSum / (s32)FOC_OBSERVER_FLUX_DIV;
            observerVBetaAvg = s_observerVBetaSum / (s32)FOC_OBSERVER_FLUX_DIV;
            observerIAlphaAvg = s_observerIAlphaSum / (s32)FOC_OBSERVER_FLUX_DIV;
            observerIBetaAvg = s_observerIBetaSum / (s32)FOC_OBSERVER_FLUX_DIV;

            foc_observer_update(observerVAlphaAvg, observerVBetaAvg,
                                observerIAlphaAvg, observerIBetaAvg,
                                state_now->i_alpha, state_now->i_beta,
                                (u16)(FOC_CONTROL_DT_US * FOC_OBSERVER_FLUX_DIV),
                                &motor_now->m_observer_state, motor_now);

            s_observerVAlphaSum = 0L;
            s_observerVBetaSum = 0L;
            s_observerIAlphaSum = 0L;
            s_observerIBetaSum = 0L;
            s_observerFluxSamples = 0U;
            s_observerPostFluxSlot = FOC_OBSERVER_MAGNITUDE_SLOT;
        }
        else if((gObserverStage >= FOC_OBSERVER_STAGE_MAGNITUDE) &&
                (s_observerPostFluxSlot == FOC_OBSERVER_MAGNITUDE_SLOT)) // 3
        {
            foc_observer_apply_correction(
                &motor_now->m_observer_state, motor_now);
            s_observerPostFluxSlot = FOC_OBSERVER_PHASE_SLOT;
        }
        else if((gObserverStage >= FOC_OBSERVER_STAGE_PHASE) &&
                (s_observerPostFluxSlot == FOC_OBSERVER_PHASE_SLOT)) // 4
        {
            foc_observer_update_phase(
                &motor_now->m_observer_state,
                &motor_now->m_phase_now_observer);
            foc_observer_pll_run(
                motor_now->m_phase_now_observer,
                &motor_now->m_observer_state, motor_now);
            gObserverPhase = motor_now->m_phase_now_observer;
            gObserverPhaseError = FocHw_PhaseDifference(
                motor_now->m_phase_now_observer, state_now->phase);
            s_observerPostFluxSlot = 0U;
        }

        /*
         * 阶段 3 只观测并发布角度；阶段 4 中还需要等待 1 ms 任务清除
         * phase_override，观测角才会真正成为 Park 变换角度。
         */
        if((gObserverStage >= FOC_OBSERVER_STAGE_CLOSED_LOOP) &&
           (motor_now->m_phase_override == false))
        {
            state_now->phase = motor_now->m_phase_now_observer;
        }
    }
    else
    {
        s_observerVAlphaSum = 0L;
        s_observerVBetaSum = 0L;
        s_observerIAlphaSum = 0L;
        s_observerIBetaSum = 0L;
        s_observerFluxSamples = 0U;
        s_observerPostFluxSlot = 0U;
        motor_now->m_observer_state.i_alpha_last = state_now->i_alpha;
        motor_now->m_observer_state.i_beta_last = state_now->i_beta;
    }

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
    state_now->max_duty = conf_now->l_max_duty;

    if(motor_now->m_control_mode == CONTROL_MODE_NONE)
    {
        motor_now->m_state = MC_STATE_OFF;
    }
    else
    {
        motor_now->m_state = MC_STATE_RUNNING;
    }

    /* 第 6 步：Park -> 电流 PI -> 反 Park -> SVM -> 更新 PWM 比较值。 */
    Motor_CurrentLoopRun(1U);
}

void AdcEocHandler(void)
{
    s16 bus_adc;
    u32 bus_mv;

    /* 电流控制对时序最敏感，因此放在 ADC 中断最前面执行。 */
    AdcSampleCal();

    /*
     * 母线电压和 Hall 诊断值在 PWM 计算之后更新，因此快速环有意使用上一周期
     * 的滤波母线电压，该数据只滞后一个 PWM 周期。
     */
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
