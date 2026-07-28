#include "main.h"

/*
 * 定点电流控制器各变量单位：
 *   i_alpha/i_beta、id/iq 及其目标值       ：mA
 *   phase、phase_sin/phase_cos              ：Q16 角度、Q15 正余弦
 *   vd/vq、mod_alpha_raw/mod_beta_raw       ：Q15 调制度，不是 mV
 *   pwm_a/pwm_b/pwm_c                       ：定时器比较计数值
 *
 * 本模块是 FOC 最内层控制环，不负责决定转速和角度，只负责让测量的 id/iq
 * 跟随外层逻辑给出的目标值。
 */

#define MCS_CURRENT_KP_Q15_PER_MA          (4L)
#define MCS_CURRENT_KI_Q15_PER_MA_TICK     (6933L)
#define MCS_CURRENT_KI_FRAC_SHIFT          (15U)
#define MCS_CURRENT_FILTER_Q15              (32767L)
#define MCS_CURRENT_RAMP_MA_PER_MS          (2L)
#define MCS_SVM_MAX_MOD_Q15                (30000L)
#define MCS_MOTOR_CURRENT_MAX_MA            (4000L)
#define MCS_OVERMOD_FACTOR_Q15              (32767L)
#define MCS_SQRT3_BY_2_Q15                 (28378L)
#define MCS_Q15_SHIFT                      (15U)
#define MCS_Q15_ONE                        (32767L)
#define FOC_MOTOR_R_MOHM                    (254L)
#define FOC_MOTOR_L_UH                      (343L)
#define FOC_MOTOR_FLUX_LINKAGE_UWB          (4450L)

static mc_configuration m_motor_conf = {
    .foc_control_sample_mode = FOC_CONTROL_SAMPLE_MODE_V0,
    .foc_mtpa_mode = MTPA_MODE_OFF,
    .foc_speed_soure = FOC_SPEED_SRC_CORRECTED,
    .foc_sensor_mode = FOC_SENSOR_MODE_SENSORLESS,
    .foc_temp_comp = 0,
    .foc_current_ki = MCS_CURRENT_KI_Q15_PER_MA_TICK,
    .foc_current_filter_const = MCS_CURRENT_FILTER_Q15,
    .current_ramp_ma_per_ms = MCS_CURRENT_RAMP_MA_PER_MS,
    .foc_current_kp = MCS_CURRENT_KP_Q15_PER_MA,
    .foc_cc_decoupling = FOC_CC_DECOUPLING_DISABLED,
    .foc_motor_r = FOC_MOTOR_R_MOHM,
    .foc_motor_l = FOC_MOTOR_L_UH,
    .foc_motor_flux_linkage = FOC_MOTOR_FLUX_LINKAGE_UWB,
    .foc_overmod_factor = MCS_OVERMOD_FACTOR_Q15,         // 最大电压矢量的附加缩放系数
    .l_max_duty = MCS_SVM_MAX_MOD_Q15,                    // 允许使用最大调制比例
    .lo_current_min = -MCS_MOTOR_CURRENT_MAX_MA,
    .lo_current_max = MCS_MOTOR_CURRENT_MAX_MA
};

motor_all_state_t m_motor;

static s32 Foc_IntegrateCurrentError(s32 integral,
                                     s32 error,
                                     s32 ki_q15,
                                     volatile s32 *residual)
{
    s32 accumulator;
    s32 increment;

    /* 保存小于一个 Q15 计数的余数，使很小的误差也能随时间累积。 */
    accumulator = *residual + error * ki_q15;
    increment = accumulator >> MCS_CURRENT_KI_FRAC_SHIFT;
    *residual = accumulator -
                increment * (1L << MCS_CURRENT_KI_FRAC_SHIFT);

    return integral + increment;
}

static void Foc_InitMotorStruct(motor_all_state_t *motor)
{
    s32 max_duty;
    s32 overmod_factor;
    s32 max_v_mag;

    if(motor->m_conf == 0)
    {
        motor->m_conf = &m_motor_conf;
        motor->m_run_state = MOTOR_RUN_STATE_OFF;
        motor->m_fault_code = MOTOR_FAULT_NONE;
        motor->m_control_mode = CONTROL_MODE_NONE;

        /* 这些配置在运行中保持不变，预先计算以减少快速环中的限幅和乘法。 */
        max_duty = McsMath_LimitS32((s32)motor->m_conf->l_max_duty,
                                0L,
                                MCS_Q15_ONE);
        overmod_factor = McsMath_LimitS32((s32)motor->m_conf->foc_overmod_factor,
                                      0L,
                                      MCS_Q15_ONE);
        max_v_mag = McsMath_MulQ15(max_duty, overmod_factor);
        max_v_mag = McsMath_MulQ15(max_v_mag, MCS_SQRT3_BY_2_Q15);

        motor->m_motor_state.max_duty = (s16)max_duty;
        motor->p_max_v_mag = (s16)max_v_mag;
        motor->p_dt = 1U;
    }
}

static void Foc_ResetCurrentPi(motor_all_state_t *motor)
{
    volatile motor_state_t *state_m = &motor->m_motor_state;

    state_m->vd_int = 0;
    state_m->vq_int = 0;
    state_m->vd_int_residual = 0;
    state_m->vq_int_residual = 0;
    state_m->vd = 0;
    state_m->vq = 0;
    state_m->id_error = 0;
    state_m->iq_error = 0;
    state_m->mod_alpha_raw = 0;
    state_m->mod_beta_raw = 0;
}


/* 将一组三相中心对齐比较值写入下一 PWM 周期。 */
static void Foc_WriteSvmPwm(u32 phaseA, u32 phaseB, u32 phaseC)
{
    if(phaseA > PWM_PERIOD)
    {
        phaseA = PWM_PERIOD;
    }

    if(phaseB > PWM_PERIOD)
    {
        phaseB = PWM_PERIOD;
    }

    if(phaseC > PWM_PERIOD)
    {
        phaseC = PWM_PERIOD;
    }

    MCPWM_TH20 = (u16)(0U - (u16)phaseC);
    MCPWM_TH21 = (u16)phaseC;
    MCPWM_TH10 = (u16)(0U - (u16)phaseB);
    MCPWM_TH11 = (u16)phaseB;
    MCPWM_TH00 = (u16)(0U - (u16)phaseA);
    MCPWM_TH01 = (u16)phaseA;
}
/*
 * FOC 电流内环执行顺序：
 *   1. Park：i_alpha/i_beta -> id/iq
 *   2. PI：电流误差 -> vd/vq 调制度
 *   3. 电压限幅及积分抗饱和
 *   4. 反 Park：vd/vq -> alpha/beta 调制度
 *   5. SVM：alpha/beta -> 三相 PWM 比较值
 */
static void control_current(motor_all_state_t *motor, u16 dt)
{
    volatile motor_state_t *state_m;
    volatile mc_configuration *conf_now;
    MCS_TRIG_Q15 trig;
    s32 id;
    s32 iq;
    s32 vd;
    s32 vq;
    s32 alpha;
    s32 beta;
    u32 tA;
    u32 tB;
    u32 tC;
    u32 sector;
    s32 Ierr_d;
    s32 Ierr_q;
    s32 kp;
    s32 ki;
    s32 vd_int;
    s32 vq_int;
    s32 p_d;
    s32 p_q;
    s32 max_duty;
    s32 max_v_mag;
    s32 integral_before_limit;
    s32 current_abs;
    s32 vd_abs;
    s32 vq_abs;
    s32 voltage_abs_max;
    s32 voltage_abs_min;
    s32 voltage_mag_approx;
    s32 voltage_scale_q15;

    state_m = &motor->m_motor_state;
    conf_now = motor->m_conf;

    /* 当前 Ki 已经是每个快速环周期使用一次的离散增益。 */
    (void)dt;

    if(motor->m_control_mode == CONTROL_MODE_NONE)
    {
        // 先回到初始状态，让三相回到中性点
        Foc_ResetCurrentPi(motor);
        Foc_WriteSvmPwm(PWM_PERIOD / 2U, PWM_PERIOD / 2U, PWM_PERIOD / 2U);
        return;
    }

    /*
     * 在局部变量中保存一份 sin/cos 快照，确保二者属于同一个角度；若反复读取
     * volatile 状态字段，可能在更新边界处读到不同角度的数据。
     */
    trig.cos = state_m->phase_cos;
    trig.sin = state_m->phase_sin;

    /* 最大占空比和电压矢量上限已在初始化时完成限幅及计算。 */
    max_duty = (s32)state_m->max_duty;
    max_v_mag = (s32)motor->p_max_v_mag;

    /* Park 变换：静止坐标系采样电流 -> 转子坐标系 d/q 电流，单位 mA。 */
    id = (((s32)state_m->i_alpha * trig.cos) +
          ((s32)state_m->i_beta * trig.sin)) >> 15;
    iq = (((s32)state_m->i_beta * trig.cos) -
          ((s32)state_m->i_alpha * trig.sin)) >> 15;

    state_m->id = McsMath_SatS16(id);
    state_m->iq = McsMath_SatS16(iq);

    /* 快速环保留未滤波值，监控显示所需的滤波放到慢速任务中完成。暂时未使用，注释掉 */
    // state_m->id_filter = state_m->id;
    // state_m->iq_filter = state_m->iq;

    if(motor->m_control_mode == CONTROL_MODE_OPENLOOP_DUTY_PHASE)
    {
        /* 直接电压模式仍测量 d/q 电流，但不覆盖外部直接写入的 PWM 矢量。有疑问，这个判断是否必要 */
        state_m->id_error = 0;
        state_m->iq_error = 0;
        return;
    }

    /* 正 iq 在当前电角度和相序定义下产生正方向转矩。 */
    Ierr_d = (s32)state_m->id_target - id;
    Ierr_q = (s32)state_m->iq_target - iq;
    state_m->id_error = Ierr_d;
    state_m->iq_error = Ierr_q;

    kp = (s32)conf_now->foc_current_kp;
    ki = (s32)conf_now->foc_current_ki;
    
//    ki = conf_now->foc_temp_comp ?
//         (s32)motor->m_current_ki_temp_comp :
//         (s32)conf_now->foc_current_ki;

    /* Kp/Ki 输出均为 Q15 调制度，Ki 在每次 ADC 中断中积分一次。 */
    p_d = Ierr_d * kp;
    p_q = Ierr_q * kp;

    /* 保留 Ki 的小数余量，使较小误差也能平滑累积。 */
    vd_int = Foc_IntegrateCurrentError(state_m->vd_int,
                                       Ierr_d,
                                       ki,
                                       &state_m->vd_int_residual);
    vq_int = Foc_IntegrateCurrentError(state_m->vq_int,
                                       Ierr_q,
                                       ki,
                                       &state_m->vq_int_residual);
    
    /* 先将积分状态限制在 Q15 有效范围，保证后续 32 位乘法不溢出。 */
    integral_before_limit = vd_int;
    vd_int = McsMath_LimitS32(vd_int, -MCS_Q15_ONE, MCS_Q15_ONE);
    if(vd_int != integral_before_limit)
    {
        state_m->vd_int_residual = 0;
    }
    integral_before_limit = vq_int;
    vq_int = McsMath_LimitS32(vq_int, -MCS_Q15_ONE, MCS_Q15_ONE);
    if(vq_int != integral_before_limit)
    {
        state_m->vq_int_residual = 0;
    }

    vd = McsMath_LimitS32(vd_int + p_d, -MCS_Q15_ONE, MCS_Q15_ONE);
    vq = McsMath_LimitS32(vq_int + p_q, -MCS_Q15_ONE, MCS_Q15_ONE);

    /*
     * 用 max(|vd|,|vq|) + 27/64*min(|vd|,|vq|) 保守近似圆形幅值。
     * 27/64 略大于 sqrt(2)-1，可保证近似值不小于真实幅值。
     * 只有超过上限时才执行一次除法并等比例缩放，保持电压矢量方向不变。
     */
    vd_abs = McsMath_AbsS32(vd);
    vq_abs = McsMath_AbsS32(vq);
    if(vd_abs >= vq_abs)
    {
        voltage_abs_max = vd_abs;
        voltage_abs_min = vq_abs;
    }
    else
    {
        voltage_abs_max = vq_abs;
        voltage_abs_min = vd_abs;
    }
    voltage_mag_approx = voltage_abs_max +
                         ((voltage_abs_min * 27L) >> 6);

    if((voltage_mag_approx > max_v_mag) && (voltage_mag_approx > 0L))
    {
        voltage_scale_q15 = (max_v_mag << MCS_Q15_SHIFT) /
                            voltage_mag_approx;
        vd = McsMath_MulQ15(vd, voltage_scale_q15);
        vq = McsMath_MulQ15(vq, voltage_scale_q15);
        vd_int = McsMath_MulQ15(vd_int, voltage_scale_q15);
        vq_int = McsMath_MulQ15(vq_int, voltage_scale_q15);
        state_m->vd_int_residual = 0;
        state_m->vq_int_residual = 0;
    }

    state_m->vd_int = vd_int;
    state_m->vq_int = vq_int;
    state_m->vd = McsMath_SatS16(vd);
    state_m->vq = McsMath_SatS16(vq);
    state_m->mod_d = state_m->vd;
    state_m->mod_q = state_m->vq;
    
    /* 不计算 sqrt，以 max(|id|, |iq|) 近似电流幅值供诊断使用。 */
    current_abs = McsMath_AbsS32(id);
    if(McsMath_AbsS32(iq) > current_abs)
    {
        current_abs = McsMath_AbsS32(iq);
    }
    state_m->i_abs_filter = McsMath_SatS16(current_abs);
    
    
    /* 反 Park 变换：转子坐标系 d/q 调制度 -> 静止坐标系 alpha/beta 调制度。 */
    alpha = ((vd * trig.cos) - (vq * trig.sin)) >> 15;
    beta = ((vd * trig.sin) + (vq * trig.cos)) >> 15;

    state_m->mod_alpha_raw = McsMath_SatS16(alpha);
    state_m->mod_beta_raw = McsMath_SatS16(beta);

    /* SVM 将电压矢量转换为中心对齐的 U/V/W 三相占空比。 */
    FOC_SVM_Q15(state_m->mod_alpha_raw,
                state_m->mod_beta_raw,
                max_duty,
                PWM_PERIOD,
                &tA,
                &tB,
                &tC,
                &sector);

    state_m->pwm_a = (u16)tA;
    state_m->pwm_b = (u16)tB;
    state_m->pwm_c = (u16)tC;
    state_m->svm_sector = (u16)sector;

    /* 这些比较值会在下一次 PWM 更新事件中生效。 */
    Foc_WriteSvmPwm(tA, tB, tC);
}

void Motor_FocInit(void)
{
    Foc_InitMotorStruct(&m_motor);
}

void Motor_CurrentLoopRun(u16 dt)
{
    control_current(&m_motor, dt);
}

void Motor_FocLoopRun(u16 angle)
{
    MCS_TRIG_Q15 trig;

    Foc_InitMotorStruct(&m_motor);
    trig = Motor_GetSinCosQ15(angle);
    m_motor.m_motor_state.phase = (s16)angle;
    m_motor.m_motor_state.phase_sin = trig.sin;
    m_motor.m_motor_state.phase_cos = trig.cos;
    control_current(&m_motor, 1U);
}
