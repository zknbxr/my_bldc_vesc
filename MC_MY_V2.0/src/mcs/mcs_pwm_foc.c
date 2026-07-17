#include "main.h"

/*
 * 定点电流控制器各变量单位：
 *   i_alpha/i_beta、id/iq 及其目标值       ：mA
 *   phase、phase_sin/phase_cos              ：Q16 角度、Q15 正余弦
 *   vd/vq、mod_alpha/mod_beta               ：Q15 调制度，不是 mV
 *   pwm_a/pwm_b/pwm_c                       ：定时器比较计数值
 *
 * 本模块是 FOC 最内层控制环，不负责决定转速和角度，只负责让测量的 id/iq
 * 跟随外层逻辑给出的目标值。
 */

#define MCS_CURRENT_KP_Q15_PER_MA          (4L)
#define MCS_CURRENT_KI_Q15_PER_MA_TICK     (6933L)
#define MCS_CURRENT_KI_FRAC_SHIFT          (15U)
#define MCS_CURRENT_FILTER_Q15              (32767L)
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
    .foc_temp_comp = 0,
    .foc_current_ki = MCS_CURRENT_KI_Q15_PER_MA_TICK,
    .foc_current_filter_const = MCS_CURRENT_FILTER_Q15,
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

static s32 Foc_LimitS32(s32 value, s32 min, s32 max)
{
    if(value > max)
    {
        return max;
    }

    if(value < min)
    {
        return min;
    }

    return value;
}

static s16 Foc_SatS16(s32 value)
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

static s32 Foc_AbsS32(s32 value)
{
    if(value == (-2147483647L - 1L))
    {
        return 2147483647L;
    }

    return (value < 0) ? -value : value;
}

static s32 Foc_MulQ15(s32 a, s32 b)
{
    return (a * b) >> MCS_Q15_SHIFT;
}

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
    if(motor->m_conf == 0)
    {
        motor->m_conf = &m_motor_conf;
        motor->m_state = MC_STATE_OFF;
        motor->m_control_mode = CONTROL_MODE_NONE;
        motor->m_motor_state.max_duty = MCS_SVM_MAX_MOD_Q15;
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
    state_m->mod_alpha = 0;
    state_m->mod_beta = 0;
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
    s32 max_vq;
    s32 overmod_factor;
    s32 integral_before_limit;
    s32 current_abs;

    // 如果未初始化，就初始化
    Foc_InitMotorStruct(motor);

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

    max_duty = Foc_AbsS32((s32)state_m->max_duty);
    max_duty = Foc_LimitS32(max_duty, 0L, (s32)conf_now->l_max_duty);

    overmod_factor = Foc_LimitS32((s32)conf_now->foc_overmod_factor,
                                  0L,
                                  MCS_Q15_ONE);
    
    /* 当前母线电压和最大占空比允许的 alpha-beta 电压矢量调制度上限。 */
    max_v_mag = Foc_MulQ15(max_duty, overmod_factor);
    max_v_mag = Foc_MulQ15(max_v_mag, MCS_SQRT3_BY_2_Q15);

    /* Park 变换：静止坐标系采样电流 -> 转子坐标系 d/q 电流，单位 mA。 */
    id = (((s32)state_m->i_alpha * trig.cos) +
          ((s32)state_m->i_beta * trig.sin)) >> 15;
    iq = (((s32)state_m->i_beta * trig.cos) -
          ((s32)state_m->i_alpha * trig.sin)) >> 15;

    state_m->id = Foc_SatS16(id);
    state_m->iq = Foc_SatS16(iq);

    /* 快速环保留未滤波值，监控显示所需的滤波放到慢速任务中完成。 */
    state_m->id_filter = state_m->id;
    state_m->iq_filter = state_m->iq;

    if(motor->m_control_mode == CONTROL_MODE_OPENLOOP_DUTY_PHASE)
    {
        /* 直接电压模式仍测量 d/q 电流，但不覆盖外部直接写入的 PWM 矢量。 */
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
    ki = conf_now->foc_temp_comp ?
         (s32)motor->m_current_ki_temp_comp :
         (s32)conf_now->foc_current_ki;

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
    
    vd = vd_int + p_d;
    vq = vq_int + p_q;

    
    /* 优先分配 d 轴电压，并同步限制积分项，防止积分饱和。 */
    vd = Foc_LimitS32(vd, -max_v_mag, max_v_mag);
    integral_before_limit = vd_int;
    vd_int = Foc_LimitS32(vd_int, -max_v_mag, max_v_mag);
    if(vd_int != integral_before_limit)
    {
        state_m->vd_int_residual = 0;
    }

    /*
     * 保守的菱形限幅：|vd| + |vq| <= max_v_mag。
     * 这样可避免每次中断计算 sqrt，但电压利用率低于圆形限幅。
     * 高速或大电流时会更早进入饱和，可能增大电流纹波、运行声音和相电流失真。
     */
    max_vq = max_v_mag - Foc_AbsS32(vd);
    vq = Foc_LimitS32(vq, -max_vq, max_vq);
    integral_before_limit = vq_int;
    vq_int = Foc_LimitS32(vq_int, -max_vq, max_vq);
    if(vq_int != integral_before_limit)
    {
        state_m->vq_int_residual = 0;
    }

    state_m->vd_int = vd_int;
    state_m->vq_int = vq_int;
    state_m->vd = Foc_SatS16(vd);
    state_m->vq = Foc_SatS16(vq);
    state_m->mod_d = state_m->vd;
    state_m->mod_q = state_m->vq;
    
    /* 不计算 sqrt，以 max(|id|, |iq|) 近似电流幅值供诊断使用。 */
    current_abs = Foc_AbsS32(id);
    if(Foc_AbsS32(iq) > current_abs)
    {
        current_abs = Foc_AbsS32(iq);
    }
    state_m->i_abs_filter = Foc_SatS16(current_abs);
    
    
    /* 反 Park 变换：转子坐标系 d/q 调制度 -> 静止坐标系 alpha/beta 调制度。 */
    alpha = ((vd * trig.cos) - (vq * trig.sin)) >> 15;
    beta = ((vd * trig.sin) + (vq * trig.cos)) >> 15;

    state_m->mod_alpha = Foc_SatS16(alpha);
    state_m->mod_beta = Foc_SatS16(beta);
    
    state_m->mod_alpha_raw = state_m->mod_alpha;
    state_m->mod_beta_raw = state_m->mod_beta;

    /* SVM 将电压矢量转换为中心对齐的 U/V/W 三相占空比。 */
    FOC_SVM_Q15(state_m->mod_alpha,
                state_m->mod_beta,
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
