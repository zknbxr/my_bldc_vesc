#include "main.h"

#define MCS_CURRENT_KP_Q15_PER_MA          (4L)
#define MCS_CURRENT_KI_Q15_PER_MA_TICK     (0L)
#define MCS_CURRENT_FILTER_Q15              (32767L)
#define MCS_SVM_MAX_MOD_Q15                (30000L)
#define MCS_MOTOR_CURRENT_MAX_MA            (4000L)
#define MCS_OVERMOD_FACTOR_Q15              (32767L)
#define MCS_SQRT3_BY_2_Q15                 (28378L)
#define MCS_Q15_SHIFT                      (15U)
#define MCS_Q15_ONE                        (32767L)
#define FOC_MOTOR_R_MOHM                    (381L)
#define FOC_MOTOR_L_UH                      (514L)
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
    .foc_overmod_factor = MCS_OVERMOD_FACTOR_Q15,
    .l_max_duty = MCS_SVM_MAX_MOD_Q15,
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

static s32 Foc_SatS32FromS64(int64_t value)
{
    if(value > 2147483647LL)
    {
        return 2147483647L;
    }

    if(value < (-2147483647LL - 1LL))
    {
        return (-2147483647L - 1L);
    }

    return (s32)value;
}

static s32 Foc_MulQ15(s32 a, s32 b)
{
    return Foc_SatS32FromS64(((int64_t)a * (int64_t)b) >> MCS_Q15_SHIFT);
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
    state_m->vd = 0;
    state_m->vq = 0;
    state_m->id_error = 0;
    state_m->iq_error = 0;
    state_m->mod_alpha = 0;
    state_m->mod_beta = 0;
    state_m->mod_alpha_raw = 0;
    state_m->mod_beta_raw = 0;
}


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
//FOC核心电流内环，接收id和iq，根据实际电流计算出需要的vd/vq，最后通过svpwm得到三相占空比。
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
    s32 dec_vd = 0;
    s32 dec_vq = 0;
    s32 dec_bemf = 0;
    s32 max_duty;
    s32 max_v_mag;
    s32 max_vq;
    s32 overmod_factor;
    int64_t tmp64;
    u32 vq_square;

    // 如果未初始化，就初始化
    Foc_InitMotorStruct(motor);

    state_m = &motor->m_motor_state;
    conf_now = motor->m_conf;

    if(dt == 0U)
    {
        dt = 1U;
    }

    if(motor->m_control_mode == CONTROL_MODE_NONE)
    {
        // 先回到初始状态，让三相回到中性点
        Foc_ResetCurrentPi(motor);
        Foc_WriteSvmPwm(PWM_PERIOD / 2U, PWM_PERIOD / 2U, PWM_PERIOD / 2U);
        return;
    }
    
    trig.cos = state_m->phase_cos;
    trig.sin = state_m->phase_sin;

    max_duty = Foc_AbsS32((s32)state_m->max_duty);
    max_duty = Foc_LimitS32(max_duty, 0L, (s32)conf_now->l_max_duty);

    overmod_factor = Foc_LimitS32((s32)conf_now->foc_overmod_factor,
                                  0L,
                                  MCS_Q15_ONE);
    
    max_v_mag = Foc_MulQ15(max_duty, overmod_factor);
    max_v_mag = Foc_MulQ15(max_v_mag, MCS_SQRT3_BY_2_Q15);

    id = (((s32)state_m->i_alpha * trig.cos) +
          ((s32)state_m->i_beta * trig.sin)) >> 15;
    iq = (((s32)state_m->i_beta * trig.cos) -
          ((s32)state_m->i_alpha * trig.sin)) >> 15;

    state_m->id = Foc_SatS16(id);
    state_m->iq = Foc_SatS16(iq);

    /* 滤波值只供监控和慢速逻辑使用，PI反馈仍使用未滤波电流。 */
    state_m->id_filter = Foc_SatS16((s32)state_m->id_filter +
        Foc_MulQ15((s32)conf_now->foc_current_filter_const,
                   (s32)state_m->id - (s32)state_m->id_filter));
    state_m->iq_filter = Foc_SatS16((s32)state_m->iq_filter +
        Foc_MulQ15((s32)conf_now->foc_current_filter_const,
                   (s32)state_m->iq - (s32)state_m->iq_filter));

    Ierr_d = (s32)state_m->id_target - id;
    Ierr_q = (s32)state_m->iq_target - iq;
    state_m->id_error = Ierr_d;
    state_m->iq_error = Ierr_q;

    kp = (s32)conf_now->foc_current_kp;
    ki = conf_now->foc_temp_comp ?
         (s32)motor->m_current_ki_temp_comp :
         (s32)conf_now->foc_current_ki;

    /* Kp/Ki输出直接是Q15调制度，Ki按快环tick积分。 */
    
    // Kp
    p_d = Foc_SatS32FromS64((int64_t)Ierr_d * (int64_t)kp);
    p_q = Foc_SatS32FromS64((int64_t)Ierr_q * (int64_t)kp);

    // Ki
    tmp64 = (int64_t)Ierr_d * (int64_t)ki * (int64_t)dt;
    vd_int = Foc_SatS32FromS64((int64_t)state_m->vd_int + tmp64);
    tmp64 = (int64_t)Ierr_q * (int64_t)ki * (int64_t)dt;
    vq_int = Foc_SatS32FromS64((int64_t)state_m->vq_int + tmp64);
    
    vd = Foc_SatS32FromS64((int64_t)vd_int + p_d);
    vq = Foc_SatS32FromS64((int64_t)vq_int + p_q);

    /* 可选解耦：速度、Ld/Lq增益和磁链补偿量均按Q15约定。 */
    if(conf_now->foc_cc_decoupling != FOC_CC_DECOUPLING_DISABLED)
    {
        if((conf_now->foc_cc_decoupling == FOC_CC_DECOUPLING_CROSS) ||
           (conf_now->foc_cc_decoupling == FOC_CC_DECOUPLING_CROSS_BEMF))
        {
            tmp64 = (int64_t)iq * (int64_t)motor->m_speed_est_fast *
                    (int64_t)motor->p_lq;
            dec_vd = Foc_SatS32FromS64(tmp64 >> 30);

            tmp64 = (int64_t)id * (int64_t)motor->m_speed_est_fast *
                    (int64_t)motor->p_ld;
            dec_vq = Foc_SatS32FromS64(tmp64 >> 30);
        }

        if((conf_now->foc_cc_decoupling == FOC_CC_DECOUPLING_BEMF) ||
           (conf_now->foc_cc_decoupling == FOC_CC_DECOUPLING_CROSS_BEMF))
        {
            dec_bemf = Foc_MulQ15((s32)motor->m_speed_est_fast,
                                  (s32)conf_now->foc_motor_flux_linkage);
        }
    }

    vd = Foc_SatS32FromS64((int64_t)vd - dec_vd);
    vq = Foc_SatS32FromS64((int64_t)vq + dec_vq + dec_bemf);

    
    /* d轴优先分配电压，积分项同步限幅，防止饱和时积分继续累积。 */
    vd = Foc_LimitS32(vd, -max_v_mag, max_v_mag);
    vd_int = Foc_LimitS32(vd_int, -max_v_mag, max_v_mag);

    vq_square = (u32)((int64_t)max_v_mag * max_v_mag -
                      (int64_t)vd * vd);
    max_vq = (s32)utils_sqrt_u32(vq_square);
    vq = Foc_LimitS32(vq, -max_vq, max_vq);
    vq_int = Foc_LimitS32(vq_int, -max_vq, max_vq);

    state_m->vd_int = vd_int;
    state_m->vq_int = vq_int;
    state_m->vd = Foc_SatS16(vd);
    state_m->vq = Foc_SatS16(vq);
    state_m->mod_d = state_m->vd;
    state_m->mod_q = state_m->vq;
    
    // 计算电流幅值
    tmp64 = (int64_t)state_m->id_filter * state_m->id_filter +
            (int64_t)state_m->iq_filter * state_m->iq_filter;
    //        
    state_m->i_abs_filter = Foc_SatS16((s32)utils_sqrt_u32((u32)tmp64));
    
    
    // 反Park变换
    alpha = ((vd * trig.cos) - (vq * trig.sin)) >> 15;
    beta = ((vd * trig.sin) + (vq * trig.cos)) >> 15;

    state_m->mod_alpha = Foc_SatS16(alpha);
    state_m->mod_beta = Foc_SatS16(beta);
    
    state_m->mod_alpha_raw = state_m->mod_alpha;
    state_m->mod_beta_raw = state_m->mod_beta;

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
