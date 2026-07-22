#include "main.h"

#define OBSERVER_MILLI_SCALE               (1000L)
#define OBSERVER_CORDIC_ITERATIONS         (15U)
#define OBSERVER_Q13_ONE                   (8192L)
#define OBSERVER_FLUX_ERROR_SHIFT          (13U)
/* 3.5 kHz 矫正频率下的离散 gamma/2 * lambda^2 * Ts 增益。 */
#define OBSERVER_GAIN_TS_Q13               (256L)
#define OBSERVER_PLL_KP_Q15                (4096L)
#define OBSERVER_PLL_KI_Q15                (64L)
#define OBSERVER_PLL_MAX_ERPM              MCS_SPEED_EST_MAX_ERPM
/* 一圈 65536 个角度计数，PLL 更新频率 = 14000 / 4 = 3500 Hz。 */
#define OBSERVER_PLL_STEP_Q16_PER_ERPM     (20452L)





/*
 * 带非线性磁链矫正的定点电压模型。
 * 状态量保存 eta = x_hat - L*i，因此 phase = atan2(eta)。
 *
 * 本实现不使用浮点数，各物理量单位如下：
 *   v_alpha/v_beta : mV
 *   i_alpha/i_beta : mA
 *   foc_motor_r    : mOhm
 *   foc_motor_l    : uH
 *   dt             : us
 *   x1/x2          : uWb
 *   phase          : 一圈对应 65536
 *
 * 观测器有意拆分为三个步骤：
 *   1. 积分电压模型和 L*di -> x1/x2 磁链矢量
 *   2. 将矢量幅值矫正到配置的电机磁链附近
 *   3. atan2 提取原始角度，再由 PLL 输出平滑角度和 ERPM
 *
 * 静止时几乎没有反电动势信息，因此直接无感启动可能先来回摆动，直到转子运动
 * 为观测器提供足够信号。
 */

volatile s16 gObserverFluxErrorQ13;
volatile s16 gObserverCorrectionGainQ13;
volatile s16 gObserverPllPhaseError;
volatile s32 gObserverPllSpeedStepQ16;
volatile s16 gObserverPhase;
volatile s16 gObserverPhaseError;

static s32 s_observerModAlphaSum;
static s32 s_observerModBetaSum;
static s32 s_observerIAlphaSum;
static s32 s_observerIBetaSum;
static u8 s_observerFluxSamples;
static u8 s_observerPostFluxSlot;


static const u16 observer_atan_table[OBSERVER_CORDIC_ITERATIONS] = {
    8192U, 4836U, 2555U, 1297U, 651U,
    326U, 163U, 81U, 41U, 20U,
    10U, 5U, 3U, 1U, 1U
};

static s32 observer_div_1000(s32 value)
{
    /* 除法前加入半个分母，使正负数都按最接近的整数取整。 */
    if(value >= 0L)
    {
        value += OBSERVER_MILLI_SCALE / 2L;
    }
    else
    {
        value -= OBSERVER_MILLI_SCALE / 2L;
    }

    return value / OBSERVER_MILLI_SCALE;
}

static s32 observer_truncate_abs(s32 value, s32 max_abs)
{
    if(value > max_abs)
    {
        return max_abs;
    }

    if(value < -max_abs)
    {
        return -max_abs;
    }

    return value;
}

/* 定点 CORDIC atan2，返回 0~65535 的无符号电角度。 */
u16 Foc_Atan2Q16(s32 y, s32 x)
{
    s32 x_before;
    s32 angle;
    u8 i;

    if((x == 0L) && (y == 0L))
    {
        return 0U;
    }

    angle = 0L;
    if(x < 0L)
    {
        x = -x;
        y = -y;
        angle = 32768L;
    }

    for(i = 0U; i < OBSERVER_CORDIC_ITERATIONS; i++)
    {
        x_before = x;
        if(y > 0L)
        {
            x += y >> i;
            y -= x_before >> i;
            angle += (s32)observer_atan_table[i];
        }
        else
        {
            x -= y >> i;
            y += x_before >> i;
            angle -= (s32)observer_atan_table[i];
        }
    }

    return (u16)angle;
}

void foc_observer_reset(observer_state *state)
{
    state->x1 = 0L;
    state->x2 = 0L;
    state->i_alpha_last = 0L;
    state->i_beta_last = 0L;
    state->pll_phase_q16 = 0UL;
    state->pll_speed_step_q16 = 0L;
    state->pll_initialized = false;
    gObserverFluxErrorQ13 = 0;
    gObserverCorrectionGainQ13 = 0;
    gObserverPllPhaseError = 0;
    gObserverPllSpeedStepQ16 = 0L;
}

/* 无感模式启停时在快速环内统一复位观测器及其分步流水线。 */
static void foc_sensorless_reset(motor_all_state_t *motor,
                                 motor_state_t *state)
{
    s16 phase_seed;

    phase_seed = state->phase;
    s_observerModAlphaSum = 0L;
    s_observerModBetaSum = 0L;
    s_observerIAlphaSum = 0L;
    s_observerIBetaSum = 0L;
    s_observerFluxSamples = 0U;
    s_observerPostFluxSlot = 0U;

    foc_observer_reset(&motor->m_observer_state);
    motor->m_observer_state.i_alpha_last = state->i_alpha;
    motor->m_observer_state.i_beta_last = state->i_beta;
    motor->m_phase_control_initialized = false;
    motor->m_phase_control_q16 = (u32)(u16)phase_seed << 16;
    motor->m_phase_now_observer = phase_seed;
    motor->m_pll_phase = phase_seed;
    motor->m_pll_speed = 0;
    gObserverPhase = phase_seed;
    gObserverPhaseError = 0;
}

/* PLL 校正之间按速度状态逐 PWM 周期外推控制角。 */
static void foc_observer_update_predicted_phase(motor_all_state_t *motor,
                                                bool pll_updated)
{
    observer_state *observer;

    observer = &motor->m_observer_state;
    if(!observer->pll_initialized)
    {
        motor->m_phase_control_initialized = false;
        return;
    }

    if(pll_updated || !motor->m_phase_control_initialized)
    {
        motor->m_phase_control_q16 = observer->pll_phase_q16;
        motor->m_phase_control_initialized = true;
    }
    else
    {
        motor->m_phase_control_q16 += (u32)(
            observer->pll_speed_step_q16 / (s32)FOC_OBSERVER_FLUX_DIV);
    }

    motor->m_pll_phase = (s16)(motor->m_phase_control_q16 >> 16);
}

/* 对静止坐标系电压模型积分，估算转子磁链。 */
void foc_observer_update(s32 v_alpha, s32 v_beta,
                         s32 i_alpha_avg, s32 i_beta_avg,
                         s32 i_alpha_now, s32 i_beta_now,
                         u16 dt, observer_state *state,
                         motor_all_state_t *motor)
{
    mc_configuration *conf_now;
    s32 r_i_alpha;
    s32 r_i_beta;
    s32 x1_step;
    s32 x2_step;
    s32 lambda;

    conf_now = motor->m_conf;
    lambda = conf_now->foc_motor_flux_linkage;
    if((lambda <= 0L) || (dt == 0U))
    {
        state->i_alpha_last = i_alpha_now;
        state->i_beta_last = i_beta_now;
        return;
    }

    if(lambda > 32767L)
    {
        lambda = 32767L;
    }

    /*
     * alpha/beta 两轴分别使用以下离散电压模型：
     *   eta += (v - R*i)*dt - L*(i_now - i_last)
     * eta 是转子磁链矢量，x1/x2 的单位为 uWb。
     * 配置中的 R、L 已按当前模型定义填写，除非修改配置含义，否则这里不要再乘 1.5。
     */
    /* mOhm * mA / 1000 = mV，即定子电阻压降。 */
    r_i_alpha = observer_div_1000(conf_now->foc_motor_r * i_alpha_avg);
    r_i_beta = observer_div_1000(conf_now->foc_motor_r * i_beta_avg);
    /* mV*us/1000 和 uH*mA/1000 的结果单位均为 uWb。 */
    x1_step = observer_div_1000(
        (v_alpha - r_i_alpha) * (s32)dt -
        conf_now->foc_motor_l * (i_alpha_now - state->i_alpha_last));
    x2_step = observer_div_1000(
        (v_beta - r_i_beta) * (s32)dt -
        conf_now->foc_motor_l * (i_beta_now - state->i_beta_last));

    state->x1 = observer_truncate_abs(state->x1 + x1_step, lambda);
    state->x2 = observer_truncate_abs(state->x2 + x2_step, lambda);
    state->i_alpha_last = i_alpha_now;
    state->i_beta_last = i_beta_now;
}

void foc_observer_apply_correction(observer_state *state,
                                   motor_all_state_t *motor)
{
    s32 lambda;
    s32 lambda_sq_scaled;
    s32 flux_error;
    s32 flux_error_scaled;
    s32 error_q13;
    s32 correction_gain_q13;
    s32 correction_x1;
    s32 correction_x2;
    u32 lambda_sq;
    u32 mag_sq;

    lambda = motor->m_conf->foc_motor_flux_linkage;
    if(lambda <= 0L)
    {
        gObserverFluxErrorQ13 = 0;
        gObserverCorrectionGainQ13 = 0;
        return;
    }
    if(lambda > 32767L)
    {
        lambda = 32767L;
    }
    /* 配置的目标磁链幅值平方。 */
    lambda_sq = (u32)(lambda * lambda);
    /* 观测得到的 x1/x2 磁链矢量幅值平方。 */
    mag_sq = (u32)(state->x1 * state->x1) +
             (u32)(state->x2 * state->x2);
    flux_error = (s32)lambda_sq - (s32)mag_sq;

    /* 不使用 int64，将 (lambda^2 - |eta|^2) / lambda^2 归一化为 Q13。 */
    lambda_sq_scaled = (s32)(lambda_sq >> OBSERVER_FLUX_ERROR_SHIFT);
    flux_error_scaled = flux_error / (1L << OBSERVER_FLUX_ERROR_SHIFT);
    if(lambda_sq_scaled <= 0L)
    {
        return;
    }

    error_q13 = (flux_error_scaled * OBSERVER_Q13_ONE) /
                lambda_sq_scaled;
    error_q13 = observer_truncate_abs(error_q13, OBSERVER_Q13_ONE);
    correction_gain_q13 =
        (error_q13 * OBSERVER_GAIN_TS_Q13) / OBSERVER_Q13_ONE;

    /*
     * 径向非线性矫正：只改变磁链矢量幅值，不直接旋转矢量。
     * 正误差增大幅值，负误差减小幅值。
     */
    correction_x1 =
        (state->x1 * correction_gain_q13) / OBSERVER_Q13_ONE;
    correction_x2 =
        (state->x2 * correction_gain_q13) / OBSERVER_Q13_ONE;
    state->x1 = observer_truncate_abs(
        state->x1 + correction_x1, lambda);
    state->x2 = observer_truncate_abs(
        state->x2 + correction_x2, lambda);

    gObserverFluxErrorQ13 = (s16)error_q13;
    gObserverCorrectionGainQ13 = (s16)correction_gain_q13;
}

void foc_observer_update_phase(const observer_state *state, s16 *phase)
{
    if(phase != 0)
    {
        /* 提取矫正后 alpha-beta 磁链矢量的原始电角度。 */
        *phase = (s16)Foc_Atan2Q16(state->x2, state->x1);
    }
}
void foc_observer_pll_run(s16 phase, observer_state *state,
                          motor_all_state_t *motor)
{
    s32 phase_error;
    s32 phase_correction_q16;
    s32 speed_correction_q16;
    s32 speed_limit_q16;
    s32 speed_erpm;

    /*
     * PLL 先根据速度状态预测角度，再通过 PI 将预测值校正到原始 CORDIC 角度。
     * pll_phase 是平滑后的控制角，pll_speed_step_q16 会换算为电气 RPM 供监控。
     */
    if(!state->pll_initialized)
    {
        state->pll_phase_q16 = (u32)(u16)phase << 16;
        state->pll_speed_step_q16 = 0L;
        state->pll_initialized = true;
    }
    else
    {
        state->pll_phase_q16 += (u32)state->pll_speed_step_q16;
        phase_error = (s16)((u16)phase -
            (u16)(state->pll_phase_q16 >> 16));

        phase_correction_q16 =
            phase_error * (OBSERVER_PLL_KP_Q15 * 2L);
        speed_correction_q16 =
            phase_error * (OBSERVER_PLL_KI_Q15 * 2L);
        state->pll_phase_q16 += (u32)phase_correction_q16;
        state->pll_speed_step_q16 += speed_correction_q16;

        speed_limit_q16 = OBSERVER_PLL_MAX_ERPM *
                          OBSERVER_PLL_STEP_Q16_PER_ERPM;
        state->pll_speed_step_q16 = observer_truncate_abs(
            state->pll_speed_step_q16, speed_limit_q16);
    }

    phase_error = (s16)((u16)phase -
        (u16)(state->pll_phase_q16 >> 16));
    speed_erpm = state->pll_speed_step_q16 /
                 OBSERVER_PLL_STEP_Q16_PER_ERPM;
    motor->m_pll_phase = (s16)(state->pll_phase_q16 >> 16);
    motor->m_pll_speed = (s16)speed_erpm;
    gObserverPllPhaseError = (s16)phase_error;
    gObserverPllSpeedStepQ16 = state->pll_speed_step_q16;
}

void foc_sensorless_update(motor_all_state_t *motor_now)
{
    motor_state_t *state_now;
    s32 observerVAlphaAvg;
    s32 observerVBetaAvg;
    s32 observerModAlphaAvg;
    s32 observerModBetaAvg;
    s32 observerIAlphaAvg;
    s32 observerIBetaAvg;
    bool pllUpdated;
    
    if((motor_now == 0) || (motor_now->m_conf == 0))
    {
        return;
    }

    state_now = &motor_now->m_motor_state;

    /* 退出运行或切换到其他传感器时，使下次进入无感必定重新初始化。 */
    if((motor_now->m_state != MC_STATE_RUNNING) ||
       (motor_now->m_conf->foc_sensor_mode != FOC_SENSOR_MODE_SENSORLESS))
    {
        motor_now->m_observer_initial = false;
        return;
    }

    if(!motor_now->m_observer_initial)
    {
        foc_sensorless_reset(motor_now, state_now);
        motor_now->m_observer_initial = true;
    }

    pllUpdated = false;
    
    /* 磁链更新前先对四个 PWM 周期的数据求平均，以降低计算频率。
     * 区间末端的瞬时电流单独保留，用于计算 L*di 项。
     */
    /* 先累加上一 PWM 周期的调制度，第四次才统一换算为物理电压。 */

    /* 有个问题，这里没有到达计数值时角度也是在更新的，根据当前速度惯性更新，所以会出现我如果用手捏住转子不让它转动，角度还是会在变化的情况。 
     * 不过暂时不考虑吧，因为我无感并不考虑转子绝对位置
     */
    s_observerModAlphaSum += state_now->mod_alpha_raw;
    s_observerModBetaSum += state_now->mod_beta_raw;
    s_observerIAlphaSum += state_now->i_alpha;
    s_observerIBetaSum += state_now->i_beta;
    s_observerFluxSamples++;

    if(s_observerFluxSamples >= FOC_OBSERVER_FLUX_DIV)
    {
        observerModAlphaAvg = s_observerModAlphaSum /
                              (s32)FOC_OBSERVER_FLUX_DIV;
        observerModBetaAvg = s_observerModBetaSum /
                             (s32)FOC_OBSERVER_FLUX_DIV;
        observerVAlphaAvg = FocHw_ModToVoltageMv(
            FocHw_SatS16(observerModAlphaAvg), state_now->v_bus);
        observerVBetaAvg = FocHw_ModToVoltageMv(
            FocHw_SatS16(observerModBetaAvg), state_now->v_bus);
        state_now->v_alpha = observerVAlphaAvg;
        state_now->v_beta = observerVBetaAvg;
        observerIAlphaAvg = s_observerIAlphaSum / (s32)FOC_OBSERVER_FLUX_DIV;
        observerIBetaAvg = s_observerIBetaSum / (s32)FOC_OBSERVER_FLUX_DIV;

        foc_observer_update(observerVAlphaAvg, observerVBetaAvg,
                            observerIAlphaAvg, observerIBetaAvg,
                            state_now->i_alpha, state_now->i_beta,
                            (u16)(FOC_CONTROL_DT_US * FOC_OBSERVER_FLUX_DIV),
                            &motor_now->m_observer_state, motor_now);

        s_observerModAlphaSum = 0L;
        s_observerModBetaSum = 0L;
        s_observerIAlphaSum = 0L;
        s_observerIBetaSum = 0L;
        s_observerFluxSamples = 0U;
        s_observerPostFluxSlot = FOC_OBSERVER_MAGNITUDE_SLOT;
    }
    else if(s_observerPostFluxSlot == FOC_OBSERVER_MAGNITUDE_SLOT)
    {
        foc_observer_apply_correction(
            &motor_now->m_observer_state, motor_now);
        s_observerPostFluxSlot = FOC_OBSERVER_PHASE_SLOT;
    }
    else if(s_observerPostFluxSlot == FOC_OBSERVER_PHASE_SLOT)
    {
        foc_observer_update_phase(
            &motor_now->m_observer_state,
            &motor_now->m_phase_now_observer);
        foc_observer_pll_run(
            motor_now->m_phase_now_observer,
            &motor_now->m_observer_state, motor_now);
        pllUpdated = true;
        gObserverPhase = motor_now->m_phase_now_observer;
        gObserverPhaseError = FocHw_PhaseDifference(
            motor_now->m_phase_now_observer, state_now->phase);
        s_observerPostFluxSlot = 0U;
    }

    foc_observer_update_predicted_phase(motor_now, pllUpdated);
    if((motor_now->m_phase_override == false) &&
       motor_now->m_phase_control_initialized)
    {
        /* 电流环使用逐 PWM 周期外推的 PLL 角度，而不是阶梯状原始观测角。 */
        state_now->phase = motor_now->m_pll_phase;
    }
}


