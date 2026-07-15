#include "main.h"

#define OBSERVER_MILLI_SCALE               (1000L)
#define OBSERVER_CORDIC_ITERATIONS         (15U)

/*
 * VESC 中 MxLemming 电压模型观测器的定点移植。
 * 原始观测器算法作者为 MESC 项目的 David Molony，复制或修改时需保留署名。
 *
 * 本实现不使用浮点数，各物理量单位如下：
 *   v_alpha/v_beta : mV
 *   i_alpha/i_beta : mA
 *   foc_motor_r    : mOhm
 *   foc_motor_l    : uH
 *   dt             : us
 *   x1/x2          : uWb
 *   phase          : 一圈对应 65536
 */

static const u16 observer_atan_table[OBSERVER_CORDIC_ITERATIONS] = {
    8192U, 4836U, 2555U, 1297U, 651U,
    326U, 163U, 81U, 41U, 20U,
    10U, 5U, 3U, 1U, 1U
};

static s32 observer_sat_s32(int64_t value)
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
static u16 observer_atan2(s32 y, s32 x)
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
}

void foc_observer_update(s32 v_alpha, s32 v_beta,
                         s32 i_alpha, s32 i_beta,
                         u16 dt, observer_state *state,
                         s16 *phase, motor_all_state_t *motor)
{
    mc_configuration *conf_now;
    s32 r_i_alpha;
    s32 r_i_beta;
    s32 x1_step;
    s32 x2_step;
    s32 lambda;
    u32 mag_sq;
    u32 mag;

    conf_now = motor->m_conf;
    lambda = conf_now->foc_motor_flux_linkage;
    if((lambda <= 0L) || (dt == 0U))
    {
        state->i_alpha_last = i_alpha;
        state->i_beta_last = i_beta;
        return;
    }

    /* x1^2 + x2^2 在该范围内不会超过有符号 32 位上限。 */
    if(lambda > 32767L)
    {
        lambda = 32767L;
    }

    /* R[mOhm] * I[mA] / 1000 = mV。 */
    r_i_alpha = observer_div_1000(observer_sat_s32(
        (int64_t)conf_now->foc_motor_r * i_alpha));
    r_i_beta = observer_div_1000(observer_sat_s32(
        (int64_t)conf_now->foc_motor_r * i_beta));

    /*
     * VESC MxLemming 的核心磁链方程：
     *   x += (v - R*i) * dt - L * (i - i_last)
     * mV*us 和 uH*mA 都是 nWb，除以 1000 后得到 uWb。
     */
    x1_step = observer_div_1000(observer_sat_s32(
        (int64_t)(v_alpha - r_i_alpha) * dt -
        (int64_t)conf_now->foc_motor_l * (i_alpha - state->i_alpha_last)));
    x2_step = observer_div_1000(observer_sat_s32(
        (int64_t)(v_beta - r_i_beta) * dt -
        (int64_t)conf_now->foc_motor_l * (i_beta - state->i_beta_last)));

    state->x1 = observer_truncate_abs(observer_sat_s32(
        (int64_t)state->x1 + x1_step), lambda);
    state->x2 = observer_truncate_abs(observer_sat_s32(
        (int64_t)state->x2 + x2_step), lambda);
    state->i_alpha_last = i_alpha;
    state->i_beta_last = i_beta;

    /* 磁链过小时角度对噪声很敏感，按 VESC 的处理把幅值轻微向外推。 */
    mag_sq = (u32)(state->x1 * state->x1) +
             (u32)(state->x2 * state->x2);
    mag = utils_sqrt_u32(mag_sq);

    if(mag < (u32)(lambda / 2L))
    {
        state->x1 = observer_truncate_abs(
            state->x1 + state->x1 / 10L, lambda);
        state->x2 = observer_truncate_abs(
            state->x2 + state->x2 / 10L, lambda);
    }

    if(phase != 0)
    {
        *phase = (s16)observer_atan2(state->x2, state->x1);
    }
}
