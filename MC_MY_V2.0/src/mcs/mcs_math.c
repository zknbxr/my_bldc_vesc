#include "main.h"

#define SVM_Q                         (15)
#define SVM_Q_ONE                     (1L << SVM_Q)   /* 32768 */


static u32 utils_abs_s32(s32 value)
{
    if(value >= 0)
    {
        return (u32)value;
    }

    return (u32)(-(value + 1L)) + 1UL;
}

void utils_truncate_number(s32 *number, s32 min, s32 max)
{
    if(number == 0)
    {
        return;
    }

    if(*number > max)
    {
        *number = max;
    }
    else if(*number < min)
    {
        *number = min;
    }
}

void utils_truncate_number_abs(s32 *number, s32 max)
{
    s32 max_abs;

    if(number == 0)
    {
        return;
    }

    if(max == (-2147483647L - 1L))
    {
        max_abs = 2147483647L;
    }
    else
    {
        max_abs = (max < 0) ? -max : max;
    }
    utils_truncate_number(number, -max_abs, max_abs);
}

s32 utils_min_abs(s32 va, s32 vb)
{
    return (utils_abs_s32(va) < utils_abs_s32(vb)) ? va : vb;
}

s32 utils_max_abs(s32 va, s32 vb)
{
    return (utils_abs_s32(va) > utils_abs_s32(vb)) ? va : vb;
}

u32 utils_sqrt_u32(u32 value)
{
    u32 result = 0U;
    u32 bit = 1UL << 30;

    while(bit > value)
    {
        bit >>= 2;
    }

    while(bit != 0U)
    {
        if(value >= (result + bit))
        {
            value -= result + bit;
            result = (result >> 1) + bit;
        }
        else
        {
            result >>= 1;
        }

        bit >>= 2;
    }

    return result;
}

static int32_t SVM_Q15_Mul(int32_t a, int32_t b)
{
   
    return (int32_t)((a * b) >> SVM_Q);
}

static int32_t SVM_Q15_ToPwmCount(int32_t q15_value, uint32_t pwm_period)
{
    
    return (int32_t)((q15_value * (int32_t)pwm_period) >> SVM_Q);
}

static void SVM_LimitInt32(int32_t *value, int32_t min_value, int32_t max_value)
{
    if (*value < min_value) {
        *value = min_value;
    } else if (*value > max_value) {
        *value = max_value;
    }
}

/*
 * alpha_q15, beta_q15:
 *      Q15 格式，范围约 -32768 ~ 32767
 *      例如 0.5 = 16384，-0.5 = -16384
 *
 * max_mod_q15:
 *      Q15 格式，建议用 int32_t
 *      32768 = 1.0
 *      31130 = 0.95
 *
 * PWMFullDutyCycle:
 *      PWM 周期计数值，例如 1714
 *
 * tAout, tBout, tCout:
 *      输出三相 PWM 比较值
 *
 * svm_sector:
 *      输出扇区 1~6
 */
void FOC_SVM_Q15(int16_t alpha_q15_in,
                        int16_t beta_q15_in,
                        int32_t max_mod_q15,
                        uint32_t PWMFullDutyCycle,
                        uint32_t *tAout,
                        uint32_t *tBout,
                        uint32_t *tCout,
                        uint32_t *svm_sector)
{
    uint32_t sector;

    int32_t alpha_q15 = (int32_t)alpha_q15_in;
    int32_t beta_q15  = (int32_t)beta_q15_in;

    /*
     * 用于扇区判断：
     * beta / sqrt(3)
     */
    int32_t beta_div_sqrt3_q15 = SVM_Q15_Mul(beta_q15, ONE_BY_SQRT3);

    /*
     * 1. 扇区判断
     */
    if (beta_q15 >= 0) {
        if (alpha_q15 >= 0) {
            /* quadrant I */
            if (beta_div_sqrt3_q15 > alpha_q15) {
                sector = 2;
            } else {
                sector = 1;
            }
        } else {
            /* quadrant II */
            if (-beta_div_sqrt3_q15 > alpha_q15) {
                sector = 3;
            } else {
                sector = 2;
            }
        }
    } else {
        if (alpha_q15 >= 0) {
            /* quadrant IV */
            if (-beta_div_sqrt3_q15 > alpha_q15) {
                sector = 5;
            } else {
                sector = 6;
            }
        } else {
            /* quadrant III */
            if (beta_div_sqrt3_q15 > alpha_q15) {
                sector = 4;
            } else {
                sector = 5;
            }
        }
    }

    /*
     * 2. 计算三相 PWM 计数值
     */
    int32_t tA = 0;
    int32_t tB = 0;
    int32_t tC = 0;

    switch (sector) {
    case 1:
    {
        /*
         * sector 1:
         * V1 = 100
         * V2 = 110
         */
        int32_t t1_q15 = alpha_q15 - SVM_Q15_Mul(beta_q15, ONE_BY_SQRT3);
        int32_t t2_q15 = SVM_Q15_Mul(beta_q15, TWO_BY_SQRT3);

        int32_t t1 = SVM_Q15_ToPwmCount(t1_q15, PWMFullDutyCycle);
        int32_t t2 = SVM_Q15_ToPwmCount(t2_q15, PWMFullDutyCycle);

        tA = ((int32_t)PWMFullDutyCycle + t1 + t2) / 2;
        tB = tA - t1;
        tC = tB - t2;
        break;
    }

    case 2:
    {
        /*
         * sector 2:
         * V2 = 110
         * V3 = 010
         */
        int32_t t2_q15 = alpha_q15 + SVM_Q15_Mul(beta_q15, ONE_BY_SQRT3);
        int32_t t3_q15 = -alpha_q15 + SVM_Q15_Mul(beta_q15, ONE_BY_SQRT3);

        int32_t t2 = SVM_Q15_ToPwmCount(t2_q15, PWMFullDutyCycle);
        int32_t t3 = SVM_Q15_ToPwmCount(t3_q15, PWMFullDutyCycle);

        tB = ((int32_t)PWMFullDutyCycle + t2 + t3) / 2;
        tA = tB - t3;
        tC = tA - t2;
        break;
    }

    case 3:
    {
        /*
         * sector 3:
         * V3 = 010
         * V4 = 011
         */
        int32_t t3_q15 = SVM_Q15_Mul(beta_q15, TWO_BY_SQRT3);
        int32_t t4_q15 = -alpha_q15 - SVM_Q15_Mul(beta_q15, ONE_BY_SQRT3);

        int32_t t3 = SVM_Q15_ToPwmCount(t3_q15, PWMFullDutyCycle);
        int32_t t4 = SVM_Q15_ToPwmCount(t4_q15, PWMFullDutyCycle);

        tB = ((int32_t)PWMFullDutyCycle + t3 + t4) / 2;
        tC = tB - t3;
        tA = tC - t4;
        break;
    }

    case 4:
    {
        /*
         * sector 4:
         * V4 = 011
         * V5 = 001
         */
        int32_t t4_q15 = -alpha_q15 + SVM_Q15_Mul(beta_q15, ONE_BY_SQRT3);
        int32_t t5_q15 = -SVM_Q15_Mul(beta_q15, TWO_BY_SQRT3);

        int32_t t4 = SVM_Q15_ToPwmCount(t4_q15, PWMFullDutyCycle);
        int32_t t5 = SVM_Q15_ToPwmCount(t5_q15, PWMFullDutyCycle);

        tC = ((int32_t)PWMFullDutyCycle + t4 + t5) / 2;
        tB = tC - t5;
        tA = tB - t4;
        break;
    }

    case 5:
    {
        /*
         * sector 5:
         * V5 = 001
         * V6 = 101
         */
        int32_t t5_q15 = -alpha_q15 - SVM_Q15_Mul(beta_q15, ONE_BY_SQRT3);
        int32_t t6_q15 = alpha_q15 - SVM_Q15_Mul(beta_q15, ONE_BY_SQRT3);

        int32_t t5 = SVM_Q15_ToPwmCount(t5_q15, PWMFullDutyCycle);
        int32_t t6 = SVM_Q15_ToPwmCount(t6_q15, PWMFullDutyCycle);

        tC = ((int32_t)PWMFullDutyCycle + t5 + t6) / 2;
        tA = tC - t5;
        tB = tA - t6;
        break;
    }

    case 6:
    default:
    {
        /*
         * sector 6:
         * V6 = 101
         * V1 = 100
         */
        int32_t t6_q15 = -SVM_Q15_Mul(beta_q15, TWO_BY_SQRT3);
        int32_t t1_q15 = alpha_q15 + SVM_Q15_Mul(beta_q15, ONE_BY_SQRT3);

        int32_t t6 = SVM_Q15_ToPwmCount(t6_q15, PWMFullDutyCycle);
        int32_t t1 = SVM_Q15_ToPwmCount(t1_q15, PWMFullDutyCycle);

        tA = ((int32_t)PWMFullDutyCycle + t6 + t1) / 2;
        tC = tA - t1;
        tB = tC - t6;
        break;
    }
    }

    /*
     * 3. 限幅
     */
    if (max_mod_q15 < 0) {
        max_mod_q15 = 0;
    } else if (max_mod_q15 > SVM_Q_ONE) {
        max_mod_q15 = SVM_Q_ONE;
    }

    int32_t t_max = (int32_t)(((int32_t)PWMFullDutyCycle *
                              (SVM_Q_ONE + max_mod_q15)) >> (SVM_Q + 1));

    SVM_LimitInt32(&tA, 0, t_max);
    SVM_LimitInt32(&tB, 0, t_max);
    SVM_LimitInt32(&tC, 0, t_max);

    *tAout = (uint32_t)tA;
    *tBout = (uint32_t)tB;
    *tCout = (uint32_t)tC;
    *svm_sector = sector;
}


