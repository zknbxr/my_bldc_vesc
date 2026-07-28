#ifndef MCS_MATH_H
#define MCS_MATH_H

#include <stdint.h>

/*
 * 通用定点数学原语。
 *
 * 简短且位于ADC快速环中的函数放在头文件内联，避免统一接口后增加函数调用
 * 开销；步进、向量比较、开方等较长函数实现在mcs_math.c中。
 */
static __inline int32_t McsMath_LimitS32(int32_t value,
                                        int32_t min_value,
                                        int32_t max_value)
{
    if(value > max_value)
    {
        return max_value;
    }
    if(value < min_value)
    {
        return min_value;
    }
    return value;
}

static __inline int16_t McsMath_SatS16(int32_t value)
{
    if(value > 32767L)
    {
        return 32767;
    }
    if(value < -32768L)
    {
        return -32768;
    }
    return (int16_t)value;
}

/*
 * 有符号绝对值无法表示abs(INT32_MIN)，因此该输入饱和到INT32_MAX。
 * 需要保留完整2147483648幅值时使用McsMath_AbsU32。
 */
static __inline int32_t McsMath_AbsS32(int32_t value)
{
    if(value == (-2147483647L - 1L))
    {
        return 2147483647L;
    }
    return (value < 0L) ? -value : value;
}

static __inline int32_t McsMath_LimitAbsS32(int32_t value,
                                           int32_t max_abs)
{
    max_abs = McsMath_AbsS32(max_abs);
    return McsMath_LimitS32(value, -max_abs, max_abs);
}

static __inline uint32_t McsMath_AbsU32(int32_t value)
{
    if(value >= 0L)
    {
        return (uint32_t)value;
    }
    return (uint32_t)(-(value + 1L)) + 1UL;
}

/*
 * Q15乘法。调用者应保证a*b不会超过s32；FOC调制度和正余弦均满足此条件。
 * 保持32位乘法可避免Cortex-M0快速环引入64位乘法开销。
 */
static __inline int32_t McsMath_MulQ15(int32_t a, int32_t b)
{
    return (a * b) >> 15;
}

/* Q16圆周角最短有符号差值，返回范围-32768...32767。 */
static __inline int16_t McsMath_PhaseDiffQ16(int16_t phase,
                                            int16_t reference)
{
    return (int16_t)((uint16_t)phase - (uint16_t)reference);
}

int32_t McsMath_StepTowardsS32(int32_t value,
                               int32_t target,
                               int32_t step);
int16_t McsMath_StepTowardsS16(int16_t value,
                               int16_t target,
                               int32_t step);
int32_t McsMath_MaxAbs3S32(int32_t a, int32_t b, int32_t c);

void utils_truncate_number(int32_t *number, int32_t min, int32_t max);
void utils_truncate_number_abs(int32_t *number, int32_t max);
int32_t utils_min_abs(int32_t va, int32_t vb);
int32_t utils_max_abs(int32_t va, int32_t vb);
uint32_t utils_sqrt_u32(uint32_t value);

void FOC_SVM_Q15(int16_t alpha_q15_in,
                 int16_t beta_q15_in,
                 int32_t max_mod_q15,
                 uint32_t PWMFullDutyCycle,
                 uint32_t *tAout,
                 uint32_t *tBout,
                 uint32_t *tCout,
                 uint32_t *svm_sector);

#endif
