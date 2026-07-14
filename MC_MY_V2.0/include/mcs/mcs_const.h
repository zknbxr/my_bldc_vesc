#ifndef MCS_CONST_H
#define MCS_CONST_H

#define ONE_BY_SQRT3          (18919L)
#define TWO_BY_SQRT3          (37837L)


#define UTILS_LP_FAST(value, sample, filter_constant) \
    (value = value + ((s32)(filter_constant * (sample - value)) >> 15))
    
#define GET_INPUT_VOLTAGE() 2
#define SIGN(x)				(((x) < 0) ? -1 : 1)

#endif
