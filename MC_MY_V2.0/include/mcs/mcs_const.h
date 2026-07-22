#ifndef MCS_CONST_H
#define MCS_CONST_H

#define ONE_BY_SQRT3          (18919L)
#define TWO_BY_SQRT3          (37837L)

/*
 * LKS32 ADC 结果为左对齐。下面的系数将 ADC 计数转换为 FOC 状态使用的整数单位。
 */
#define MCS_CURRENT_ADC_TO_MA_Q15          (19802L)
#define MCS_BUS_ADC_TO_MV_Q15              (75600L)
#define MCS_BUS_FILTER_Q15                 (8192L)
#define FOC_TWO_BY_THREE_Q15               (21845L)
#define FOC_CONTROL_DT_US                   ((u16)(1000000UL / PWM_FREQ))

/* 无感观测器每四个 PWM 周期更新一次磁链，后续步骤分散到独立时隙。 */
#define FOC_OBSERVER_FLUX_DIV               (4U)
#define FOC_OBSERVER_MAGNITUDE_SLOT         (1U)
#define FOC_OBSERVER_PHASE_SLOT             (2U)

/* 电机机械转向定义。正反转只改变转矩/速度指令符号，不改变 ADC 电流极性。 */
#define MCS_MOTOR_DIRECTION_FORWARD         (1)
#define MCS_MOTOR_DIRECTION_REVERSE         (-1)
#define MCS_MOTOR_DIRECTION_DEFAULT         MCS_MOTOR_DIRECTION_FORWARD

/* 操作模式的外层控制器选择；底层最终都由ADC中断中的电流环执行。 */
#define MCS_OPERATION_CONTROL_CURRENT        (0U)
#define MCS_OPERATION_CONTROL_SPEED          (1U)

/* 正式操作模式固定速度目标，以及PLL允许估算的最大电角速度，单位ERPM。 */
#define MCS_SPEED_TARGET_DEFAULT_ERPM         (10000)
#define MCS_SPEED_EST_MAX_ERPM                (20000L)


#define UTILS_LP_FAST(value, sample, filter_constant) \
    (value = value + ((s32)(filter_constant * (sample - value)) >> 15))
    
#define GET_INPUT_VOLTAGE() 2
#define SIGN(x)				(((x) < 0) ? -1 : 1)



#endif
