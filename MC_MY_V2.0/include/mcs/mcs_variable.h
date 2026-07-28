#ifndef MCS_VARIABLE_H
#define MCS_VARIABLE_H

#include "mcs_include.h"

/* ADC / 相电流采样状态。
 * 这些变量由初始化或 ADC 快路径更新，当前保留给 Keil watch 观察。
 */

extern s16 hPhaseAOffset;
extern s16 hPhaseBOffset;
extern motor_all_state_t m_motor;
extern volatile s16 ADC_curr_norm_value[3];
extern volatile u16 gMotorCurrentLimitMa;
extern volatile u16 gMotorStartDelayMs;
extern volatile u8 gMotorWorkMode;
extern volatile motor_speed_control_t gMotorSpeedControl;
#endif
