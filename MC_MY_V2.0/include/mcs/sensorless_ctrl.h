#ifndef SENSORLESS_CTRL_H
#define SENSORLESS_CTRL_H

#include "mcs_motor_type.h"

/*
 * 定点版接口沿用 VESC 的函数和状态命名。
 * 电压单位为 mV，电流单位为 mA，dt 单位为 us，相位一圈为 65536。
 */
void foc_observer_reset(observer_state *state);
void foc_observer_update(s32 v_alpha, s32 v_beta,
                         s32 i_alpha, s32 i_beta,
                         u16 dt, observer_state *state,
                         s16 *phase, motor_all_state_t *motor);

#endif
