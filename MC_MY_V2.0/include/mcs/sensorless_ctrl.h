#ifndef SENSORLESS_CTRL_H
#define SENSORLESS_CTRL_H

#include "mcs_motor_type.h"

/*
 * 定点版接口沿用 VESC 的函数和状态命名。
 * 电压单位为 mV，电流单位为 mA，dt 单位为 us，相位一圈为 65536。
 */
void foc_observer_reset(observer_state *state);
void foc_observer_update(s32 v_alpha, s32 v_beta,
                         s32 i_alpha_avg, s32 i_beta_avg,
                         s32 i_alpha_now, s32 i_beta_now,
                         u16 dt, observer_state *state,
                         motor_all_state_t *motor);
void foc_observer_apply_correction(observer_state *state,
                                   motor_all_state_t *motor);
extern volatile s16 gObserverFluxErrorQ13;
extern volatile s16 gObserverCorrectionGainQ13;
extern volatile s16 gObserverPllPhaseError;
extern volatile s32 gObserverPllSpeedStepQ16;
void foc_observer_update_phase(const observer_state *state, s16 *phase);
void foc_observer_pll_run(s16 phase, observer_state *state,
                          motor_all_state_t *motor);

#endif
