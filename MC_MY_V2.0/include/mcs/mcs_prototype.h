#ifndef MCS_PROTOTYPE_H
#define MCS_PROTOTYPE_H

#include <stdint.h>
#include "mcs_motor_type.h"
void mc_sys_init(void);
void Mcs_Task_Run(const TASK_TICK *tick);

/*
	mcs_control.c
**/
void Motor_WritePwmCompare(u16 phaseA, u16 phaseB, u16 phaseC);
void Motor_FocInit(void);
void Motor_CurrentLoopRun(u16 dt);
void Motor_FocLoopRun(u16 angle);

/*
	mcs_error_handing.c
**/


/*
    mcs_motor.c
**/

MCS_TRIG_Q15 Motor_GetSinCosQ15(u16 angle);
void Motor_WriteDqVector(u16 angle, s16 dRef, s16 qRef);
/*
	mcs_foc_hw.c
**/
void AdcSampleCal(void);
void AdcEocHandler(void);
void PwmAOutputs(FuncState t_state);
void Motor_WriteNeutralPwm(void);
//mcs_control.c
void CurrentErrDetc(void);
void StopMotorImmdly(void);

//mcs_math.c
void utils_truncate_number(s32 *number, s32 min, s32 max);
void utils_truncate_number_abs(s32 *number, s32 max);
s32 utils_min_abs(s32 va, s32 vb);
s32 utils_max_abs(s32 va, s32 vb);
u32 utils_sqrt_u32(u32 value);

void FOC_SVM_Q15(int16_t alpha_q15_in,
                 int16_t beta_q15_in,
                 int32_t max_mod_q15,
                 uint32_t PWMFullDutyCycle,
                 uint32_t *tAout,
                 uint32_t *tBout,
                 uint32_t *tCout,
                 uint32_t *svm_sector);

#endif

