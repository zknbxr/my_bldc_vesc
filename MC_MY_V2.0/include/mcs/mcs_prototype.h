#ifndef MCS_PROTOTYPE_H
#define MCS_PROTOTYPE_H

#include <stdint.h>
#include "mcs_motor_type.h"

/*
    Task
 **/
void mc_sys_init(void);
void Mcs_Task_Run(const TASK_TICK *tick);
void User_Task_Always(void);

/*
    User_App
 **/
void Uartrx_Error_Check(void);
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
void Motor_FaultInit(void);
void Motor_FaultTask1ms(u16 elapsed_ms);
void Motor_FaultTrip(motor_fault_t fault);
bool Motor_FaultIsActive(void);

/*
	interrupt.c
**/
void FeedDogcmd(void);

/*
    mcs_motor.c
**/

MCS_TRIG_Q15 Motor_GetSinCosQ15(u16 angle);
void Motor_WriteDqVector(u16 angle, s16 dRef, s16 qRef);

/*
    mcs_direction_test.c
**/
void Direction_Init(void);
void Motor_DirectionTest_Task(void);
void Motor_DirectionTest_Stop(void);

/*
    sensorless_ctrl.c
**/
void foc_sensorless_update(motor_all_state_t *motor);
u16 Foc_Atan2Q16(s32 y, s32 x);

/*
	mcs_foc_hw.c
**/
void AdcSampleCal(void);
void AdcEocHandler(void);
void Motor_FocSlowUpdate1ms(void);
void PwmAOutputs(FuncState t_state);
void Motor_WriteNeutralPwm(void);
s32 FocHw_ModToVoltageMv(s16 modulation, s32 bus_voltage);

//mcs_control.c
void CurrentErrDetc(void);
void StopMotorImmdly(void);
void Motor_ControlInit(void);
void Motor_ControlTask1ms(u16 elapsed_ms);
bool Motor_IsControlRunning(const motor_all_state_t *motor);
bool Motor_IsControlActive(const motor_all_state_t *motor);
bool Motor_IsPwmEnabled(void);
bool Motor_IsRotorMoving(const motor_all_state_t *motor, s16 min_erpm);
void Motor_SpeedControlUpdate1ms(motor_all_state_t *motor, u16 elapsed_ms);
void Motor_SetCurrentTarget(motor_all_state_t *motor,
                            s16 id_target_ma,
                            s16 iq_target_ma);
void Motor_CurrentCommandUpdate(motor_all_state_t *motor, u16 elapsed_ms);
void Motor_CurrentCommandReset(motor_all_state_t *motor);

#endif

