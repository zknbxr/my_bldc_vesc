#ifndef MCS_MOTOR_INTERNAL_H
#define MCS_MOTOR_INTERNAL_H

#include "mcs_motor.h"

MCS_MOTOR_START_RESULT Motor_CheckStartSafety(void);
void Motor_BlockStart(MCS_MOTOR_START_RESULT reason);
void Motor_SetLastStartResult(MCS_MOTOR_START_RESULT result);
void Motor_WriteNeutralPwm(void);
void Motor_WriteDVector(uint16_t angle, int16_t dRef);
void Motor_ServiceSensorlessFocAutoStart(void);
void Motor_ServiceSensorlessFoc(void);

#endif
