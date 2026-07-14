#ifndef MCS_MOTOR_H
#define MCS_MOTOR_H

#include <stdint.h>
#include "mcs_state_type.h"

/*
 * MCS 对外电机接口。
 *
 * APP 层不要直接调用 PWM 寄存器。
 * 启动、停止、状态查询都从这里进入，保证安全门控集中在 mcs_motor.c。
 */

/* 对外方向定义，MCS 内部再映射到实际 PWM/FOC 方向。 */
typedef enum
{
    MCS_MOTOR_DIR_REVERSE = -1,
    MCS_MOTOR_DIR_FORWARD = 1
} MCS_MOTOR_DIR;

/* 启动门控结果，数值保持简单，方便在 Keil watch 中观察。 */
typedef enum
{
    MCS_MOTOR_START_OK = 0,
    MCS_MOTOR_START_BLOCKED_FAULT,
    MCS_MOTOR_START_BLOCKED_BUS_VOLTAGE,
    MCS_MOTOR_START_BLOCKED_PHASE_CURRENT,
    MCS_MOTOR_START_BLOCKED_HALL_NOT_READY,
    MCS_MOTOR_START_BLOCKED_FLUX_NOT_READY,
    MCS_MOTOR_START_NOT_IMPLEMENTED
} MCS_MOTOR_START_RESULT;



void Motor_Init(void);

void MCS_Motor_Stop(void);
uint8_t Motor_IsRunning(void);


#endif
