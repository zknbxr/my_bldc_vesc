#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include <stdint.h>
#include "Type.h"
#include "mcs_motor.h"

/* APP业务层最小状态机。
 * 这里只描述升降桌业务意图，不描述FOC/PWM内部状态。
 */
typedef enum
{
    APP_CONTROL_IDLE = 0,       /* 空闲：当前没有运动请求。 */
    APP_CONTROL_RUNNING_UP,     /* 上升运行：APP已经请求MCS正向开环运行。 */
    APP_CONTROL_RUNNING_DOWN,   /* 下降运行：APP已经请求MCS反向开环运行。 */
    APP_CONTROL_FAULT           /* 故障：MCS上报错误后APP进入故障态。 */
} APP_CONTROL_STATE;

/* 初始化APP业务状态机，恢复空闲状态。 */
void AppControl_Init(void);

/* APP常驻任务：检查MCS只读状态快照，处理故障态切换。 */
void AppControl_TaskAlways(void);

/* 消费串口层解析出的命令，并转换为APP业务状态切换。 */
void AppControl_HandleUartCommand(const APP_UART_COMMAND *cmd);

/* 获取APP当前业务状态，供调试或后续上报使用。 */
APP_CONTROL_STATE AppControl_GetState(void);

#endif
