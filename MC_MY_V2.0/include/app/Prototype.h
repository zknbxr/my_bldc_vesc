#ifndef __PROTOTYPE_H__
#define __PROTOTYPE_H__

#include <stdint.h>


void delay(u16 cnt);
void AnalyseUART0(void);
void Task_vTickTimerEvent(void);
void User_app_init(void);
typedef struct
{
    uint16_t ms1;
    uint16_t ms10;
    uint16_t ms100;
    uint16_t ms1000;
} TASK_TICK;

void User_App_Task_Run(const TASK_TICK *tick);
void AppUartComm_Init(void);

void AppUartComm_TaskAlways(void);
void AppUartComm_Task1ms(void);
void AppUartComm_Task100ms(void);



u8 AppUartComm_HasPendingCommand(void);
const APP_UART_COMMAND *AppUartComm_GetPendingCommand(void);
void AppUartComm_ClearPendingCommand(void);

const APP_UART_COMM_STATUS *AppUartComm_GetStatus(void);


#endif 
