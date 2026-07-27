#include "main.h"

volatile u16 TickCounter = 0;
UART0_Data UART0_Message;
uint8_t uUart0_Counter;
volatile u8 TickRecevice;							// 最后一个接收字节后的等待时间，单位ms
volatile u8 KeyState;											//键值
u8 LastKeyState;										//上一个键值
u16 Uart_TargetHeight = APP_POSITION_MANUAL_TARGET;		//接收目标行程，单位0.1mm
float Uart_TargetSpeed = 1;//串口接收目标速度

/* 串口接收诊断量，可直接放入Keil Watch观察。 */
volatile u32 gUartRxByteCount;					// 进入接收中断的总字节数
volatile u32 gUartRxFrameCount;					// 成功解析的总帧数
volatile u32 gUartRxErrorCount;					// 帧头、长度、校验或命令错误次数
volatile u8 gUartLastRxByte;						// 最近收到的原始字节
volatile u8 gUartLastCommand;						// 最近成功解析的命令



void Task_vTickTimerEvent(void)
{
    if(TickCounter < 0xFFFFU)
    {
        TickCounter++;
    }
}


void User_app_init(void)
{
	UART0_Message.RecStatus = UART_STY;
	UART0_Message.R_Index = 0U;
	UART0_Message.T_Index = 0U;
	UART0_Message.MAX_Len = 0U;
	TickRecevice = 0U;
	uUart0_Counter = 0U;
	KeyState = Motor_Stop;
	LastKeyState = Motor_Stop;
	gUartRxByteCount = 0U;
	gUartRxFrameCount = 0U;
	gUartRxErrorCount = 0U;
	gUartLastRxByte = 0U;
	gUartLastCommand = 0U;
	Uart_TargetHeight = APP_POSITION_MANUAL_TARGET;
	AppHeight_Init();
}

