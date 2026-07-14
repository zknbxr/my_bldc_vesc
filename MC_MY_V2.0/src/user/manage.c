#include "main.h"

volatile u16 TickCounter = 0;
UART0_Data UART0_Message;
uint8_t uUart0_Counter;
u8 TickRecevice;										//定时接收
volatile u8 KeyState;											//键值
u8 LastKeyState;										//上一个键值
u16 Uart_TargetHeight;									//接收目标行程
float Uart_TargetSpeed = 1;//串口接收目标速度



void Task_vTickTimerEvent(void)
{
	
	TickCounter++;
}


void User_app_init(void)
{

}

