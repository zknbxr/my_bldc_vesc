#include "main.h"

void UART0_init(void)
{

    UART_InitTypeDef UART_InitStruct;
    UART_StructInit(&UART_InitStruct);

    UART_InitStruct.BaudRate = 9600;//115200;                            /* 设置波特率9600 */
    UART_InitStruct.WordLength = UART_WORDLENGTH_8b;                     /* 发送数据长度8位 */
    UART_InitStruct.StopBits = UART_STOPBITS_1b;
    UART_InitStruct.FirstSend = UART_FIRSTSEND_LSB;                      /* 先发送LSB */
    UART_InitStruct.ParityMode = UART_Parity_NO;                         /* 无奇偶校验 */
    UART_InitStruct.IRQEna = UART_IRQEna_RcvOver | UART_IRQEna_SendOver; /* 接受中断使能 */
    UART_Init(UART0, &UART_InitStruct);

//    SYS_ModuleClockCmd(SYS_Module_UART, ENABLE);
//    SYS_CLK_DIV2 = 0x0000;
//    UART0->DIVL = 0x87;
//    UART0->DIVH = 0x13;
//    UART0->CTRL = 0x00;
//    UART0->INV  = 0;
//    UART0->ADR  = 0;
//    UART0->RE   = 0;
//    UART0->IE   = 0x03;
}
