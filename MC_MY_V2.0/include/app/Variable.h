#include "stdint.h"
#include "stdbool.h"


extern u8 textbu;



extern UART0_Data UART0_Message;
extern uint8_t uUart0_Counter;
extern volatile u8 TickRecevice;
extern volatile u32 gUartRxByteCount;
extern volatile u32 gUartRxFrameCount;
extern volatile u32 gUartRxErrorCount;
extern volatile u8 gUartLastRxByte;
extern volatile u8 gUartLastCommand;


extern volatile u8 KeyState;											//¼üÖµ
extern u8 LastKeyState;
extern volatile u16 TickCounter;
extern u8 motor_run_errorflag;
extern DESK_MAIN_CMD_STR gDeskMianMbr;
extern tsAPP_NVM_Data sAPP_NVM_Data;
extern u8 rxerror_timecount;
