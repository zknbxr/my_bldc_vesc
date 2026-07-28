#include "main.h"

/* User layer jobs are grouped here from always/fast to slow periods. */

static void User_App_DispatchUartCommand(void);
static void User_Uart_Receive_Check(void);
static void AnalyseUART0(void);
uint8_t UARTADD(uint8_t *pStream,uint8_t uLen,uint8_t uDir);
static void Run_Status_Check(void);
static u16 s_lastTargetHeight = APP_POSITION_MANUAL_TARGET;

void User_App_Task_Run(const TASK_TICK *tick)
{
    if(tick == NULL)
    {
        return;
    }

    if(tick->ms1 != 0U)
    {
        // 看门狗，原来在adc中断中
        FeedDogcmd();
        
        // 连续3ms没接收到数据就认为数据接收完毕 
        User_Uart_Receive_Check();
        AppHeight_Task1ms(tick->ms1);
    }
    if(tick->ms10 != 0U)
    {
        AppHeight_Task10ms();
    }
    if(tick->ms100 != 0U) {
        Uartrx_Error_Check();
    }
}

static void User_App_DispatchUartCommand(void)
{
	/* 学习模式由霍尔状态机独占电机控制权，串口命令只接收但不执行。 */
	if(gMotorWorkMode != MCS_WORK_MODE_CONTROL){
		return;
	}

	/* 命令字和目标行程都不变时，不重复触发状态转换。 */
	if((KeyState == LastKeyState) &&
	   (Uart_TargetHeight == s_lastTargetHeight)){
		return;
	}
    
// 没必要，后期不止这几个命令
//	if((KeyState != Motor_Up) &&
//			(KeyState != Motor_Down) &&
//			(KeyState != Motor_Stop)){
//		gUartRxErrorCount++;
//		return;
//	}

	/*
	 * 先记录命令边沿。故障期间的命令变化只被记录而不执行，因此恢复后持续
	 * 重发同一个命令不会自动启动，必须再次出现不同的新命令。
	 */
	LastKeyState = KeyState;
	s_lastTargetHeight = Uart_TargetHeight;
	if(Motor_FaultIsActive()){
		return;
	}

	AppHeight_HandleCommand(KeyState, Uart_TargetHeight);
}


static void User_Uart_Receive_Check(void)
{
	/* 只有收到帧头后才计时；连续3ms没有新字节表示一帧接收完成。 */
	if((UART0_Message.RecStatus == UART_REC) && (UART0_Message.R_Index > 0U)){
		if(TickRecevice < 3U){
			TickRecevice++;
		}
		if(TickRecevice >= 3U){
			UART0_Message.RecStatus = UART_RECD;
		}
	}else{
		TickRecevice = 0U;
	}
	
	if(uUart0_Counter < 100)
	{
		uUart0_Counter ++;
	}
}

void User_Task_Always(void)
{
    AnalyseUART0();
}


static void AnalyseUART0(void)
{
	u8 command = 0U;
	u8 length;
	u8 frameValid = 0U;

	if(UART0_Message.RecStatus == UART_RECD){
		length = UART0_Message.R_Index;

		/*
		 * 固定9字节协议：
		 * AA AA 地址 命令 速度(当前保留) 电流 行程高 行程低 校验和
		 * 校验和为BYTE2到BYTE7的8位累加结果。
		 */
		if((length == UART_RX_FRAME_LEN) &&
				(UART0_Message.RXBuffer[0] == Head_RxH) &&
				(UART0_Message.RXBuffer[1] == Head_RxL) &&
				(UART0_Message.RXBuffer[2] == Motor_Address) &&
				UARTADD(UART0_Message.RXBuffer, 6U, 0U)){
			command = UART0_Message.RXBuffer[3];
			Uart_TargetHeight =
				((u16)UART0_Message.RXBuffer[6] << 8) |
				(u16)UART0_Message.RXBuffer[7];
			frameValid = 1U;
		}

		UART0_Message.RecStatus = UART_STY;
		UART0_Message.R_Index = 0U;
		TickRecevice = 0U;

		if(frameValid != 0U){
			KeyState = command;
			gUartLastCommand = command;
			gUartRxFrameCount++;
			uUart0_Counter = 0U;
			rxerror_timecount = 0U;
			User_App_DispatchUartCommand();
		}else{
			gUartRxErrorCount++;
		}
	}
	
	if(uUart0_Counter == 5){
		uUart0_Counter = 10;
		Run_Status_Check();
	}
}

uint8_t UARTADD(uint8_t *pStream,uint8_t uLen,uint8_t uDir)
{
	uint8_t	uTemp;	
	
	uTemp = 0;

	pStream+=2;
			
	while(uLen--){		
		uTemp = uTemp + (*pStream++);
	}		
			
	if(uDir)// 计算校验值并保存在最后两字节	
	{	 		 
		*pStream++ = uTemp;	 //CRC低字节 	  	  
		return 1;
	}	
	else	// 用于校验	
	{
		if(*pStream++ != uTemp )	return 0;	
		else 						return 1;
	}
}


uint16_t uCounter_Tex1;
u8 	rxerror_timecount;//无接收计数
static void Run_Status_Check(void)
{
	#if(1)
	
	uCounter_Tex1 ++;
	
	if(KeyState != Motor_SET)
	{
        UART0_Message.TXBuffer[0] = Head_TxH;
        UART0_Message.TXBuffer[1] = Head_TxL;
		UART0_Message.TXBuffer[2] = Motor_Address;
		UART0_Message.TXBuffer[3] = m_motor.m_run_state;//Run_CurrentStatus
		UART0_Message.TXBuffer[4] = (gAppHeight.position.current_01mm >> 8) & 0xFF;
		UART0_Message.TXBuffer[5] = gAppHeight.position.current_01mm & 0xFF;
		UART0_Message.TXBuffer[6] = 0;
		UART0_Message.TXBuffer[8] = 0;
		UART0_Message.TXBuffer[9] = 0;
		UART0_Message.TXBuffer[10] = 0;
		UARTADD(UART0_Message.TXBuffer,6,1);
		UART0_Message.MAX_Len  = 9;
		UART0_Message.T_Index  = 1;
		UART0_Message.RecStatus = UART_TRN;
		UART_SendData(UART0, UART0_Message.TXBuffer[0]);
	}
	#else
	int height_chage;
	
	uCounter_Tex1 ++;
	
	height_chage = MAX_TRAVEL_DISTANCE - gDeskMianMbr.currentHeight;
	
	if(KeyState != Motor_SET)
	{
        UART0_Message.TXBuffer[0] = Head_TxH;
        UART0_Message.TXBuffer[1] = Head_TxL;
		UART0_Message.TXBuffer[2] = Motor_Address;
		UART0_Message.TXBuffer[3] = 1; //电机运行状态，预留
		UART0_Message.TXBuffer[4] = 0; //电机高度高位，预留
		UART0_Message.TXBuffer[5] = 0; //电机高度低位，预留;
		UART0_Message.TXBuffer[6] = 0; //错误码;
		UART0_Message.TXBuffer[8] = 0;
		UART0_Message.TXBuffer[9] = 0;
		UART0_Message.TXBuffer[10] = 0;
		UARTADD(UART0_Message.TXBuffer,6,1);
		UART0_Message.MAX_Len  = 9;
		UART0_Message.T_Index  = 1;
		UART0_Message.RecStatus = UART_TRN;
		UART_SendData(UART0, UART0_Message.TXBuffer[0]);
	}
	#endif
}

void Uartrx_Error_Check(void)
{	
	if(rxerror_timecount < 10)
	{
		rxerror_timecount++;
	}
    if(rxerror_timecount >= 5) {
        
    }
	// 0.5s接收不到串口数据停止电机，预留
}
