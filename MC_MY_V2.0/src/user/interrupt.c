/*******************************************************************************
 * 版权所有 (C)2019, Bright Power Semiconductor Co.ltd
 *
 * 文件名称： interrupt.c
 * 文件标识：
 * 内容摘要： 中断服务程序文件
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： BPS IOT TEAM.
 * 完成日期： 2019年10月1日
 *
 *
 *******************************************************************************/
#include "main.h"




UINT16 g_CMP_IF=0;
UINT16 g_CMP_DATA=0;
volatile UINT16 gMcpwmEifAtShort;
volatile UINT16 gMcpwmFail012AtShort;
volatile UINT16 gCmpDataAtShort;
volatile UINT16 gShortFaultCount;
#define CMP_0
#define CMP0_OUT ((CMP_DATA & (0x04))>>2)
#define CMP1_OUT ((CMP_DATA & (0x08))>>3)


/*******************************************************************************
 函数名称：    void FeedDogcmd(void)
 功能描述：    执行喂狗操作
 操作的表：    无
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 多任务访问:   该函数涉及全局表项操作，不可重入
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/02/20     V1.0           BPS IOT TEAM         创建
 *******************************************************************************/
void FeedDogcmd(void)
{
    static UINT16 FeedDog_cnt=0;

    if(++FeedDog_cnt>100)
    {
        FeedDog_cnt=0;
        IWDG_Feed();
    }
}
/*******************************************************************************
 函数名称：    void MCPWM0_IRQHandler(void)
 功能描述：    MCPWM0中断函数
 操作的表：    无
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 多任务访问:   该函数涉及全局表项操作，不可重入
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/02/20     V1.0           BPS IOT TEAM         创建
 *******************************************************************************/
void MCPWM0_IRQHandler(void)
{
    ADC_CFG |= BIT11;
    MCPWM_IF0 = BIT1 | BIT0;
}


/*******************************************************************************
 函数名称：    void ADC_IRQHandler(void)
 功能描述：    ADC中断函数
 操作的表：    无
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 多任务访问:   该函数涉及全局表项操作，不可重入
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/02/20     V1.0           BPS IOT TEAM         创建
 *******************************************************************************/
extern u16 FOC_angle;
s16 s16Timer1,s16Timer2;
volatile u16 errTimer;
volatile u16 errTimerMax;
/*
 * 函数功能: ADC 采样完成中断服务函数。
 * 触发时机: 由 PWM/ADC 采样时序触发, 是电机快速控制回路的核心中断之一。
 * 主要处理: 清中断标志、执行电流采样后续控制、推进系统时基任务, 并检查硬件级短路保护。
 */
void ADC_IRQHandler(void)
{
    s32 elapsedTicks;
    /* 清 ADC 中断标志, 避免重复进入同一次中断。 */
    ADC_IF |= BIT1|BIT0;
    /* 重新置位 ADC 配置位, 为下一次 PWM 触发采样做准备。 */
    ADC_CFG |= BIT11;
    
    
    // GPIO_SetBits(GPIO1, GPIO_Pin_4);

    /* 记录进入中断时的 PWM 计数值, 用于统计本次中断执行耗时。 */
    s16Timer1 = MCPWM_CNT0;

    /* ADC 采样后处理: 电流采样换算、FOC 控制步进、霍尔角度更新和母线电压更新。 */
    AdcEocHandler();
    
    
//    /* 推进系统软件定时基准, 供 1ms/10ms/100ms 等任务调度使用。 */
    Task_vTickTimerEvent();

    /* 统计本次 ADC 中断执行结束时的 PWM 计数值。 */
    s16Timer2 = MCPWM_CNT0;
    /* 计算中断执行时间差, 便于调试控制回路耗时裕量。 */
    elapsedTicks = (s32)s16Timer2 - (s32)s16Timer1;
    if(elapsedTicks < 0L)
    {
        elapsedTicks += (s32)PWM_PERIOD * 2L;
    }
    errTimer = (u16)elapsedTicks;
    if(errTimer > errTimerMax)
    {
        errTimerMax = errTimer;
    }

    // GPIO_ResetBits(GPIO1, GPIO_Pin_4);
	
    /* 检查 MCPWM 扩展故障标志 BIT4/BIT5, 一般用于桥臂短路/过流等硬件级保护。CMP直接输出到 */
    if((MCPWM_EIF & BIT4)||(MCPWM_EIF & BIT5))
    {
        gMcpwmEifAtShort = MCPWM_EIF;
        gMcpwmFail012AtShort = MCPWM_FAIL012;
        gCmpDataAtShort = CMP_DATA;
        gShortFaultCount++;
        /* 发现硬件保护故障后立即停机, 防止功率器件继续受冲击。 */
        StopMotorImmdly();
        /* 清对应 MCPWM 故障标志位。 */
        MCPWM_EIF = BIT4|BIT5;
        /* 上报短路故障到系统错误字。 */
    }

}


/*******************************************************************************
 函数名称：    void CMP_IRQHandler(void)
 功能描述：    CMP中断函数
 操作的表：    无
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 多任务访问:   该函数涉及全局表项操作，不可重入
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/02/20     V1.0           BPS IOT TEAM         创建
 *******************************************************************************/
//实际并未使用，CMP的过流保护直接由硬件链路到MCPWM，无需单独开NVIC中断，除非需要在软件层面捕获该事件进行特殊处理
 void CMP_IRQHandler(void)
{
    g_CMP_IF = CMP_IF;
    //中断触发
    if(CMP_IF & (BIT0))
    {
        volatile u8 t_bi;
        volatile u8 t_bcnt;

        t_bcnt = 0;

        for(t_bi = 0; t_bi < 5; t_bi++)
        {
            if(CMP_DATA & BIT0)                                                                        /* BIT14 CMP0 OUT Flag| BIT15 CMP1 OUT Flag */
            {
                t_bcnt ++;
            }
        }
        if(t_bcnt > 3)
        {
//            StopMotorImmdly();
//            g_CMP_DATA = MCPWM_EIF;
//            MCPWM_EIF = BIT4|BIT5;
//            SetSysErrorFlag(E_FAULT_SHORT_ERROR);
//			textbu = 2;
        }
    }
    CMP_IF = BIT0 | BIT1;
}

/*******************************************************************************
 函数名称：    void HardFault_Handler(void)
 功能描述：    HardFault中断函数
 操作的表：    无
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 多任务访问:   该函数涉及全局表项操作，不可重入
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/02/20     V1.0           BPS IOT TEAM         创建
 *******************************************************************************/
void HardFault_Handler(void)
{

    MCPWM_PRT = 0x0000DEAD;
    StopMotorImmdly();
    NVIC_SystemReset();

}

/*******************************************************************************
 函数名称：    void UART_IRQHandler(void)
 功能描述：    UART0中断处理函数
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2023/2/21      V1.0           LLYY                创建
 *******************************************************************************/
void UART_IRQHandler(void)
{
	uint8_t hRec = 0;
	
    if(UART0->IF & UART_IF_SendOver){
			UART0->IF = UART_IF_SendOver;
			if(UART0_Message.RecStatus == UART_TRN){
				if(UART0_Message.T_Index < UART0_Message.MAX_Len){
					UART_SendData(UART0,UART0_Message.TXBuffer[UART0_Message.T_Index++]); 			
				}
				else{
					UART0_Message.RecStatus = UART_STY;
					UART0_Message.T_Index = 0;				
				}	
			}
			else{
				UART0_Message.RecStatus = UART_STY;
				UART0_Message.T_Index = 0;				
			}
		}
    if(UART0->IF & UART_IF_RcvOver){
		UART0->IF = UART_IF_RcvOver;
		hRec = UART_ReadData(UART0);
		gUartLastRxByte = hRec;
		gUartRxByteCount++;

		/* 空闲时只接受第一个帧头0xAA，收到后进入接收状态。 */
		if(UART0_Message.RecStatus == UART_STY){
			UART0_Message.R_Index = 0U;
			if(hRec != Head_RxH){
				gUartRxErrorCount++;
				return;
			}
			UART0_Message.RecStatus = UART_REC;
		}

		if(UART0_Message.RecStatus == UART_REC){
			/* 第二字节必须也是0xAA；若再次收到0xAA，则可直接作为新帧头重同步。 */
			if((UART0_Message.R_Index == 1U) && (hRec != Head_RxL)){
				UART0_Message.R_Index = 0U;
				UART0_Message.RecStatus = UART_STY;
				TickRecevice = 0U;
				gUartRxErrorCount++;
				return;
			}

			TickRecevice = 0U;
			if(UART0_Message.R_Index < UartMaxLen){
				UART0_Message.RXBuffer[UART0_Message.R_Index++] = hRec;
				/* 固定9字节帧收齐后立即交给主循环解析，不再等待帧间超时。 */
				if(UART0_Message.R_Index == UART_RX_FRAME_LEN){
					UART0_Message.RecStatus = UART_RECD;
				}
			}else{
				UART0_Message.R_Index = 0U;
				UART0_Message.RecStatus = UART_STY;
				gUartRxErrorCount++;
			}
		}
    }
    if(UART0->IF & UART_IF_SendBufEmpty)
    {
        UART0->IF = UART_IF_SendBufEmpty;
    }
}

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR *****END OF FILE****/

