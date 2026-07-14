/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： lks32mc03x_dac.c
 * 文件标识：
 * 内容摘要： DAC外设驱动程序
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： Zhu Jie
 * 完成日期： 2022年4月18日
 *
 *******************************************************************************/
#include "lks32mc03x_dac.h"
#include "string.h"
#include "lks32mc03x_sys.h"
#include "lks32mc03x_nvr.h"
DAC_CheckTypeDef Stru_DAC_CHECK;
/*******************************************************************************
 函数名称：    void DAC_StructInit(DAC_InitTypeDef* DAC_InitStruct)
 功能描述：    DAC结构体初始化
 操作的表：    无
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2022/04/18     V1.0           Zhu Jie              创建
 *******************************************************************************/
void DAC_StructInit(DAC_InitTypeDef* DAC_InitStruct)
{
    memset(DAC_InitStruct, 0, sizeof(DAC_InitTypeDef));
}
/*******************************************************************************
 函数名称：    void DAC_Init(DAC_InitTypeDef* DAC_InitStruct)
 功能描述：    DAC初始化
 操作的表：    无
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：    03xDAC分AB版本，量程可选 A只有3.0V；B版本有3.0和4.8V
               初始化配置时只需要选择量程即可，若为芯片为B版本，则4.8V量程有效；
               若芯片为A版本，则只能使用3.0V量程，4.8V配置无效
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2022/04/18      V1.0          Zhu Jie           创建
 -----------------------------------------------------------------------------
 2022/10/27      V2.0          Zhu Jie           修改
 *******************************************************************************/
void DAC_Init(DAC_InitTypeDef* DAC_InitStruct)
{
    SYS_AnalogModuleClockCmd(SYS_AnalogModule_DAC, ENABLE);  /* DAC 时钟使能 */
	
	  DAC_InitStruct->DAC_VERSION = REG32(0x40000004); /* 读取03芯片版本号 1 为A版本；2为B版本 */
	
	  SYS_WR_PROTECT = 0x7a83;  /* 解锁寄存器写保护 */
	
	  SYS_AFE_REG1 |= (DAC_InitStruct->DACOUT_EN << 1);/* DAC输出至IO口使能配置 */
	  
	  SYS_WR_PROTECT = 0xffff;  /* 锁定寄存器写保护 */
	
		if ((DAC_InitStruct->DAC_VERSION) == 1)/* 加载DAC 3.0V量程校正值 */
		{
			  Stru_DAC_CHECK.VersionAndDACGain = 1;
				Stru_DAC_CHECK.DAC_AMC = (Read_Trim(0x000001c0)&0xFFFF);
				Stru_DAC_CHECK.DAC_DC  = (Read_Trim(0x000001c4)&0xFFFF);
			  
		}
		else if (DAC_InitStruct->DAC_VERSION == 2)
		{
	      if(DAC_InitStruct->DAC_GAIN == DAC_RANGE_3V0)
				{
					  Stru_DAC_CHECK.VersionAndDACGain = 2;
				    Stru_DAC_CHECK.DAC_AMC = (Read_Trim(0x000001c0)&0xFFFF);
				    Stru_DAC_CHECK.DAC_DC  = (Read_Trim(0x000001c4)&0xFFFF);
				}
				else if(DAC_InitStruct->DAC_GAIN == DAC_RANGE_4V8)
				{
					  Stru_DAC_CHECK.VersionAndDACGain = 3;
				    Stru_DAC_CHECK.DAC_AMC = ((Read_Trim(0x000001c0)&0xFFFF0000) >> 16);
				    Stru_DAC_CHECK.DAC_DC  = ((Read_Trim(0x000001c4)&0xFFFF0000) >> 16);
				}
		}
}

/*******************************************************************************
 函数名称：    void DAC_OutputValue(uint32_t DACValue)
 功能描述：    DAC输出数字量数值设置
 操作的表：    无
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：    03xDAC量程分AB版，A版只有3.0V一个量程；B版有3.0V和4.8V两个量程可选 
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2022/04/18     V1.0           Zhu Jie              创建
 -----------------------------------------------------------------------------
 2022/10/27     V2.0           Zhu Jie              修改
 *******************************************************************************/
void DAC_OutputValue(uint32_t DACValue)
{
	  s16 temp1 = 0;
	
	  if(DACValue >=255) /* 限幅 */
		{
		    DACValue = 255;
		}
		
		temp1 = (s16)(DACValue * Stru_DAC_CHECK.DAC_AMC >> 9 )+ (s16)Stru_DAC_CHECK.DAC_DC;
		
		if(temp1 < 0)
		{
		  temp1 = 0;
		}
		else if(temp1 > 255)
		{
		  temp1 = 255;
		}	
	  SYS_AFE_DAC = (u32)temp1;
}

/*******************************************************************************
 函数名称：    void DAC_OutputVoltage(uint32_t DACVoltage)
 功能描述：    DAC输出模拟量数值设置
 操作的表：    无
 输入参数：    DACVoltage为Q12格式,范围0~4096对应DAC输出0~3V
 输出参数：    无
 返 回 值：    无
 其它说明：    03xDAC量程分AB版，A版只有3.0V一个量程；B版有3.0V和4.8V两个量程可选 
               与05x和08x不同，DAC数字量需要人为计算后再赋值给寄存器SYS_AFE_DAC
               y = A * x + B，即DAC输出电压 = DAC_Gain * x + DAC_Offset;
               校正值从地址中直接读取 分别对应0x000001c4 和 0x000001c0;
               函数Read_Trim();在nvr.o中。
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2022/04/18     V1.0           Zhu Jie              创建
 -----------------------------------------------------------------------------
 2022/10/28     V2.0           Zhu Jie              优化
 -----------------------------------------------------------------------------
2023/09/27     V2.1           LLYY                  优化: 兼容早期没有校准参数
 *******************************************************************************/
void DAC_OutputVoltage(uint16_t DACVoltage)
{
	  s32 temp = 0;
	  u32 range = 0;
	  s16 temp1 = 0; 
    
	  if(Stru_DAC_CHECK.VersionAndDACGain <= 2)
		{
		    range = (uint16_t)((1.0/3.0)*BIT8); /* BIT8 表示2^8 DAC为8位 */
		}
    else if(Stru_DAC_CHECK.VersionAndDACGain == 3)
		{
		    range = (uint16_t)((1.0/4.8)*BIT8); /* BIT8 表示2^8 DAC为8位 */
		}
	   temp = (DACVoltage * range ) >> 12; 
	
	  if(temp >=255) /* 限幅 */
		{
		    temp = 255;
		}
		
		temp1 = (s16)((temp * Stru_DAC_CHECK.DAC_AMC )>> 9) + (s16)Stru_DAC_CHECK.DAC_DC; ;
		
		if(temp1 < 0)
		{
		  temp1 = 0;
		}
		else if(temp1 > 255)
		{
		  temp1 = 255;
		}		
		
		if((Stru_DAC_CHECK.DAC_AMC ==0 )&& (Stru_DAC_CHECK.DAC_DC == 0))
		{
		   	SYS_AFE_DAC = temp;
		}
		else
		{
		  	SYS_AFE_DAC = temp1;
		}


	
}
/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR *****END OF FILE****/
