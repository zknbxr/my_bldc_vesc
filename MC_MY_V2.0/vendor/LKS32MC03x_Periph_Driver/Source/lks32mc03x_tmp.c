/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： lks32mc03_tmp.c
 * 文件标识：
 * 内容摘要： TMP外设驱动程序
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： Howlet
 * 完成日期： 2015年11月5日
 *
 * 修改记录1：
 * 修改日期：2015年11月5日
 * 版 本 号：V 1.0
 * 修 改 人：Howlet
 * 修改内容：创建
 *
 *
 *******************************************************************************/
#include "lks32mc03x_tmp.h"
#include "lks32mc03x_lib.h"
#include "lks32mc03x_nvr.h"
extern unsigned int Read_Trim(unsigned int addr);
void TempSensor_Init(void);
Stru_TempertureCof_Def m_TempertureCof;     /* 温度传感器系数 */
/*******************************************************************************
 函数名称：    void TempSensor_Init(void)
 功能描述：    温度传感器初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/11/5      V1.0           Howlet Li          创建
 *******************************************************************************/
void TempSensor_Init(void)
{
    SYS_WR_PROTECT = 0x7a83;   /* 解除系统寄存器写保护 */
    SYS_AFE_REG0 |= BIT12;     /* 打开温度传感器开关 */
  
    m_TempertureCof.nCofA    = Read_Trim(0x000001C8);
    m_TempertureCof.nOffsetB = Read_Trim(0x000001CC);
}

/*******************************************************************************
 函数名称：    s16 GetCurrentTemperature(s16 ADC_value)
 功能描述：    得到当前温度值
 输入参数：    ADC_value: ADC通道14为温度传感器，ADC采样结果值
 输出参数：    无
 返 回 值：    t_Temperture：当前温度值，单位：1个Lsb代表0.1度
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/11/5      V1.0           Howlet Li          创建
 *******************************************************************************/
s16 GetCurrentTemperature(s16 ADC_value)
{
    s16 t_Temperture;  
	
    if(ADC->CFG & BIT10)
		{
        t_Temperture = (m_TempertureCof.nOffsetB - ((s32)m_TempertureCof.nCofA * ADC_value) / 1000);
		}
		else
		{
		    t_Temperture = (m_TempertureCof.nOffsetB - ((s32)m_TempertureCof.nCofA * (ADC_value >> 4)) / 1000);
		}

    return t_Temperture;
}
/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR *****END OF FILE****/
