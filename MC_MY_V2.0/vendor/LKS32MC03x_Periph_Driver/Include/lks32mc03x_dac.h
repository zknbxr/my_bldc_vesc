/*******************************************************************************
 * 版权所有 (C)2021, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： lks32mc03x_dac.h
 * 文件标识：
 * 内容摘要： DAC外设驱动程序头文件
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： Zhujie
 * 完成日期： 2022年4月18日
 *
 *******************************************************************************/

#ifndef _LKS32MC03x_DAC_H_
#define _LKS32MC03x_DAC_H_

/* Includes ------------------------------------------------------------------*/
#include "lks32mc03x.h"
#include "basic.h"
typedef struct
{	
	  uint8_t DACOUT_EN;          /* DAC输出到IO使能：0，不使能；1，使能。 */
	  uint8_t DAC_VERSION;        /* 03芯片版本号，1表示A版本；2表示B版本。不用配置！！！ */
	  uint8_t DAC_GAIN;           /* DAC量程选择：0，3.0V；1，4.8V。注：03A版本只有3.0V一个量程，B版本有3.0V和4.8V两个 */                   
}DAC_InitTypeDef;

typedef struct
{	
	  uint8_t VersionAndDACGain; /* 芯片型号和DAC增益 1&2对应3.0V量程 3对应4.8V量程 */
	  int16_t DAC_AMC;           /* DAC增益校准值 */
	  int16_t DAC_DC;            /* DAC偏置校准值 */                   
}DAC_CheckTypeDef;

#define DAC_RANGE_3V0                  0                       /* DAC 3.0V量程 */
#define DAC_RANGE_4V8                  1                       /* DAC 4.8V量程 */

void DAC_StructInit(DAC_InitTypeDef* DAC_InitStruct);/* DAC结构体初始化 */
void DAC_Init(DAC_InitTypeDef* DAC_InitStruct);      /* DAC初始化 */
void DAC_OutputValue(uint32_t DACValue);             /* DAC输出数值设置--数字量 */
void DAC_OutputVoltage(uint16_t DACVoltage);         /* DAC输出模拟量数值设置--模拟量 */

#endif

