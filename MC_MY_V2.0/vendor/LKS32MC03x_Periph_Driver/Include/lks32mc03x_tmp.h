/*******************************************************************************
 * 版权所有 (C)2018, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： lks32mc03x_tmp.h
 * 文件标识：
 * 内容摘要： ADC外设驱动程序头文件
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： William Zhang
 * 完成日期： 2018月25日
 *
 *
 * 修改记录：
 * 修改日期：
 * 版 本 号：
 * 修 改 人：
 * 修改内容：
 *
 *******************************************************************************/

#ifndef _LKS32MC03x_TMP_H_
#define _LKS32MC03x_TMP_H_


/* Includes ------------------------------------------------------------------*/
#include "lks32mc03x_lib.h"

typedef struct
{
   u16 nCofA;      /* 温度系数A */
   u16 nOffsetB;   /* 温度系数偏置 */
} Stru_TempertureCof_Def;


void TempSensor_Init(void);
s16 GetCurrentTemperature(s16 ADC_value);

#endif /*_CONNIE_TMP_H_ */



/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR *****END OF FILE****/
