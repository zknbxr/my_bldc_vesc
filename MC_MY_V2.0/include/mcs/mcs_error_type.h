#ifndef MCS_ERROR_TYPE_H
#define MCS_ERROR_TYPE_H

#include "basic.h"
/* Macro Definitions--------------------------------------------------------*/
#define HAlL_ERROR           				BIT0  /* ((u32)0x01)*/
#define SHORT_ERROR          				BIT1  /* ((u32)0x02)*/
#define LOW_VOL_ERROR        				BIT2  /* ((u32)0x04)*/
#define HIG_VOL_ERROR        				BIT3  /* ((u32)0x08)*/
#define OV_TEMPER_ERROR      				BIT4  /* ((u32)0x10)*/
#define MOSFET_ERROR         				BIT5  /* ((u32)0x20)*/
#define OVER_SPEED_ERR       				BIT6  /* ((u32)0x40)*/
#define OFFSET_ERROR         				BIT7  /* ((u32)0x80)*/
#define OVER_LOAD_ERROR      				BIT8  /* ((u32)0x0100)*/
#define HANDLEBAR_ERROR      				BIT9  /* ((u32)0x0200)*/
#define PHASE_LOSS_ERROR        			BIT10 /* ((u32)0x0400)*/
#define EMERGENCY_STOP_ERROR 				BIT11 /* ((u32)0x0800)*/
#define BRAKE_ERROR          				BIT12 /* ((u32)0x1000)*/
#define OVER_TIME_ERROR      			 	BIT13 /* ((u32)0x2000)*/
#define SVC_MOTOR_STALL_ERROR      			BIT14 /* ((u32)0x4000)*/


typedef enum
{
    E_FAULT_HAlL_ERROR						  = HAlL_ERROR,            //ªÙ∂˚π ’œ
    E_FAULT_SHORT_ERROR						  = SHORT_ERROR,           //∂Ã¬∑π ’œ
    E_FAULT_UNDER_VOL_ERROR		      = LOW_VOL_ERROR,         //«∑—ππ ’œ
    E_FAULT_OVER_VOL_ERROR		 			= HIG_VOL_ERROR,         //π˝—ππ ’œ
    E_FAULT_OV_TEMPER_ERROR		      = OV_TEMPER_ERROR,       //π˝Œ¬π ’œ
    E_FAULT_MOSFET_ERROR		 				= MOSFET_ERROR,          //MOSFETπ ’œ
    E_FAULT_OVER_SPEED_ERR	 				= OVER_SPEED_ERR,        //π˝ÀŸπ ’œ
    E_FAULT_OFFSET_ERROR	 			 		= OFFSET_ERROR,          //¡„∆Øπ ’œ
    E_FAULT_OVER_LOAD_ERROR			 		= OVER_LOAD_ERROR,       //π˝‘ÿπ ’œ
    E_FAULT_HANDLEBAR_ERROR			 		= HANDLEBAR_ERROR,       //◊™∞—π ’œ
    E_FAULT_PHASE_LOSS_ERROR			 	= PHASE_LOSS_ERROR,      //»±œ‡π ’œ
    E_FAULT_EMERGENCY_STOP_ERROR    = EMERGENCY_STOP_ERROR,  //ΩÙº±Õ£ª˙
    E_FAULT_BRAKE_ERROR			 		 		= BRAKE_ERROR,           //…≤≥µπ ’œ
    E_FAULT_OVER_TIME_ERROR     		= OVER_TIME_ERROR,       //≥¨ ±π ’œ
    E_FAULT_SVC_MOTOR_STALL_ERROR		= SVC_MOTOR_STALL_ERROR, //∂¬◊™π ’œ
} ENU_SysError;


#endif

