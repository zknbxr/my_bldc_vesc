#ifndef _HallAngle_H
#define _HallAngle_H

#include "basic.h"
#include "lks32mc03x.h"
#include "lks32mc03x_MCPWM.h"
#include "lks32mc03x_Gpio.h"


extern void HalltoAngle(s16 Hall_a_adc, s16 Hall_b_adc,u8 dir);
extern void Hall_Init(u8 mode);
extern void Hall_Stop(void);
extern INT32 RespondHALL_TOAPP_TokenWord(UINT16 uToken);
extern void Clear_Cumulative_Angle(void);

#endif
