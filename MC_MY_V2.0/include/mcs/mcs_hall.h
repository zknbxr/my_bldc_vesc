#ifndef MCS_HALL_H
#define MCS_HALL_H

#include <stdint.h>

void HallControl_Init(void);
void HallControl_BlockStart(void);

void HallControl_RequestAlign(void);
void HallControl_AbortAlign(void);
void HallControl_ServiceAlignPulse(void);

void HallControl_RequestOffsetLearn(void);
void HallControl_AbortOffsetLearn(void);
void HallControl_ServiceOffsetLearn(void);
void HallControl_ServiceOffsetLearnAutoStart(void);

uint8_t HallControl_IsRunning(void);

#endif
