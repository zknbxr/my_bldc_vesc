#ifndef APP_HEIGHT_H
#define APP_HEIGHT_H

#include "stdint.h"

typedef struct
{
    /* Hall A/B中心点，由运行时min/max估算。 */
    int16_t centerA;
    int16_t centerB;
    /* Hall A/B当前记录到的原始ADC范围。 */
    int16_t minA;
    int16_t maxA;
    int16_t minB;
    int16_t maxB;
    /* 调试量：cumulativeAngle / 16384，约等于四分之一电周期计数。 */
    int32_t cumulativeEdge;
    /* 核心位置累计量：electricAngle差分积分，65536代表一个电周期。 */
    int32_t cumulativeAngle;
    /* APP层当前高度。 */
    int currentHeight;
    /* 当前电角度，0~65535代表一圈电角度。 */
    uint16_t electricAngle;
    /* min/max中心点校准是否完成。 */
    uint8_t calibrated;
    /* 最近一次A/B数字化状态，仅用于辅助观察。 */
    uint8_t lastState;
} APP_HEIGHT_STATUS;

/* Hall学习结果，供 Keil watch 直接观察。
 * 这些值保存在 RAM 中，掉电后丢失；后续接 Flash/NVM 时可从这里取值写入。
 */
extern volatile int16_t gHallLearnLatestA;
extern volatile int16_t gHallLearnLatestB;
extern volatile int16_t gHallLearnMinA;
extern volatile int16_t gHallLearnMaxA;
extern volatile int16_t gHallLearnMinB;
extern volatile int16_t gHallLearnMaxB;
extern volatile int16_t gHallLearnCenterA;
extern volatile int16_t gHallLearnCenterB;
extern volatile uint16_t gHallLearnRangeA;
extern volatile uint16_t gHallLearnRangeB;
extern volatile uint32_t gHallLearnSampleCount;
extern volatile uint16_t gHallLearnElectricAngle;
extern volatile int32_t gHallLearnCumulativeAngle;
extern volatile int32_t gHallLearnCumulativeEdge;
extern volatile uint8_t gHallLearnCalibrated;
extern volatile uint8_t gHallLearnValid;
extern volatile int32_t gHallNormX;
extern volatile int32_t gHallNormY;

/* 初始化高度观测模块，填充默认行程参数并清零累计量。 */
void AppHeight_Init(void);
/* ADC中断内调用，输入两路Hall ADC原始值。 */
void AppHeight_Sample(int16_t hallA, int16_t hallB);
/* 100ms任务调用，把累计电角度换算成gDeskMianMbr.currentHeight。 */
void AppHeight_Task100ms(void);
/* 清零累计量并重新开始min/max中心点校准。 */
void AppHeight_ClearCumulative(void);
/* 返回当前APP高度。 */
int AppHeight_GetCurrent(void);
/* 读取调试状态快照，供Keil watch或上报使用。 */
void AppHeight_GetStatus(APP_HEIGHT_STATUS *status);

#endif
