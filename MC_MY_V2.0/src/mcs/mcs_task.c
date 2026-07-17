#include "main.h"
#include "mcs_motor_internal.h"

/*
 * 电机控制慢速任务调度器。
 * 1 ms 任务只处理启动状态和指令斜坡；电流采样、观测器更新、电流 PI 和 PWM
 * 生成仍放在 ADC 中断中执行。
 */

void Mcs_Task_Run(const TASK_TICK *tick)
{
    if(tick == NULL)
    {
        return;
    }

    if(tick->ms1 != 0U)
    {
        /* 为下一次 ADC/PWM 快速环准备控制指令。 */
        Motor_DirectionTest_Task();
    }
}
