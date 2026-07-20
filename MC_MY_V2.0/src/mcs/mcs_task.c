#include "main.h"
#include "mcs_motor_internal.h"

/*
 * 电机控制慢速任务调度器。
 * 1 ms 任务处理正式控制命令和电流斜坡；电流采样、观测器更新、电流 PI 和 PWM
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

        /* 母线电压等慢变量不占用 14 kHz ADC 中断时间。 */
        Motor_FocSlowUpdate1ms();


        /* 测试任务退出正式调度，保留源码仅供后续单独调试。 */
        // Motor_DirectionTest_Task();

        /* 正式控制先发布目标，再由独立斜坡模块生成 ADC 快速环使用的电流指令。 */
        Motor_ControlTask1ms(tick->ms1);
        Motor_CurrentCommandUpdate(&m_motor, tick->ms1);
        Hall_LearnTask1ms(tick->ms1);
    }
}
