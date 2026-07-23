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


        /* 测试任务退出正式调度，保留源码仅供后续单独调试。 */
        // Motor_DirectionTest_Task();

        /* 故障任务拥有最高软件优先级，锁存后模式和控制任务都不会重新启动PWM。 */
        Motor_FaultTask1ms(tick->ms1);

        if(!Motor_FaultIsActive())
        {
            /* 模式状态机管理PWM，速度外环发布iq目标，最后由电流斜坡送入快速环。 */
            Motor_ControlTask1ms(tick->ms1);

            Motor_SpeedControlUpdate1ms(&m_motor, tick->ms1);

            Motor_CurrentCommandUpdate(&m_motor, tick->ms1);

            Hall_LearnTask1ms(tick->ms1);
        }
        
    }
    
    if(tick->ms10 != 0) {
        /* 母线电压等慢变量不占用 14 kHz ADC 中断时间。 */
        Motor_FocSlowUpdate1ms();
    }
}
