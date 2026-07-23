#include "main.h"

/*
MCPWM_THx0 = -phase;
MCPWM_THx1 =  phase;
计数器到达THx0以前：
P 上管关闭
N 下管导通
*/

/*
 * 运行命令观察 gMotorCommand，权威软件状态观察 m_motor.m_run_state。
 * PWM 是否实际输出由 Motor_IsPwmEnabled() 判断，转子运动由 m_pll_speed 判断。
 */
void Task_Scheduler(void);

int main(void)
{
    /* 硬件配置初始化。 */
    Hardware_init();

    /* 电机控制系统初始化。 */
    mc_sys_init();

    /* 客户应用初始化，未涉及硬件初始化。 */
    User_app_init();
    /* 测试任务不再参与正式启动。 */
    // Direction_Init();
    Motor_ControlInit();
    while(1)
    {
        Task_Scheduler();
    }
}

void Task_Scheduler(void)
{
    static u16 s_pwmTicksTo1ms = 0U;
    static u16 s_msTo10ms = 0U;
    static u16 s_msTo100ms = 0U;
    static u16 s_msTo1000ms = 0U;
    TASK_TICK tick = {0U, 0U, 0U, 0U};
    u16 pendingPwmTicks;

    /* TickCounter is written by the ADC ISR, so take and clear it atomically. */
    __disable_irq();
    pendingPwmTicks = TickCounter;
    TickCounter = 0U;
    __enable_irq();

    while(pendingPwmTicks > 0U)
    {
        pendingPwmTicks--;
        s_pwmTicksTo1ms++;

        if(s_pwmTicksTo1ms >= PWM_TIME_1MS_COUNTER)
        {
            s_pwmTicksTo1ms -= PWM_TIME_1MS_COUNTER;
            tick.ms1++;

            s_msTo10ms++;
            s_msTo100ms++;
            s_msTo1000ms++;

            if(s_msTo10ms >= 10U)
            {
                s_msTo10ms -= 10U;
                tick.ms10++;
            }

            if(s_msTo100ms >= 100U)
            {
                s_msTo100ms -= 100U;
                tick.ms100++;
            }

            if(s_msTo1000ms >= 1000U)
            {
                s_msTo1000ms -= 1000U;
                tick.ms1000++;
            }
        }
    }
    
    User_Task_Always();
    if((tick.ms1 != 0U) || (tick.ms10 != 0U) ||
       (tick.ms100 != 0U) || (tick.ms1000 != 0U))
    {
        Mcs_Task_Run(&tick);
        User_App_Task_Run(&tick);
    }
}
