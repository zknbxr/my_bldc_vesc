#include "main.h"

/*
MCPWM_THx0 = -phase;
MCPWM_THx1 =  phase;
计数器到达THx0以前：
P 上管关闭
N 下管导通
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
    
    while(1)
    {
        Task_Scheduler();
    }
}

void Task_Scheduler(void)
{
    TASK_TICK tick;
}

