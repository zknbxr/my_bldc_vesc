#include "main.h"

void Task_Scheduler(void);

volatile u8 gSensorlessStartedWatch;
volatile MCS_MOTOR_START_RESULT gSensorlessStartRetWatch;

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

