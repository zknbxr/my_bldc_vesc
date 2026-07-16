#include "main.h"
#include "mcs_motor_internal.h"

/* MCS layer jobs are grouped here from fast to slow periods. */

void Mcs_Task_Run(const TASK_TICK *tick)
{
    if(tick == NULL)
    {
        return;
    }

    if(tick->ms1 != 0U)
    {
        Motor_DirectionTest_Task();
    }
}
