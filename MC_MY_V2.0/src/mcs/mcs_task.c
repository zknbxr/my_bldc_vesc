#include "main.h"
#include "mcs_motor_internal.h"

/* MCS layer jobs are grouped here from fast to slow periods. */

void Mcs_Task_Run(const TASK_TICK *tick)
{
    uint16_t i;

    if(tick == NULL)
    {
        return;
    }

}
