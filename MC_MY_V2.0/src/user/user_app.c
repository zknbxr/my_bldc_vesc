#include "main.h"

/* User layer jobs are grouped here from always/fast to slow periods. */

static void User_App_DispatchUartCommand(void);

void User_App_Task_Run(const TASK_TICK *tick)
{
    if(tick == NULL)
    {
        return;
    }

    if(tick->ms1 != 0U)
    {
        
    }
}

static void User_App_DispatchUartCommand(void)
{

}
