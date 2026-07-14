#include "main.h"



#define MCS_FOC_SPEED_REF_Q16       ((s32)(20L << 16))
#define MCS_FOC_SPEED_LOOP_DIV      (10U)
#define MCS_FOC_SPEED_IQ_LIMIT_MA   (180L)

static s32 Motor_LimitS32(s32 value, s32 min, s32 max)
{
    if(value > max)
    {
        return max;
    }

    if(value < min)
    {
        return min;
    }

    return value;
}



void StopMotorImmdly(void)
{
    /* Emergency-style stop used by legacy control code.
     * Keep it simple: disable PWM first, then mark the MCS motor sub-state
     * as BRAKE so APP/reporting code can observe that the drive is no longer
     * producing torque.
     */
    PwmAOutputs(DISABLE);
		
}

