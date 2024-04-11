#include "Stopwatch.h"

StopWatch_value_t StopWatch_value;
/************************** setters ****************************/
/* reset function when reset button pressed */
StopWatch_ErrorState_t StopWatch_Reset(void){
    StopWatch_ErrorState_t Loc_errorState = StopWatch_OK;
    StopWatch_value.millisec = 0;
    StopWatch_value.sec = 0;
    StopWatch_value.min = 0;
    StopWatch_value.hour = 0;
    return Loc_errorState;
}

StopWatch_ErrorState_t StopWatch_Count(StopWatch_value_t* StopWatch_values){
    StopWatch_ErrorState_t Loc_errorState = StopWatch_OK;
    if(StopWatch_value.millisec < 9)
    {
        StopWatch_value.millisec++;
    }
    else
    {
        StopWatch_value.sec++;
        StopWatch_value.millisec = 0;
        if(StopWatch_value.sec >59)
        {
            StopWatch_value.min++;
            StopWatch_value.sec = 0;
            if(StopWatch_value.min > 59)
            {
                StopWatch_value.hour++;
                StopWatch_value.min = 0;
                if(StopWatch_value.hour > 23)
                {
                    StopWatch_value.hour = 0;
                    Loc_errorState = StopWatch_TimeExceeded;
                }
            }
        }
    }
    StopWatch_values->millisec = StopWatch_value.millisec;
    StopWatch_values->sec = StopWatch_value.sec;
    StopWatch_values->min = StopWatch_value.min;
    StopWatch_values->hour = StopWatch_value.hour;
    return Loc_errorState;
}

void StopWatch_Pause(void){
    /* DO NOTHING */
} 
/************************** getters *****************************/


