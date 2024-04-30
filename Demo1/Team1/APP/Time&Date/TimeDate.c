/********************************************************************************************************/
/************************************************Includes************************************************/
/********************************************************************************************************/
#include "TimeDate.h"
#include "LCD.h"
#include "LED.h"

/********************************************************************************************************/
/************************************************Defines*************************************************/
/********************************************************************************************************/


/********************************************************************************************************/
/************************************************Types***************************************************/
/********************************************************************************************************/
typedef struct
{
    u8 hr_dig1;
    u8 hr_dig2;
    u8 min_dig1;
    u8 min_dig2;
    u8 sec_dig1;
    u8 sec_dig2;
} Time_Value_t;

typedef struct
{
    u8 day_dig1;
    u8 day_dig2;
    u8 mon_dig1;
    u8 mon_dig2;
    u8 yr_dig1;
    u8 yr_dig2;
} Date_Value_t;

typedef enum
{
    hr_index1=0,
    hr_index2=1,
    min_index1=3,
    min_index2=4,
    sec_index1=6,
    sec_index2=7
} Time_Index_t;

typedef enum
{
    day_index1=8,
    day_index2=9,
    mon_index1=11,
    mon_index2=12,
    yr_index1=14,
    yr_index2=15
} Date_Index_t;


/********************************************************************************************************/
/************************************************Variables***********************************************/
/********************************************************************************************************/
 u8 X_Pos=0;
 u8 Y_Pos=0;

 u8 Sec=0;
 u8 Min=0;
 u8 Hr=0;
 u8 Yr=20;
 u8 Mon=0;
 u8 Day=0;

 Time_Value_t Time_Value={0,0,0,0,0,0};
 Date_Value_t Date_Value={0,0,0,0,2,0};

/********************************************************************************************************/
/*********************************************APIs Implementation****************************************/
/********************************************************************************************************/
/* call back function for testing */
 void ledon(void)
 {
 	LED_SetStatus(LED_RED, LED_STATE_ON);
 }

/**
 * @brief  : Shift the cursor one place to the right/left
 * @param  : Direction , can be :
 *           - Cursor_Left_Shift
 *           - Cursor_Right_Shift
 * @return : None
 */
void LCD_MOVE_DIRECTION(u8 Direction)
{
    switch(Direction)
    {
        case Cursor_Left_Shift:
        if(X_Pos==0 && Y_Pos==0)
        {
            X_Pos=1;
            Y_Pos=7;
        }
        else if(X_Pos==1 && Y_Pos==0)
        {
            X_Pos=0;
            Y_Pos=7;
        }
        else
        {
            Y_Pos--;
        }
        LCD_GotoPos_XY_async(X_Pos,Y_Pos,ledon);
        break;

        case Cursor_Right_Shift:
        if(X_Pos==0 && Y_Pos==7)
        {
            X_Pos=1;
            Y_Pos=0;
        }
        else if(X_Pos==1 && Y_Pos==7)
        {
            X_Pos=0;
            Y_Pos=0;
        }
        else
        {
            Y_Pos++;
        }
        LCD_GotoPos_XY_async(X_Pos,Y_Pos,ledon);
        break;
    }
}

/**
 * @brief  : Update the digit value (increment or decrement)
 * @param  : IncDec , can be :
 *           - Increment
 *           - Decrement
 * @return : None
 */
void LCD_INCDEC_TimeDate(IncDec_t IncDec)
{
   // calculate the index that the current x and y positions maps to
    u8 Index=(X_Pos==0)? Y_Pos:(X_Pos*8+Y_Pos);
    switch(Index)
    {
        //Inc or Dec Time Values
        // 05:55:56
        case sec_index1:
        if(IncDec==Increment)
        {
        	Time_Value.sec_dig1 = (Time_Value.sec_dig1 + 1) % 6;
        }
        else
        {
        	Time_Value.sec_dig1 = (Time_Value.sec_dig1 - 1 + 6) % 6;
        }
        break;

        case sec_index2:
        if(IncDec==Increment)
        {
        	Time_Value.sec_dig2 = (Time_Value.sec_dig2 + 1) % 10;

        }
        else
        {
        	Time_Value.sec_dig2 = (Time_Value.sec_dig2 - 1 + 10) % 10;
        }
        break;

        case min_index1:
        if(IncDec==Increment)
        {
        	Time_Value.min_dig1 = (Time_Value.min_dig1 + 1) % 6;
        }
        else
        {
        	Time_Value.min_dig1 = (Time_Value.min_dig1 - 1 + 6) % 6;
        }
        break;

        case min_index2:
        if(IncDec==Increment)
        {
        	Time_Value.min_dig2 = (Time_Value.min_dig2 + 1) % 10;
        }
        else
        {
        	Time_Value.min_dig2 = (Time_Value.min_dig2 - 1 + 10) % 10;
        }
        break;

        case hr_index1:
        if(IncDec==Increment)
        {
        	if(Time_Value.hr_dig2<4)
        	{
        		Time_Value.hr_dig1 = (Time_Value.hr_dig1 + 1) % 3;
        	}
        	else
        	{
        		Time_Value.hr_dig1 = (Time_Value.hr_dig1 + 1) % 2;
        	}
        }
        else
        {
        	Time_Value.hr_dig1 = (Time_Value.hr_dig1 -1 + 3) % 3;
        }
        break;

        case hr_index2:
        if(IncDec==Increment)
        {
        	if(Time_Value.hr_dig1==2)
        	{
        		Time_Value.hr_dig2 = (Time_Value.hr_dig2 + 1) % 4;
        	}
        	else
        	{
        		Time_Value.hr_dig2 = (Time_Value.hr_dig2 + 1) % 10;
        	}
        }
        else
        {
        	Time_Value.hr_dig2 = (Time_Value.hr_dig2 - 1 + 10) % 10;
        }
        break;

        // Inc or Dec Date Values
        case day_index1:
        if(IncDec==Increment)
        {
        	Date_Value.day_dig1 = (Date_Value.day_dig1 + 1) % 4;
        }
        else
        {
        	Date_Value.day_dig1 = (Date_Value.day_dig1 -1 +4 ) % 4;
        }
        break;

        case day_index2:
        if(IncDec==Increment)
        {
        	if(Date_Value.day_dig1<3)
        	{
        		Date_Value.day_dig2 = (Date_Value.day_dig2 + 1) % 10;
        	}
        	else if (Date_Value.day_dig1==3)
        	{
        		Date_Value.day_dig2 = (Date_Value.day_dig2 + 1) % 2;
        	}
        }
        else
        {
        	if(Date_Value.day_dig1<3)
        	{
        		Date_Value.day_dig2 = (Date_Value.day_dig2 -1 +10) % 10;
        	}
        	else if (Date_Value.day_dig1==3)
        	{
        		Date_Value.day_dig2 = (Date_Value.day_dig2 -1 +2 ) % 2;
        	}
        }
        break;

        case mon_index1:
        if(IncDec==Increment)
        {
        	Date_Value.mon_dig1 = (Date_Value.mon_dig1 + 1) % 2;
        }
        else
        {
        	Date_Value.mon_dig1 = (Date_Value.mon_dig1 - 1 + 2) % 2;
        }
        break;

        case mon_index2:
        if(IncDec==Increment)
        {
        	if(Date_Value.mon_dig1==0)
        	{
        		Date_Value.mon_dig2 = (Date_Value.mon_dig2 + 1) % 10;
        		if(Date_Value.mon_dig2==0)
        		{
        			Date_Value.mon_dig2=1;
        		}
        	}
        	else if (Date_Value.mon_dig1==1)
        	{
        		Date_Value.mon_dig2 = (Date_Value.mon_dig2 + 1) % 3;
        	}
        }
        else
        {
        	if(Date_Value.mon_dig1==0)
        	{
        		Date_Value.mon_dig2 = (Date_Value.mon_dig2 - 1 +10) % 10;
        		if(Date_Value.mon_dig2==0)
        		{
        			Date_Value.mon_dig2=1;
        		}
        	}
        	else if (Date_Value.mon_dig1==1)
        	{
        		Date_Value.mon_dig2 = (Date_Value.mon_dig2 -1 +3) % 3;
        	}
        }
        break;

        case yr_index1:
        if(IncDec==Increment)
        {
        	if(Date_Value.yr_dig1==2)
        	{
        		Date_Value.yr_dig1=3;
        	}
        }
        else
        {
          if (Date_Value.yr_dig1==3)
          {
        	  Date_Value.yr_dig1=2;
          }
        }
        break;

        case yr_index2:
        if(IncDec==Increment)
        {
        	if(Date_Value.yr_dig1==2)
        	{
        		Date_Value.yr_dig2=(Date_Value.yr_dig2 +1 ) % 10;
        	}
        	else if(Date_Value.yr_dig1==3)
        	{
        		Date_Value.yr_dig2=0;
        	}
        }
        else
        {
        	if(Date_Value.yr_dig1==2)
        	{
        		Date_Value.yr_dig2=(Date_Value.yr_dig2 - 1 + 10) % 10;
        	}
        	else if(Date_Value.yr_dig1==3)
        	{
        		Date_Value.yr_dig2=0;
        	}
        }
        break;

    }
    // Update time values from Time_Value struct members
     Sec = (Time_Value.sec_dig1 * 10) + Time_Value.sec_dig2;
     Min = (Time_Value.min_dig1 * 10) + Time_Value.min_dig2;
     Hr = (Time_Value.hr_dig1 * 10) + Time_Value.hr_dig2;
     // Update date values from Date_Value struct members
     Yr = (Date_Value.yr_dig1 * 10) + Date_Value.yr_dig2;
     Mon = (Date_Value.mon_dig1 * 10) + Date_Value.mon_dig2;
     Day = (Date_Value.day_dig1 * 10) + Date_Value.day_dig2;

}


void TimeDateDisplay(void)
{

     // Increment seconds and handle rollover
    Sec = (Sec + 1) % 60;
    if (Sec == 59)
    {
        // Increment minutes and handle rollover
        Min = (Min + 1) % 60;
        if (Min == 59)
        {
            // Increment hours and handle rollover
            Hr = (Hr + 1) % 24;
        }
    }

    // Display time on LCD
    LCD_GotoPos_XY_async(0, 0, NULLPTR);
    if(Hr==0)
    {
    	LCD_enuWriteString_asynch("0", NULLPTR);
    	LCD_enuWriteString_asynch("0", NULLPTR);
    	LCD_enuWriteString_asynch(":", NULLPTR);
    }
    else if(Hr<10)
    {
		LCD_enuWriteString_asynch("0", NULLPTR);
		LCD_enuWriteNumber_asynch(Hr, NULLPTR);
		LCD_enuWriteString_asynch(":", NULLPTR);
    }
    else
    {
        LCD_enuWriteNumber_asynch(Hr, NULLPTR);
        LCD_enuWriteString_asynch(":", NULLPTR);
    }
    if(Min==0)
    {
    	LCD_enuWriteString_asynch("0", NULLPTR);
    	LCD_enuWriteString_asynch("0", NULLPTR);
    	LCD_enuWriteString_asynch(":", NULLPTR);
    }
    else if(Min<10)
    {
		LCD_enuWriteString_asynch("0", NULLPTR);
		LCD_enuWriteNumber_asynch(Min, NULLPTR);
		LCD_enuWriteString_asynch(":", NULLPTR);
    }
    else
    {
        LCD_enuWriteNumber_asynch(Min, NULLPTR);
        LCD_enuWriteString_asynch(":", NULLPTR);
    }
    if(Sec==0)
    {
    	LCD_enuWriteString_asynch("0", NULLPTR);
    	LCD_enuWriteString_asynch("0", NULLPTR);
    }
    else if(Sec<10)
    {
		LCD_enuWriteString_asynch("0", NULLPTR);
		LCD_enuWriteNumber_asynch(Sec, NULLPTR);
    }
    else
    {
        LCD_enuWriteNumber_asynch(Sec, NULLPTR);
    }

    LCD_GotoPos_XY_async(1,0,NULLPTR);
      if(Day==0)
     {
     	LCD_enuWriteString_asynch("0", NULLPTR);
     	LCD_enuWriteString_asynch("0", NULLPTR);
     	LCD_enuWriteString_asynch("/", NULLPTR);
     }
     else if(Day<10)
     {
 		LCD_enuWriteString_asynch("0", NULLPTR);
 		LCD_enuWriteNumber_asynch(Day, NULLPTR);
 		LCD_enuWriteString_asynch("/", NULLPTR);
     }
     else
     {
         LCD_enuWriteNumber_asynch(Day, NULLPTR);
         LCD_enuWriteString_asynch("/", NULLPTR);
     }
     if(Mon==0)
     {
     	LCD_enuWriteString_asynch("0", NULLPTR);
     	LCD_enuWriteString_asynch("0", NULLPTR);
     	LCD_enuWriteString_asynch("/", NULLPTR);
     }
     else if(Mon<10)
     {
 		LCD_enuWriteString_asynch("0", NULLPTR);
 		LCD_enuWriteNumber_asynch(Mon, NULLPTR);
 		LCD_enuWriteString_asynch("/", NULLPTR);
     }
     else
     {
         LCD_enuWriteNumber_asynch(Mon, NULLPTR);
         LCD_enuWriteString_asynch("/", NULLPTR);
     }
     if(Yr==0)
     {
     	LCD_enuWriteString_asynch("0", NULLPTR);
     	LCD_enuWriteString_asynch("0", NULLPTR);
     }
     else if(Yr<10)
     {
 		LCD_enuWriteString_asynch("0", NULLPTR);
 		LCD_enuWriteNumber_asynch(Yr, NULLPTR);
     }
     else
     {
         LCD_enuWriteNumber_asynch(Yr, NULLPTR);
     }
 }

