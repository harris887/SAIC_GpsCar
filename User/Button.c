#include "user_inc.h"
#include "string.h"

#define BUTTON_PRINTF_DEBUG 0

typedef struct
{
  u8 ButtonPro;
  u8 ButtonStatus;//0-抬起，1-按下
  u8 HoldTime;
  u8 ButtonStatusChangFlag;
}BUTTON_OPTION;

#define BUTTON_NUM  3
BUTTON_OPTION BUTTON_Op[BUTTON_NUM];

u8 BUTTON_IM_STOP_Flag = 0;
u8 BUTTON_FOLLOW_LINE_AND_PROGRAM_Flag = 0;
u8 BARRIER_Flag = 0;

void BUTTON_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin =  BUTTON_IMM_STOP_PIN ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
  GPIO_Init(BUTTON_IMM_STOP_PORT, &GPIO_InitStructure); 
  
  GPIO_InitStructure.GPIO_Pin =  BUTTON_TOUCH_F_PIN | BUTTON_TOUCH_F_PIN ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
  GPIO_Init(BUTTON_TOUCH_PORT, &GPIO_InitStructure);   
  
  memset(BUTTON_Op,0,sizeof(BUTTON_Op));
  BUTTON_IM_STOP_Flag=0;
  BUTTON_FOLLOW_LINE_AND_PROGRAM_Flag=0;  
}



void CHECK_BUTTON_TASK(void)
{
#define AVOID_SHARK_TIME  5   //5*50ms防抖时间
  //平时为高，按下为低
  static u32 NumOfSysTickIntBk;
  static u16 counter=0;
  u8 button_down_flag;
  u8 temp,i;
  if(NumOfSysTickIntBk!=NumOfSysTickInt)
  {
    NumOfSysTickIntBk=NumOfSysTickInt;
    counter++;
    if(counter>50)
    {
      counter=0;
    }
    else return;
  }
  else return;
  

  for(i=0;i<BUTTON_NUM;i++)
  {
    if(i==0) button_down_flag=(GET_BUTTON_TOUCH_F_STATUS())?0:1;//低电平时表示按键按下
    else if(i==1) button_down_flag=(GET_BUTTON_IMM_STOP_STATUS())?0:1;//低电平时表示按键按下
    else button_down_flag=(GET_BUTTON_TOUCH_B_STATUS())?0:1;
    
    switch(BUTTON_Op[i].ButtonPro)
    {
    case 0:
      if(button_down_flag!=0)
      {
        if(BUTTON_Op[i].HoldTime>=AVOID_SHARK_TIME) 
        {
          BUTTON_Op[i].ButtonStatus=1;
          BUTTON_Op[i].ButtonStatusChangFlag=1;
          BUTTON_Op[i].HoldTime=0;
          BUTTON_Op[i].ButtonPro=1;
          //SetBeep(2,200,500);
#if (BUTTON_PRINTF_DEBUG)
          printf("%d ON \n",i);
#endif
        }
        else
        {
          BUTTON_Op[i].HoldTime+=1;
        }
      }
      else
      {
        BUTTON_Op[i].HoldTime=0;
      }
      break;
    case 1:
      if(button_down_flag==0)
      {
        if(BUTTON_Op[i].HoldTime>=AVOID_SHARK_TIME) 
        {
          BUTTON_Op[i].ButtonStatus=0;
          BUTTON_Op[i].ButtonStatusChangFlag=1;
          BUTTON_Op[i].HoldTime=0;
          BUTTON_Op[i].ButtonPro=0;
          //SetBeep(1,500,500);
#if (BUTTON_PRINTF_DEBUG)
          printf("%d OFF \n",i);
#endif
        }
        else
        {
          BUTTON_Op[i].HoldTime+=1;
        }
      }
      else
      {
        BUTTON_Op[i].HoldTime=0;
      }      
      break;
    }
  }
  //BUTTON_FOLLOW_LINE_AND_PROGRAM_Flag=!BUTTON_Op[0].ButtonStatus; 
  BUTTON_IM_STOP_Flag=BUTTON_Op[1].ButtonStatus;//!
  //触碰开关作为第5个广电开关
  if(BUTTON_Op[0].ButtonStatus)
  {
    //LIGHT_SENSOR_Flag|=(1<<0);
    M_LightSensorStatus[0]=1;
  }
  else
  {
    //LIGHT_SENSOR_Flag&=~(1<<0);
    M_LightSensorStatus[0]=0;
  }
  
  if(BUTTON_Op[2].ButtonStatus)
  {
    //LIGHT_SENSOR_Flag|=(1<<1);
    M_LightSensorStatus[1]=1;
  }
  else
  {
    //LIGHT_SENSOR_Flag&=~(1<<1);
    M_LightSensorStatus[1]=0;
  }  
  
}

