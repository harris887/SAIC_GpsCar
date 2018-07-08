#if (0)
#include "user_inc.h"
#include "string.h"
#define LIGHT_SENSOR_PRINTF_DEBUG 0

#define LIGHT_SENSOR_NUM  4
LIGHT_SENSOR_OPTION LIGHT_SENSOR_Op[LIGHT_SENSOR_NUM];
u8 LIGHT_SENSOR_Flag=0;

void LightSensor_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin =  LIGHT_SENSOR_1_PIN|LIGHT_SENSOR_2_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
  GPIO_Init(LIGHT_SENSOR_12_PORT, &GPIO_InitStructure); 
  
  GPIO_InitStructure.GPIO_Pin =  LIGHT_SENSOR_3_PIN|LIGHT_SENSOR_4_PIN ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
  GPIO_Init(LIGHT_SENSOR_34_PORT, &GPIO_InitStructure);   
  
  memset(LIGHT_SENSOR_Op,0,sizeof(LIGHT_SENSOR_Op));
}

//0-无障碍，1-有障碍
u8 GetLightSensor(u8 index)
{
  u8 temp;
  switch(index)
  {
  case 0:
    {
      temp = ((LIGHT_SENSOR_12_PORT->IDR&LIGHT_SENSOR_1_PIN)?1:0);
    }
    break;
  case 1:
    {
      temp = ((LIGHT_SENSOR_12_PORT->IDR&LIGHT_SENSOR_2_PIN)?1:0);
    }
    break;    
  case 2:
    {
      temp = ((LIGHT_SENSOR_34_PORT->IDR&LIGHT_SENSOR_3_PIN)?1:0);
    }
    break;
  case 3:
    {
      temp = ((LIGHT_SENSOR_34_PORT->IDR&LIGHT_SENSOR_4_PIN)?1:0);
    }
    break;      
  default: temp = 0;
  }
  return temp;
}


void LIGHT_SENSOR_SCALE_TASK(void)
{
#define LIGHT_AVOID_SHARK_TIME  20
  static u32 NumOfSysTickIntBk;
  static u16 counter=0;
  u8 i,temp;
  if(NumOfSysTickIntBk!=NumOfSysTickInt)
  {
    NumOfSysTickIntBk=NumOfSysTickInt;
    counter++;
    if(counter>10)
    {
      counter=0;
    }
    else return;
  }
  else return;  

  for(i=0;i<LIGHT_SENSOR_NUM;i++)
  {
    temp=GetLightSensor(i);
    
    switch(LIGHT_SENSOR_Op[i].LightSensorPro)
    {
    case 0:
      if(temp!=0)
      {
        if(LIGHT_SENSOR_Op[i].HoldTime>=LIGHT_AVOID_SHARK_TIME) 
        {
          LIGHT_SENSOR_Op[i].LightSensorStatus=1;
          LIGHT_SENSOR_Op[i].LightSensorStatusChangFlag=1;
          LIGHT_SENSOR_Op[i].HoldTime=0;
          LIGHT_SENSOR_Op[i].LightSensorPro=1;
          
          LIGHT_SENSOR_Flag|=(1<<i);
          M_LightSensorStatus[i]=1;//1-有障碍物
#if (LIGHT_SENSOR_PRINTF_DEBUG)
          printf("Light %d face someting\n",i);
#endif
        }
        else
        {
          LIGHT_SENSOR_Op[i].HoldTime+=1;
        }
      }
      else
      {
        LIGHT_SENSOR_Op[i].HoldTime=0;
      }
      break;
    case 1:
      if(temp==0)
      {
        if(LIGHT_SENSOR_Op[i].HoldTime>=LIGHT_AVOID_SHARK_TIME) 
        {
          LIGHT_SENSOR_Op[i].LightSensorStatus=0;
          LIGHT_SENSOR_Op[i].LightSensorStatusChangFlag=1;
          LIGHT_SENSOR_Op[i].HoldTime=0;
          LIGHT_SENSOR_Op[i].LightSensorPro=0;
          
          LIGHT_SENSOR_Flag&=~(1<<i);
          M_LightSensorStatus[i]=0;//0-无障碍物
#if (LIGHT_SENSOR_PRINTF_DEBUG)          
          printf("Light %d face clear\n",i);
#endif
        }
        else
        {
          LIGHT_SENSOR_Op[i].HoldTime+=1;
        }
      }
      else
      {
        LIGHT_SENSOR_Op[i].HoldTime=0;
      }      
      break;
    default:
      memset(&LIGHT_SENSOR_Op[i].LightSensorPro,0,sizeof(LIGHT_SENSOR_OPTION));
    }
  }  
  
}
#endif