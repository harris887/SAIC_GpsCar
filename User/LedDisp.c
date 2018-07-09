#include "user_inc.h"

#define EYE_Debug     0
u8 key_status=0;

void LedDispInit( void )
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin =  LED5_PIN | LED6_PIN ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //��©�����5V��������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LED5_PORT, &GPIO_InitStructure);   
  
  ClrLED5();
  ClrLED6();
}


void LED_DISPLAY_Reset(void)
{
  ClrLED5();
  ClrLED6();  
}

void LED_WATER_Display(u16 SPEED)
{
  static u32 NumOfSysTickIntBk;
  static u16 water_led_timer=0;
  static u16 water_led_status=0;
  if(NumOfSysTickInt!=NumOfSysTickIntBk)
  {
    NumOfSysTickIntBk=NumOfSysTickInt;
    if(water_led_timer) water_led_timer--;
  }
  else return;
  //��ˮ��
  switch(water_led_status)
  {
  case 0:
    {
      water_led_status=1;
      water_led_timer=SPEED;

      ClrLED5();
      ClrLED6();
    }
    break;
  case 1:
    if(water_led_timer==0)
    {
      water_led_status++;
      water_led_timer=SPEED;
      
      SetLED5();
      ClrLED6();      
    }
    break;   
  case 2:
    if(water_led_timer==0)
    {
      water_led_status++;
      water_led_timer=SPEED;

      ClrLED5();
      SetLED6();
    }
    break;
  case 3:
    if(water_led_timer==0)
    {
      water_led_status++;
      water_led_timer=SPEED;

      SetLED5();
      ClrLED6();
    }
    break;
  case 4:
    if(water_led_timer==0)
    {
      water_led_status=1;
      water_led_timer=SPEED;

      ClrLED5();
      SetLED6();      

    }
    break;      
  }
}


void LED_LOW_POWER_Display(u16 SPEED)
{
  static u32 NumOfSysTickIntBk;
  static u16 water_led_timer=0;
  static u16 water_led_status=0;
  if(NumOfSysTickInt!=NumOfSysTickIntBk)
  {
    NumOfSysTickIntBk=NumOfSysTickInt;
    if(water_led_timer)
    {
      water_led_timer--;
      return;
    }
  }
  else return;
  
  switch(water_led_status)
  {
  case 0:
    {
      water_led_status=1;
      water_led_timer=SPEED;
    }
    break;
  case 1:
    {
      water_led_status=0;
      water_led_timer=SPEED;

    }
    break;  
  default:
    water_led_status=0;
  }
}

/*��ͣ״̬�������˸*/
void LED_IM_STOP_Display(u16 SPEED)
{
  static u32 NumOfSysTickIntBk;
  static u16 water_led_timer=0;
  static u16 water_led_status=0;
  if(NumOfSysTickInt!=NumOfSysTickIntBk)
  {
    NumOfSysTickIntBk=NumOfSysTickInt;
    if(water_led_timer)
    {
      water_led_timer--;
      return;
    }
  }
  else return;
  
  switch(water_led_status)
  {
  case 0:
    {
      water_led_status=1;
      water_led_timer=SPEED;
      //SetLED1();
    }
    break;
  case 1:
    {
      water_led_status=0;
      water_led_timer=SPEED;
      //ClrLED1();
    }
    break;  
  default:
    water_led_status=0;
  }
}

/*Ѳ��״̬���̵���˸*/
void LED_FOLLOW_LINE_Display(u16 SPEED)
{
  static u32 NumOfSysTickIntBk;
  static u16 water_led_timer=0;
  static u16 water_led_status=0;
  if(NumOfSysTickInt!=NumOfSysTickIntBk)
  {
    NumOfSysTickIntBk=NumOfSysTickInt;
    if(water_led_timer)
    {
      water_led_timer--;
      return;
    }
  }
  else return;
  
  switch(water_led_status)
  {
  case 0:
    {
      water_led_status=1;
      water_led_timer=SPEED;
      //SetLED2();
    }
    break;
  case 1:
    {
      water_led_status=0;
      water_led_timer=SPEED;
      //ClrLED2();
    }
    break;  
  default:
    water_led_status=0;
  }
}

/*��ͣ״̬���Ƶ���˸*/
void LED_BARRIER_Display(u16 SPEED)
{
  static u32 NumOfSysTickIntBk;
  static u16 water_led_timer=0;
  static u16 water_led_status=0;
  if(NumOfSysTickInt!=NumOfSysTickIntBk)
  {
    NumOfSysTickIntBk=NumOfSysTickInt;
    if(water_led_timer)
    {
      water_led_timer--;
      return;
    }
  }
  else return;
  
  switch(water_led_status)
  {
  case 0:
    {
      water_led_status=1;
      water_led_timer=SPEED;

    }
    break;
  case 1:
    {
      water_led_status=0;
      water_led_timer=SPEED;

    }
    break;  
  default:
    water_led_status=0;
  }
}


/*RFID״̬��������˸*/
void LED_RFID_Display(u16 SPEED)
{
  static u32 NumOfSysTickIntBk;
  static u16 water_led_timer=0;
  static u16 water_led_status=0;
  if(NumOfSysTickInt!=NumOfSysTickIntBk)
  {
    NumOfSysTickIntBk=NumOfSysTickInt;
    if(water_led_timer)
    {
      water_led_timer--;
      return;
    }
  }
  else return;
  
  switch(water_led_status)
  {
  case 0:
    {
      water_led_status=1;
      water_led_timer=SPEED;

    }
    break;
  case 1:
    {
      water_led_status=0;
      water_led_timer=SPEED;

    }
    break;  
  default:
    water_led_status=0;
  }
}