#include "user_inc.h"

//BeepTimes-������Ĵ���
//BeepOnMs-һ�����е�ʱ�䣬��λms
//BeepDealy-���δ����еļ��ʱ�䣬��λms
u16 Beep_Times=0;
u16 Beep_OnMs=0;
u16 Beep_Dealy=0;
u8 Beep_Pro=0;
u16 Relay_status=0;

void BUZZER_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin =  BUZZER_A_PIN ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(BUZZER_A_PORT, &GPIO_InitStructure);  
  
  SetBuzzer(BUZZER_OFF);
}

void SetBuzzer(u8 on_off)
{
    if(on_off)
    {
      BUZZER_A_PORT->BSRR = BUZZER_A_PIN;
    }
    else
    {
      BUZZER_A_PORT->BRR = BUZZER_A_PIN;  
    }
}


void SetBeep(u16 BeepTimes,u16 BeepOnMs,u16 BeepDealy)
{
  Beep_Times=(BeepTimes)?BeepTimes:1;//��������һ��
  Beep_OnMs=(BeepOnMs>50)?BeepOnMs:50;//��������50ms
  Beep_Dealy=(BeepDealy>50)?BeepDealy:50;//���ټ��50ms
  Beep_Pro=0;
}

void SetBeepForever(u16 BeepOnMs,u16 BeepDealy)
{
  if(Beep_Pro==0)
  {
    SetBeep(1,BeepOnMs,BeepDealy);
  }
}

void BEEP_TASK(void)
{
  static u32 NumOfSysTickIntBk;
  static u16 Counter;
  if(NumOfSysTickIntBk!=NumOfSysTickInt)
  {
    NumOfSysTickIntBk=NumOfSysTickInt;
  }
  else return;  
  
  if(Beep_Times==0) return;
  
  switch(Beep_Pro)
  {
  case 0:
    SetBuzzer(BUZZER_ON);
    Counter=0;
    Beep_Pro++;
    break;
  case 1:
    Counter++;
    if(Counter>=Beep_OnMs)
    {
      SetBuzzer(BUZZER_OFF);
      Counter=0;
      Beep_Pro++;
    }
    break;
  case 2:
    Counter++;
    if(Counter>=Beep_Dealy)
    {
      if(Beep_Times) Beep_Times-=1;
      Counter=0;
      Beep_Pro=0;
    }    
    break;
  default:
    {
      Beep_Pro=0;
      Beep_Times=0;
    }
  }
}
