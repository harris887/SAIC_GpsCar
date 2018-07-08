#include "user_inc.h"

//BeepTimes-连续响的次数
//BeepOnMs-一次鸣叫的时间，单位ms
//BeepDealy-两次次鸣叫的间隔时间，单位ms
u16 Beep_Times=0;
u16 Beep_OnMs=0;
u16 Beep_Dealy=0;
u8 Beep_Pro=0;
u16 Relay_status=0x0;//0

void BUZZER_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin =  BUZZER_A_PIN ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(BUZZER_A_PORT, &GPIO_InitStructure);  
  
  SetBuzzer(BUZZER_OFF);
}

void RELAY_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin =  RELAY_1_PIN|RELAY_2_PIN|RELAY_PGV_PIN|RELAY_SPOWER_PIN ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(RELAY_PORT, &GPIO_InitStructure); 
  
  if(Relay_status&0x02)
  {
    SetRelay(RELAY_CHARGE_P_Index,RELAY_ON);
  }
  else
  {
    SetRelay(RELAY_CHARGE_P_Index,RELAY_OFF);
  }
        
  if(Relay_status&0x01)
  {
    SetRelay(RELAY_CHARGE_N_Index,RELAY_ON);
  }
  else
  {
    SetRelay(RELAY_CHARGE_N_Index,RELAY_OFF); 
  }
  
  if(Relay_status&0x20)
  {
    SetRelay(RELAY_SPOWER_Index,RELAY_ON);
  }
  else
  {
    SetRelay(RELAY_SPOWER_Index,RELAY_OFF);
  }  

  if(Relay_status&0x10)
  {
    SetRelay(RELAY_PGV_POWER_Index,RELAY_ON);
  }
  else
  {
    SetRelay(RELAY_PGV_POWER_Index,RELAY_OFF);
  }      
}

void SetRelay(u8 num_0_1,u8 on_off)
{
    if(on_off)
    {
      switch(num_0_1)
      {
      case RELAY_CHARGE_P_Index:
        RELAY_PORT->BSRR = RELAY_1_PIN;
        break;
      case RELAY_CHARGE_N_Index:
        RELAY_PORT->BSRR = RELAY_2_PIN;
        break;
      case RELAY_SPOWER_Index:
        RELAY_PORT->BRR = RELAY_SPOWER_PIN;
        break;         
      case RELAY_PGV_POWER_Index:
        RELAY_PORT->BRR = RELAY_PGV_PIN;
        break;        
      }
    }
    else
    {
      switch(num_0_1)
      {
      case RELAY_CHARGE_P_Index:
        RELAY_PORT->BRR = RELAY_1_PIN;
        break;
      case RELAY_CHARGE_N_Index:
        RELAY_PORT->BRR = RELAY_2_PIN;
        break; 
      case RELAY_SPOWER_Index:
        RELAY_PORT->BSRR = RELAY_SPOWER_PIN;
        break;         
      case RELAY_PGV_POWER_Index:
        RELAY_PORT->BSRR = RELAY_PGV_PIN;
        break;        
      }
    }
}


void SetBuzzer(u8 on_off)
{
    if(on_off)
    {
      //BUZZER_A_PORT->BSRR = BUZZER_A_PIN;
      SET_DIDO_Relay(DIDO_Buzzer, 1);
    }
    else
    {
      BUZZER_A_PORT->BRR = BUZZER_A_PIN;  
      SET_DIDO_Relay(DIDO_Buzzer, 0);
    }
}


void SetBeep(u16 BeepTimes,u16 BeepOnMs,u16 BeepDealy)
{
  Beep_Times=(BeepTimes)?BeepTimes:1;//最少鸣叫一次
  Beep_OnMs=(BeepOnMs>50)?BeepOnMs:50;//最少鸣叫50ms
  Beep_Dealy=(BeepDealy>50)?BeepDealy:50;//最少间隔50ms
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
