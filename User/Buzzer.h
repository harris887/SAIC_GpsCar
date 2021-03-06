#ifndef _BUZZER_H_
#define _BUZZER_H_


#define RELAY_ON    1
#define RELAY_OFF   0

#define BUZZER_ON     1
#define BUZZER_OFF    0
#define BUZZER_A_PORT   GPIOC
#define BUZZER_A_PIN    GPIO_Pin_5

#define RELAY_PGV_PIN       GPIO_Pin_6  // J18
#define RELAY_SPOWER_PIN    GPIO_Pin_5  // J20
#define RELAY_1_PIN         GPIO_Pin_4  // J22 
#define RELAY_2_PIN         GPIO_Pin_3  // J24

#define RELAY_PORT          GPIOE 

typedef enum
{
  RELAY_CHARGE_P_Index=0,
  RELAY_CHARGE_N_Index,
  RELAY_PGV_POWER_Index,
  RELAY_SPOWER_Index
}RELAY_INDEX;

extern u16 Relay_status;

void BUZZER_Init(void);
void SetBuzzer(u8 on_off);

void BEEP_TASK(void);
void SetBeep(u16 BeepTimes,u16 BeepOnMs,u16 BeepDealy);
void SetBeepForever(u16 BeepOnMs,u16 BeepDealy);
void RELAY_Init(void);
void SetRelay(u8 num_0_1,u8 on_off);




#endif