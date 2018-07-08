#ifndef __LEDDISP_H__
#define __LEDDISP_H__

#include "user_inc.h"


#define LED5_PIN      GPIO_Pin_4
#define LED5_PORT     GPIOB
#define LED6_PIN      GPIO_Pin_5
#define LED6_PORT     GPIOB

#define SetLED5()       (LED5_PORT->BRR=LED5_PIN)
#define ClrLED5()       (LED5_PORT->BSRR=LED5_PIN)
#define SetLED6()       (LED6_PORT->BRR=LED6_PIN)
#define ClrLED6()       (LED6_PORT->BSRR=LED6_PIN)

#define KEY_PIN      GPIO_Pin_8
#define KEY_PORT     GPIOA


extern void LedDispInit(void);
extern void LED_DISPLAY_Reset(void);
extern void LED_WATER_Display(u16 SPEED);
extern void LED_LOW_POWER_Display(u16 SPEED);
extern void LED_IM_STOP_Display(u16 SPEED);
extern void LED_FOLLOW_LINE_Display(u16 SPEED);
extern void LED_BARRIER_Display(u16 SPEED);
extern void LED_RFID_Display(u16 SPEED);
#endif