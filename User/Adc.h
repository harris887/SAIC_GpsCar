#ifndef __Adc_h
#define __Adc_h

/******************************************************************************/
/*预处理部分*/
#include "user_inc.h"

/******************************************************************************/
/*宏定义*/
#define ADC1_DR_Address    ((u32)0x4001244C)
#define ADC_DMA_IRQChannel      DMA1_Channel1_IRQChannel


/******************************************************************************/

#define BATT_PIN    GPIO_Pin_3
#define BATT_PORT   GPIOC
#define AN_PIN      GPIO_Pin_0
#define AN_PORT     GPIOC
#define TEMP_SENSOR_PIN  GPIO_Pin_1
#define TEMP_SENSOR_PORT  GPIOC

#define CHARGE_PIN  GPIO_Pin_2
#define CHARGE_PORT  GPIOC

//---- LM35 相关 ----//
#define COFF_AD_2_TEMPRATURE          0.0806
#define COFF_AD_2_TEMPRATURE_100TIMES 8.06
#define TEMPERATURE_OFFSET_100TIMES   -1000  //-500

#define AD_BATT_OFFSET    69        //电源电压AD的补偿值

#define AD_Batt     adc_data[3] //((adc_data[0])+69)  //电池 -- (adc_data[0])+69
#define AD_An       adc_data[0]
#define AD_Temp     adc_data[1]  
#define AD_eCurrent adc_data[0]   //adc_data[2]

typedef enum
{
  BATTERY_AD_INDEX = 0,
  E_CURRENT_AD_INDEX,
  TEMP_AD_INDEX,
  
  AD_CHECK_CHANAL_NUM
}AD_CHECK_INDEX;

#define THRESHOLD_ROLLER_AD_FORWARD   512   //1.5/12
#define THRESHOLD_ROLLER_AD_BACKWARD  3584  //10.5/12

#define THRESHOLD_ROLLER_AD_FORWARD_DEAD   376   //1.1/12
#define THRESHOLD_ROLLER_AD_BACKWARD_DEAD  3720  //10.9/12

extern u16 adc_data[16];
extern u16 AD_CHECK_TimeOut;
extern u8 BatteryVolt_LowFlag;  //1-电池电压低，0-电池电压正常
extern u16 ChargeCurrentFixFlag;
extern s16 Temperature_100times;
extern u16 AD_FILTERED_List[AD_CHECK_CHANAL_NUM];
#define eCurrent_Filterd  AD_FILTERED_List[E_CURRENT_AD_INDEX]
#define Temp_Filterd      AD_FILTERED_List[TEMP_AD_INDEX]
#define BatteryVolt_Filterd AD_FILTERED_List[BATTERY_AD_INDEX]

extern void Adc_init( void );
extern void AD_DMA_IrqHandler(void);
extern void CheckBatteryVolt_TASK(void);


#endif

