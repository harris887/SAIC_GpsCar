#if(0)
#ifndef _light_sensor_h_
#define _light_sensor_h_

#define LIGHT_SENSOR_MASK 0x03

#define LIGHT_SENSOR_1_PIN  GPIO_Pin_1
#define LIGHT_SENSOR_2_PIN  GPIO_Pin_0
#define LIGHT_SENSOR_12_PORT   GPIOE

#define LIGHT_SENSOR_3_PIN  GPIO_Pin_9
#define LIGHT_SENSOR_4_PIN  GPIO_Pin_8
#define LIGHT_SENSOR_34_PORT   GPIOB


typedef struct
{
  u8 LightSensorPro;
  u8 LightSensorStatus;//0-Œﬁ’œ∞≠£¨1-”–’œ∞≠
  u8 HoldTime;
  u8 LightSensorStatusChangFlag;
}LIGHT_SENSOR_OPTION;

extern u8 LIGHT_SENSOR_Flag;

void LightSensor_Init();
void LIGHT_SENSOR_SCALE_TASK(void);
#endif
#endif