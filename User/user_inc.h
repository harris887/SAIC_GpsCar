#ifndef user_inc_h
#define user_inc_h

#include "stm32f10x_type.h"
#include "stm32f10x_lib.h"
#include "Usart.h"
#include "Delay.h"
#include "init.h"
#include "Adc.h"
#include "LedDisp.h"
 
#include "moto.h"

#include "Protocol.h"
#include "user_lib.h"
#include "ModBus.h"
#include "Program.h"
#include "SpeedLine.h"
#include "Pid.h"
#include "Remote.h"
#include "Buzzer.h"
#include "Button.h"
//#include "LightSensor.h"
#include "ModBus_UserInterface.h"
//#include "RTC_Time.h"
#include "Rfid.h"
#include "PGV.h"
#include "UltraSonic.h"
#include "DIDO.h"
//#include "ADS8431.h"
#include "moto_monitor.h"
#include "BMS.h"
#include "WK2124.h"


#define MODBUS_PARA_REFLUSH_FLASH_ENABLE  1//µ÷ÊÔ½×¶Î²»²ÁFLASH


#define HALL_SENSOR 0
#define PGV_SENSOR  1
#define FOLLOW_LINE_SENSOR PGV_SENSOR





  


#endif