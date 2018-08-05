/* Includes ------------------------------------------------------------------*/
#include "User\user_inc.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "User\moto.h"

char test_buffer[128];
extern u32 u2_rx_num;
int main(void)
{
  RCC_Configuration();
  NVIC_Configuration();
  GPIO_Configuration();
  //FLASH_READOUT_Protect();//harris 20160522
  BackupAccessEnable();
  
  MovementListInit();
  //BUTTON_Init();
  BUZZER_Init();

  LedDispInit();
  //LightSensor_Init();
  Adc_init();
  SysTick_Init(1);
  PGV_Timer_Init();
  REMOTE_Init();//遥控部分
  Usart1_Init();
  Usart2_Init();
  Usart3_Init();
  Usart4_Init();
  Usart5_Init();
  GetFlashModBusData(&MOD_BUS_Reg);
  MOTO_Init();
  WK2124_Init();
  
  printf("----- SAIC GPS Car ----\n");

  while(1)
  {
    UART_Task();
    LED_WATER_Display(500);
    
    Check_DIDO_TASK();
    BEEP_TASK();

    //PGV_COMM_TASK();
    MOTO_SPEED_CONTROL_TASK();
    
    CheckBatteryVolt_TASK();
    JOYSTICK_SCAN_TASK();
    CHECK_REMOTE_ENABLE_TASK();
    
    AGV_RUN_Task();
    MOD_BUS_REG_MODIFY_Check();
    WK2124_TransTask();
    
    TimeoutJump();
    FeedDog();    
    
    //MOTO_FaultCheck_TASK();
    //Moto_PowerFree_Task();//5s刹车后，电机驱动器复位    
    //CHECK_BUTTON_TASK();
    
    //if(0)
    if(debug_show)
    {
      debug_show=0;

      if(1)
      {
        static s16 speed = 0;
        static s8 o_index = 0;
        if(USART_BYTE == 'y')
        {
          USART_BYTE = 0;
          printf("DIDO %d Off\n", o_index);
          SET_DIDO_Relay(o_index--, 0);
          if(o_index < 0) o_index = 11;
          else if(o_index > 11) o_index = 0;
        }        
        if(USART_BYTE == 'Y')
        {
          USART_BYTE = 0;
          printf("DIDO %d On\n", o_index);
          SET_DIDO_Relay(o_index++, 1);
          if(o_index < 0) o_index = 11;
          else if(o_index > 11) o_index = 0;          
        }
        if(USART_BYTE == 'Z')
        {
          //USART_BYTE = 0;
          printf("DIDO ack_num : %d %d, DI = %02X, AI = [%d , %d , %d, %d] %d, %d\n", MODBUS_Dido_0.read_success_num, MODBUS_Dido_1.read_success_num, \
                 DIDO_D_INPUT_Status.LightStatus, \
                 DIDO_A_INPUT_Status.Analog[0],   \
                 DIDO_A_INPUT_Status.Analog[1],   \
                 DIDO_A_INPUT_Status.Analog[2],   \
                 DIDO_A_INPUT_Status.Analog[3],   \
                 wk2124_rx_bytes[CH_DAM0808], \
                 wk2124_rx_bytes[CH_DAM0404] \
                   );        
        }        
        
        if(USART_BYTE == '+')
        {
          USART_BYTE = 0;
          speed += 10;
          SetD1Rpm(LEFT_MOTO_INDEX, speed);
          SetD1Rpm(RIGHT_MOTO_INDEX, speed + 10);
          SetD1Rpm(LEFT_2_MOTO_INDEX, speed + 20);
          SetD1Rpm(RIGHT_2_MOTO_INDEX, speed + 30);
          sprintf(test_buffer,"rpm = %d\n", speed);
          FillUartTxBufN((u8*)test_buffer,strlen(test_buffer),1);          
        }
        if(USART_BYTE == '-')
        {
          USART_BYTE = 0;
          speed -= 10;
          SetD1Rpm(LEFT_MOTO_INDEX, speed);
          SetD1Rpm(RIGHT_MOTO_INDEX, speed + 10);
          SetD1Rpm(LEFT_2_MOTO_INDEX, speed + 20);
          SetD1Rpm(RIGHT_2_MOTO_INDEX, speed + 30);
          sprintf(test_buffer,"rpm = %d\n", speed);
          FillUartTxBufN((u8*)test_buffer,strlen(test_buffer),1);  
        }
      }      
      
      if(1)
      {
        if(USART_BYTE == 'D')
        {
          sprintf(test_buffer,"AD_Batt = %d, AD_An = %d, AD_Temp = %d, AD_eCurrent = %d\n",AD_Batt,AD_An,AD_Temp,AD_eCurrent);
          FillUartTxBufN((u8*)test_buffer,strlen(test_buffer),1);
        }
      }
      if(0)
        printf("w=%d,f=%f,\n",Wonder_Disp_or_Angle_value,Finish_Disp_or_Angle_value);
      if(1)
      {
        if(USART_BYTE == 'P')
        {
          sprintf(test_buffer,"AGV_RUN_Pro = %d \n", AGV_RUN_Pro);
          FillUartTxBufN((u8*)test_buffer,strlen(test_buffer),1);
        }
      }
      if(1)
      {
        if(USART_BYTE == 'Q')
        {
          sprintf(test_buffer,"Y = %d ,NL = %d; X = %d NP = %d\n",
               PGV_LANE_TACKING_Infor.YP,
               PGV_LANE_TACKING_Infor.Status_NL,                
               PGV_LANE_TACKING_Infor.XP,
               PGV_LANE_TACKING_Infor.Status_NP);
          FillUartTxBufN((u8*)test_buffer,strlen(test_buffer),1);
        }
        //前进方向，传感器不动，码带左边正最大60，右边负最小-60
      }
      if(1)
      {
        if(USART_BYTE == 'C')
        {
          sprintf(test_buffer,"batt=%d, Temp=%d, eCurr=%d \n", BatteryVolt_Filterd, Temp_Filterd, eCurrent_Filterd);
          FillUartTxBufN((u8*)test_buffer,strlen(test_buffer),1);
        }
        else if(USART_BYTE == 'T')
        {
          sprintf(test_buffer,"tempreture = %.2f , AD_Temp = %d \n", (float)Temperature_100times*0.01, AD_Temp);//3300.0/4096.0/10.0
          FillUartTxBufN((u8*)test_buffer,strlen(test_buffer),1);
        }
      }
      
      if(1)// 电机测试
      {
        if(USART_BYTE == 'A')
        {
          sprintf(test_buffer,"Speed_mmps: [ %d %d %d %d ], L_001rpm: [ %d %d %d %d ] \n",
                  MONITOR_St[LEFT_MOTO_INDEX].real_mms, MONITOR_St[RIGHT_MOTO_INDEX].real_mms,
                  MONITOR_St[LEFT_2_MOTO_INDEX].real_mms, MONITOR_St[RIGHT_2_MOTO_INDEX].real_mms,
                  MONITOR_St[LEFT_MOTO_INDEX].real_rpm_reg, MONITOR_St[RIGHT_MOTO_INDEX].real_rpm_reg,
                  MONITOR_St[LEFT_2_MOTO_INDEX].real_rpm_reg, MONITOR_St[RIGHT_2_MOTO_INDEX].real_rpm_reg);
          FillUartTxBufN((u8*)test_buffer,strlen(test_buffer),1);
        }
        else if(USART_BYTE == 'B')
        {
          sprintf(test_buffer,"RT: [ %d %d %d %d ] , RRT: [ %d %d %d %d ] , TOTAL: %d \n", // , %d
                  ReadMotoRpmTimes[LEFT_MOTO_INDEX] ,ReadMotoRpmTimes[RIGHT_MOTO_INDEX] ,
                  ReadMotoRpmTimes[LEFT_2_MOTO_INDEX] ,ReadMotoRpmTimes[RIGHT_2_MOTO_INDEX] ,
                  MONITOR_St[LEFT_MOTO_INDEX].counter ,MONITOR_St[RIGHT_MOTO_INDEX].counter , 
                  MONITOR_St[LEFT_2_MOTO_INDEX].counter ,MONITOR_St[RIGHT_2_MOTO_INDEX].counter ,
                  MODBUS_Monitor.read_success_num);//rx5
          FillUartTxBufN((u8*)test_buffer,strlen(test_buffer),1);
        }
      }
      if(1) //BMS
      {
        if(USART_BYTE == 'M')
        {
          /*
          sprintf(test_buffer,"vol: %.3fV, curr: %.3fA, Temp: %d, charge_num: %d, is_charge: %d, MOS: %d, refresh: %d, reset_num = %d\n",
                  (float)BMS_St.voltage_mv*0.001,
                  (float)BMS_St.curr_ma*0.001,
                  BMS_St.temprature,
                  BMS_St.in_ele_num,
                  (BMS_St.status&BMS_ST_BIT_IN_CHARGE_MASK)?1:0,
                  (BMS_St.status&(BMS_ST_MOS_IN_ELE_OPEN_MASK|BMS_ST_MOS_OUT_ELE_OPEN_MASK)),
                  BMS_St.ack_num,
                  BMS_St.reset_num);//rx5
          */
          FillUartTxBufN((u8*)test_buffer,strlen(test_buffer),1);
        }
      }
      if(1)
      {
        if(USART_BYTE == 'R')
        {
          sprintf(test_buffer,"up_downl: %d, left_right: %d, Speed_gain: %d\n",
                  REMOTE_OPTION_List[DIRECTION_FORWARD_BACKWARD_CHANNEL].pwm_step,
                  REMOTE_OPTION_List[DIRECTION_LEFT_RIGHT_CHANNEL].pwm_step, 
                  REMOTE_OPTION_List[SPEED_GAIN_CHANNEL].pwm_step);
          FillUartTxBufN((u8*)test_buffer,strlen(test_buffer),1);
        }    
      }
      if(1)
      {
        if(USART_BYTE == 'K')
        {
          sprintf(test_buffer,"IMU_Angle = %d\n", IMU_Angle);
          FillUartTxBufN((u8*)test_buffer, strlen(test_buffer), 1);
        }    
        
        if(USART_BYTE == 'E')
        {
          USART_BYTE = 0;
          MOD_BUS_Reg.M_CONTROL_MODE = M_CONTROL_MODE_SPEC;
          FillUartTxBufN("Reciprocate Mode\n", strlen("Reciprocate Mode\n"), 1);
        }
        
        if(USART_BYTE == 'F')
        {
          USART_BYTE = 0;
          RECIPROCATE_Op.wonder_reciprocate_times += 8;
          FillUartTxBufN("Reciprocate +4 \n", strlen("Reciprocate +4 \n"), 1);
        }        

        if(USART_BYTE == 'U')
        {
          USART_BYTE = 0;
          printf("Update Program!\n");   
          BKP_WriteBackupRegister(BKP_DR1, TO_BOOT_UPDATE_FLAG);
          SetTimeoutJump(200);
        }
      }      
    }
  }//end of while(1)
}











