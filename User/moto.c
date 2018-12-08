#include "user_inc.h"
#include "string.h"
#include "math.h"
#include "stdlib.h"

#define MOTO_PRINTF_DEBUG               1


#define MOTO_COMM_PORT_ENUM             5   // 使用 USART5 发送 
#define MOTO_CONTROL_CYCLE              12  // 20ms 的RS485通信周期 - 10 , 12 ,100
#define DEFAULT_MOTO_READ_RPM_TIME_OUT  100
#define DEFAULT_MOTO_CHECK_MAX_CURRENT_TIMEOUT 50 
#define CHECK_MAX_CURRENT_HOLD_TIME            500

u16 MOTO_485COMM_Timeout = 2000;
u16 MOTO_CheckMaxCurrent_Timeout = 2000;
s16 moto_speed_in_rpm[MOTO_NUM] = {0 ,0 ,0 ,0}; 
u8 moto_enable_status[MOTO_NUM] = {0 ,0 ,0 ,0};
u8 moto_enable_status_change_flag[MOTO_NUM] = {0 ,0 ,0 ,0};
u16 MOTO_READ_RPM_Timeout[MOTO_NUM] = {DEFAULT_MOTO_READ_RPM_TIME_OUT ,DEFAULT_MOTO_READ_RPM_TIME_OUT ,DEFAULT_MOTO_READ_RPM_TIME_OUT ,DEFAULT_MOTO_READ_RPM_TIME_OUT};
u8 moto_reset_speed_up_down_time_flag[MOTO_NUM] = {0 ,0 ,0 ,0};
u8 moto_reset_max_current_flag[MOTO_NUM] = {0 ,0 ,0 ,0};
u16 Moto_MaxCurrent[MOTO_NUM] = {MAX_MOTO_EA_THRESHOLD_0, MAX_MOTO_EA_THRESHOLD_0, MAX_MOTO_EA_THRESHOLD_0, MAX_MOTO_EA_THRESHOLD_0};
#define MOTO_SPEED_UP_DOWN_DELAY_TIME     20  // 100 -> 20
const u8 MODBUS_MOTO_ENBALE[8] =
{0x01 ,0x06 ,0x00 ,0x38 ,0x00 ,0x01 ,0xC9 ,0xC7};
const u8 MODBUS_MOTO_RPM_SET[8] = 
{0x01 ,0x06 ,0x00 ,0x33 ,0x00 ,0x64 ,0x78 ,0x2E};
const u8 MODBUS_MOTO_STOP_SET[8] = 
{0x01 ,0x06 ,0x00 ,0x1F ,0x00 ,0x00 ,0xB8 ,0x0C};
//{0x01 ,0x06 ,0x00 ,0x1F ,0x00 ,0x01 ,0x79 ,0xCCC};
const u8 MODBUS_MOTO_UP_TIME_SET[8] = 
{0x01 ,0x06 ,0x00 ,0x30 ,0x00 ,0x64 ,0x00 ,0x00};
const u8 MODBUS_MOTO_DOWN_TIME_SET[8] = 
{0x01 ,0x06 ,0x00 ,0x31 ,0x00 ,0x64 ,0x00 ,0x00};
const u8 MODBUS_MOTO_RPM_READ[8] = 
//{0x01 ,0x03 ,0x10 ,0x2D ,0x00 ,0x01 ,0x85, 0xF6}; // 单位：0.01RPM
//{0x01 ,0x03 ,0x10 ,0x00 ,0x00 ,0x01 ,0x85, 0xF6};   // 单位：RPM
{0x01 ,0x03 ,0x10 ,0x00 ,0x00 ,0x03 ,0x01, 0x0B};   // DN_00 ~ DN_02
//{0x01 ,0x03 ,0x10 ,0x00 ,0x00 ,0x1D ,0x81, 0x03};   // DN_00 ~ DN_1C
//{0x01 ,0x03 ,0x10 ,0x1C ,0x00 ,0x01 ,0x41, 0x0C};   // DN_1C
const u8 MODBUS_MOTO_MAX_EA_SET[8] =
{0x01 ,0x06 ,0x00 ,0x1A ,0x13 ,0x88 ,0xA5 ,0x5B};

u8 moto_comm_buff[256];
u32 ReadMotoRpmTimes[MOTO_NUM] = {0 ,0 ,0 ,0};
u32 SetMotoRpmTimes[MOTO_NUM] = {0 ,0 ,0 ,0};

#if (FOLLOW_LINE_SENSOR == HALL_SENSOR) 
extern s32 SPEED_PID(u8 expect_mid,u8 current_mid);
#else
extern s32 SPEED_PID(s32 expect_offset,s32 current_offset);
#endif
//位移里程计，单位: mm
float RoadLength[MOTO_NUM] = {0.0 ,0.0, 0.0 ,0.0};
s16 RealRpm[MOTO_NUM] = {0 ,0 ,0 ,0};



float max_wheel_speed;//车轮最大速度，单位cm/S


SPEED_OPTION_LIST SPEED_OPTION_List={0,0};

/*******************************************************
 ** 功能：根据车轮参数计算轮子/车体的额定速度,单位:cm/s
 ** 输入：
 **       WHEEL_DIAMETER_IN_CM: 轮子直径(单位:cm)
 **       MAX_MOTO_SPEED_IN_RPM: 设定的电机的最高(100%)转速(单位:rpm)
 **       SPEED_DOWN_RATIO: 齿轮箱减速比
 ** 返回：车轮最大速度，单位cm/s
 ******************************************************/
float Caculate_MAX_WHEEL_SPEED(float wheel_diameter_in_cm,float max_moto_speed_in_rpm,float speed_down_ratio)
{
  return (wheel_diameter_in_cm*PI)*(max_moto_speed_in_rpm/speed_down_ratio/60.0);
}

void MOTO_Init(void)
{
  max_wheel_speed = Caculate_MAX_WHEEL_SPEED(WHEEL_DIAMETER_IN_CM, MAX_MOTO_SPEED_IN_RPM, SPEED_DOWN_RATIO);
  SPEED_UP_DOWN_STRUCT_Init(50,100,0.1,SPEED_UP_OPTION_List[DirectRun]);//加速度20cm/S^2,最高速度40cm/S,加速周期0.1s ,40
  SPEED_UP_DOWN_STRUCT_Init(5 ,10,0.1,SPEED_UP_OPTION_List[CircleRun]);
  COFF_001RPM_TO_MMS = WHEEL_DIAMETER_IN_CM * 10.0 * PI * 0.01 / 60.0 / SPEED_DOWN_RATIO;
  COFF_RPM_TO_MMS = WHEEL_DIAMETER_IN_CM * 10.0 * PI / 60.0 / SPEED_DOWN_RATIO;
  //COFF_MMS_TO_RPM = 60.0 / (WHEEL_DIAMETER_IN_CM * 10.0* PI) ;
  COFF_MMS_TO_D1RPM = SPEED_DOWN_RATIO * 60.0 / (WHEEL_DIAMETER_IN_CM * 10.0 * PI ) ; //* 0.1
  COFF_DISTANCE = (float)MOD_BUS_Reg.COFF_DISTANCE_1000TIME * 0.001;
  
#if (MOTO_PRINTF_DEBUG)
  printf("max_wheel_speed = %f , COFF_001RPM_TO_MMS = %f\n", max_wheel_speed, COFF_001RPM_TO_MMS);
  printf("COFF_MMS_TO_D1RPM = %f , COFF_DISTANCE = %f\n", COFF_MMS_TO_D1RPM, COFF_DISTANCE);
#endif
}

void SetD1Rpm(MOTO_INDEX_ENUM MOTO_SELECT,s16 d1rpm)
{
  if(d1rpm > MAX_MOTO_SPEED_IN_D1RPM) d1rpm = MAX_MOTO_SPEED_IN_D1RPM;
  else if(d1rpm < -MAX_MOTO_SPEED_IN_D1RPM)  d1rpm = -MAX_MOTO_SPEED_IN_D1RPM;  
  
#if (DRIVER_SELECT == 0) // 前驱
  if((MOTO_SELECT == LEFT_MOTO_INDEX) || (MOTO_SELECT == LEFT_2_MOTO_INDEX))
  {
    RealRpm[MOTO_SELECT] = d1rpm;
    moto_speed_in_rpm[MOTO_SELECT] = d1rpm;
  }
  else if((MOTO_SELECT == RIGHT_MOTO_INDEX) || (MOTO_SELECT == RIGHT_2_MOTO_INDEX))
  {
    RealRpm[MOTO_SELECT] = d1rpm;
    moto_speed_in_rpm[MOTO_SELECT] = -d1rpm;
  }  
#else // 后驱
  if((MOTO_SELECT == LEFT_MOTO_INDEX) || (MOTO_SELECT == LEFT_2_MOTO_INDEX))
  {
    RealRpm[MOTO_SELECT] = d1rpm;
    moto_speed_in_rpm[MOTO_SELECT + 1] = -d1rpm;
  }
  else if((MOTO_SELECT == RIGHT_MOTO_INDEX) || (MOTO_SELECT == RIGHT_2_MOTO_INDEX))
  {
    RealRpm[MOTO_SELECT] = d1rpm;
    moto_speed_in_rpm[MOTO_SELECT - 1] = d1rpm;
  }  
#endif
}

void MOTO_IM_STOP(void)
{
  SetD1Rpm(LEFT_MOTO_INDEX ,0);
  SetD1Rpm(LEFT_2_MOTO_INDEX ,0);
  SetD1Rpm(RIGHT_MOTO_INDEX ,0);
  SetD1Rpm(RIGHT_2_MOTO_INDEX ,0);
}

void ResetMotoSpeedUpDownTime(void)
{
  moto_reset_speed_up_down_time_flag[LEFT_MOTO_INDEX] = 1;
  moto_reset_speed_up_down_time_flag[LEFT_2_MOTO_INDEX] = 1;
  moto_reset_speed_up_down_time_flag[RIGHT_MOTO_INDEX] = 1;  
  moto_reset_speed_up_down_time_flag[RIGHT_2_MOTO_INDEX] = 1;
}

/*********************************************
 ** 功能：减速任务，从当前速度匀减速到静止
 ** 参数：reset - 初始化标志，内部清除此标志
 **       time_in_ms - 减速的时间，单位：ms
 ********************************************/
void SLOW_DOWN_Task(u8* reset,u16 time_in_ms)
{
  static s16 slow_value_every_time[MOTO_NUM] = {0 ,0 ,0 ,0};
  static s16 Speed_bk[MOTO_NUM];
  u8 i;
  if(PID_TimeOut==0)
  {
    PID_TimeOut=100;
    if(*reset)
    {
      *reset=0;
      if(time_in_ms<100) time_in_ms=100;
      for(i = 0; i < MOTO_NUM; i++)
      {
        Speed_bk[i] = RealRpm[i];
        slow_value_every_time[i] = (s32)RealRpm[i] / ((s32)time_in_ms/100);
      }     
    }
    else
    {
      if((RealRpm[LEFT_MOTO_INDEX] != 0) || (RealRpm[RIGHT_MOTO_INDEX] != 0))    
      {
        for(i = 0; i < MOTO_NUM; i++)
        {
          if(abs(Speed_bk[i]) <= abs(slow_value_every_time[i]))
          {
            Speed_bk[i] = 0;
          }
          else 
          {
            Speed_bk[i] -= slow_value_every_time[i];
          }
          SetD1Rpm((MOTO_INDEX_ENUM)i, Speed_bk[i]);
        }   
      }
      else
      {
        for(i = 0;i < MOTO_NUM; i++)
        {
          SetD1Rpm((MOTO_INDEX_ENUM)i, 0);
        }    
      }
    }
  }
}

//向电机驱动器发送指令，使能电机驱动器
void Enable_Moto_RS485(u8 moto_enum, u8 status)
{
  u16 cal_crc;
  status = (status)?1:0;  
  memcpy(moto_comm_buff,MODBUS_MOTO_ENBALE,sizeof(MODBUS_MOTO_ENBALE));
  moto_comm_buff[0] = moto_enum + 1;
  moto_comm_buff[5] = status;
  
  cal_crc=ModBus_CRC16_Calculate(moto_comm_buff , 6);
  moto_comm_buff[6]=cal_crc&0xFF;
  moto_comm_buff[7]=cal_crc>>8;
  FillUartTxBufN(moto_comm_buff,sizeof(MODBUS_MOTO_ENBALE),MOTO_COMM_PORT_ENUM);
}

void SetMotoStop(u8 moto_enum, u8 status)
{
  u16 cal_crc;
  status = (status)?1:0;  
  memcpy(moto_comm_buff,MODBUS_MOTO_STOP_SET,sizeof(MODBUS_MOTO_STOP_SET));
  moto_comm_buff[0] = moto_enum + 1;
  moto_comm_buff[5] = status?1:0;
  
  cal_crc=ModBus_CRC16_Calculate(moto_comm_buff , 6);
  moto_comm_buff[6]=cal_crc&0xFF;
  moto_comm_buff[7]=cal_crc>>8;
  FillUartTxBufN(moto_comm_buff,sizeof(MODBUS_MOTO_STOP_SET),MOTO_COMM_PORT_ENUM);
}

void SetMotoSpeedUpTime(u8 moto_enum, u16 time)
{
  u16 cal_crc;
  memcpy(moto_comm_buff,MODBUS_MOTO_UP_TIME_SET,sizeof(MODBUS_MOTO_UP_TIME_SET));
  moto_comm_buff[0] = moto_enum + 1;
  moto_comm_buff[4] = time>>8;
  moto_comm_buff[5] = time&0xFF;
  
  cal_crc=ModBus_CRC16_Calculate(moto_comm_buff , 6);
  moto_comm_buff[6]=cal_crc&0xFF;
  moto_comm_buff[7]=cal_crc>>8;
  FillUartTxBufN(moto_comm_buff,sizeof(MODBUS_MOTO_UP_TIME_SET),MOTO_COMM_PORT_ENUM);
}

void SetMotoSpeedDownTime(u8 moto_enum, u16 time)
{
  u16 cal_crc;
  memcpy(moto_comm_buff,MODBUS_MOTO_DOWN_TIME_SET,sizeof(MODBUS_MOTO_DOWN_TIME_SET));
  moto_comm_buff[0] = moto_enum + 1;
  moto_comm_buff[4] = time>>8;
  moto_comm_buff[5] = time&0xFF;
  
  cal_crc=ModBus_CRC16_Calculate(moto_comm_buff , 6);
  moto_comm_buff[6]=cal_crc&0xFF;
  moto_comm_buff[7]=cal_crc>>8;
  FillUartTxBufN(moto_comm_buff,sizeof(MODBUS_MOTO_DOWN_TIME_SET),MOTO_COMM_PORT_ENUM);
}

void SetMotoMaxCurrent(u8 moto_enum, u16 max_curr)
{
  u16 cal_crc;
  memcpy(moto_comm_buff, MODBUS_MOTO_MAX_EA_SET, sizeof(MODBUS_MOTO_MAX_EA_SET));
  moto_comm_buff[0] = moto_enum + 1;
  moto_comm_buff[4] = max_curr >> 8;
  moto_comm_buff[5] = max_curr & 0xFF;
  
  cal_crc = ModBus_CRC16_Calculate(moto_comm_buff , 6);
  moto_comm_buff[6] = cal_crc & 0xFF;
  moto_comm_buff[7] = cal_crc >> 8;
  FillUartTxBufN(moto_comm_buff, sizeof(MODBUS_MOTO_MAX_EA_SET), MOTO_COMM_PORT_ENUM);
}

//向电机驱动器发送指令，控制电机的转速，单位: RPM
/*
void SetMotoRpm(u8 moto_enum,s16 rpm)
{
  u16 cal_crc;
  memcpy(moto_comm_buff,MODBUS_MOTO_RPM_SET,sizeof(MODBUS_MOTO_RPM_SET));
  moto_comm_buff[0] = moto_enum + 1;
  moto_comm_buff[4] = (rpm >> 8)&0xFF;
  moto_comm_buff[5] = rpm &0xFF;
  cal_crc=ModBus_CRC16_Calculate(moto_comm_buff , sizeof(MODBUS_MOTO_RPM_SET)-2);
  moto_comm_buff[6]=cal_crc&0xFF;
  moto_comm_buff[7]=cal_crc>>8;
  FillUartTxBufN(moto_comm_buff,sizeof(MODBUS_MOTO_RPM_SET), MOTO_COMM_PORT_ENUM);
}
*/

void SetMotoD01Rpm(u8 moto_enum,s16 d01rpm)
{
  u16 cal_crc;
  memcpy(moto_comm_buff,MODBUS_MOTO_RPM_SET,sizeof(MODBUS_MOTO_RPM_SET));
  moto_comm_buff[0] = moto_enum + 1;
  moto_comm_buff[4] = (d01rpm >> 8)&0xFF;
  moto_comm_buff[5] = d01rpm &0xFF;
  cal_crc=ModBus_CRC16_Calculate(moto_comm_buff , sizeof(MODBUS_MOTO_RPM_SET)-2);
  moto_comm_buff[6]=cal_crc&0xFF;
  moto_comm_buff[7]=cal_crc>>8;
  FillUartTxBufN(moto_comm_buff,sizeof(MODBUS_MOTO_RPM_SET), MOTO_COMM_PORT_ENUM);
}

//向电机驱动器发送指令，读取电机的转速，单位: 0.01RPM
void ReadMotoRpm(u8 moto_enum)
{
  u16 cal_crc;
  memcpy(moto_comm_buff,MODBUS_MOTO_RPM_READ,sizeof(MODBUS_MOTO_RPM_READ));  
  moto_comm_buff[0] = moto_enum + 1;
  cal_crc=ModBus_CRC16_Calculate(moto_comm_buff , sizeof(MODBUS_MOTO_RPM_READ)-2);
  moto_comm_buff[6]=cal_crc&0xFF;
  moto_comm_buff[7]=cal_crc>>8;  
  FillUartTxBufN(moto_comm_buff,sizeof(MODBUS_MOTO_RPM_READ), MOTO_COMM_PORT_ENUM);
}

typedef enum
{
  MOTO_INIT = 0,
  MOTO_SET_RPM,
  MOTO_SET_IDEL,
  MOTO_ACTION_NUM
}MOTO_ACTION;

u32 Left_A_TO_1000 = 0;
u32 Left_A_TO_3500 = 0;
u32 RIGHT_A_TO_1000 = 0;
u32 RIGHT_A_TO_3500 = 0;
/************************************
 ** 电机驱动器的控制任务
 ** 功能：
 **  1. 初始化，使能电机驱动器
 **  2. 设置电机的速度 rpm
 **  3. 电机速度为0后，持续[n]秒钟电机自由(失能电机驱动器)
 ***********************************/
void MOTO_SPEED_CONTROL_TASK(void)
{
  static s16 moto_speed_in_rpm_bk[MOTO_NUM] = {0 ,0 ,0 ,0};
  static u32 moto_disable_time_counter[MOTO_NUM] = {0 ,0 ,0 ,0};
  static u8 pro = 0;
  static u8 moto_enum = 0;
  static u8 InitIndex = 0;
  switch(pro)
  {
  case MOTO_INIT: // Init : [set_free]
    {
      if(MOTO_485COMM_Timeout == 0)
      {
        if(moto_enum < MOTO_NUM)
        {
          switch(InitIndex)
          {
          case 0:
            {
              SetMotoSpeedUpTime(moto_enum, MOTO_SPEED_UP_DOWN_DELAY_TIME);
            }
            break;
          case 1:
            {
              SetMotoSpeedDownTime(moto_enum, MOTO_SPEED_UP_DOWN_DELAY_TIME);
            }
            break;
          case 2:
            {
              moto_enable_status[moto_enum] =0;
              Enable_Moto_RS485(moto_enum, moto_enable_status[moto_enum]);   
              SET_DIDO_Relay(moto_enum + DIDO_BREAK_0, moto_enable_status[moto_enum]? BREAK_OFF:BREAK_ON);   
            }
            break;
          case 3: // useless
            {
              SetMotoMaxCurrent(moto_enum, Moto_MaxCurrent[moto_enum]);
              printf("useless_0\n");
            }
            break;            
          default: ;
          }
          MOTO_485COMM_Timeout = MOTO_CONTROL_CYCLE;
          moto_enum += 1;
        }
        else
        {
          moto_enum = 0;
          InitIndex += 1;
          if(InitIndex >= 3) pro += 1; // >=4
        }
      }
    }
    break;
  case 1: // Init : [set_speed]
    {
      if(MOTO_485COMM_Timeout == 0)
      {
        if(moto_enum < MOTO_NUM)
        {
          MOTO_485COMM_Timeout = MOTO_CONTROL_CYCLE;
          moto_speed_in_rpm_bk[moto_enum] = 0;
          moto_speed_in_rpm[moto_enum] =0;
          SetMotoD01Rpm(moto_enum, moto_speed_in_rpm_bk[moto_enum]);
          SetMotoRpmTimes[moto_enum] += 1;
          moto_enum += 1;
        }
        else
        {
          moto_enum = 0;
          pro += 1;
        }
      }
    }
    break;    
  case 2: //nomal : [set_enable] + [set_speed]
    {
      if(MOTO_485COMM_Timeout == 0)
      {
        if(moto_enum < MOTO_NUM)
        {
          if(moto_speed_in_rpm_bk[moto_enum] != moto_speed_in_rpm[moto_enum])
          {
            if((moto_speed_in_rpm[moto_enum] != 0)&&(moto_enable_status[moto_enum] == 0))
            {
              MOTO_485COMM_Timeout = MOTO_CONTROL_CYCLE;
              moto_enable_status[moto_enum] = 1;
              Enable_Moto_RS485(moto_enum, moto_enable_status[moto_enum]);
              if(AGV_RUN_Pro != AGV_STATUS_REMOTE)
              {
                SET_DIDO_Relay(moto_enum + DIDO_BREAK_0, moto_enable_status[moto_enum]? BREAK_OFF:BREAK_ON); 
              }
              break;
            }
            MOTO_485COMM_Timeout = MOTO_CONTROL_CYCLE;
            moto_speed_in_rpm_bk[moto_enum] = moto_speed_in_rpm[moto_enum];
            SetMotoD01Rpm(moto_enum, moto_speed_in_rpm_bk[moto_enum]);
            SetMotoRpmTimes[moto_enum] += 1;
          }
          moto_enum += 1;
        }
        else
        {
          moto_enum = 0;
          pro += 1;
        }        
      }
    }
    break;
  case 3: // nomal : [set_free]
    {
      if(MOTO_485COMM_Timeout == 0)
      {
        if(moto_enum < MOTO_NUM)
        {
          if(moto_enable_status_change_flag[moto_enum] != 0)
          {
            MOTO_485COMM_Timeout = MOTO_CONTROL_CYCLE;
            moto_enable_status_change_flag[moto_enum] = 0;
            Enable_Moto_RS485(moto_enum, moto_enable_status[moto_enum]);
            if(AGV_RUN_Pro != AGV_STATUS_REMOTE)
            {
              SET_DIDO_Relay(moto_enum + DIDO_BREAK_0, moto_enable_status[moto_enum]? BREAK_OFF:BREAK_ON); 
            }
          
          }
          moto_enum += 1;
        }
        else
        {
          moto_enum = 0;
          pro += 1;
        }
      }
    }
    break;
  case 4: // nomal : [Read RPM]
    {
      if(MOTO_485COMM_Timeout == 0)
      {
        if(moto_enum < MOTO_NUM)
        {
            if(MOTO_READ_RPM_Timeout[moto_enum] == 0)
            {
              MOTO_485COMM_Timeout = MOTO_CONTROL_CYCLE;
              MOTO_READ_RPM_Timeout[moto_enum] = DEFAULT_MOTO_READ_RPM_TIME_OUT;
              ReadMotoRpm(moto_enum);
              ReadMotoRpmTimes[moto_enum] += 1;
            }
            moto_enum += 1;
        }
        else
        {
          moto_enum = 0;
          pro += 1; //pro = 2;
        }        
      }
    }
    break;
  case 5: // set speed up down time
    {
      if(MOTO_485COMM_Timeout == 0)
      {
        if(moto_enum < MOTO_NUM)
        {
            if(moto_reset_speed_up_down_time_flag[moto_enum] != 0)
            {
              MOTO_485COMM_Timeout = MOTO_CONTROL_CYCLE;
              SetMotoSpeedUpTime(moto_enum, MOTO_SPEED_UP_DOWN_DELAY_TIME);
            }
            moto_enum += 1;
        }
        else
        {
          moto_enum = 0;
          pro += 1;
        }        
      }
    }
    break;
  case 6: // set speed down time
    {
      if(MOTO_485COMM_Timeout == 0)
      {
        if(moto_enum < MOTO_NUM)
        {
            if(moto_reset_speed_up_down_time_flag[moto_enum] != 0)
            {
              moto_reset_speed_up_down_time_flag[moto_enum] = 0;
              MOTO_485COMM_Timeout = MOTO_CONTROL_CYCLE;
              SetMotoSpeedDownTime(moto_enum, MOTO_SPEED_UP_DOWN_DELAY_TIME);
            }
            moto_enum += 1;
        }
        else
        {
          moto_enum = 0;
          pro = 2;
        }        
      }
    }
    break;    
  case 7: // set max current - useless
    {
      if(MOTO_485COMM_Timeout == 0)
      {
        if(moto_enum < MOTO_NUM)
        {
            if(moto_reset_max_current_flag[moto_enum] != 0)
            {
              moto_reset_max_current_flag[moto_enum] = 0;
              MOTO_485COMM_Timeout = MOTO_CONTROL_CYCLE;
              SetMotoMaxCurrent(moto_enum, Moto_MaxCurrent[moto_enum]);
              printf("useless_1\n");
            }
            moto_enum += 1;
        }
        else
        {
          moto_enum = 0;
          pro = 2;
        }        
      }
    }
    break;
  case 8://useless
    {
      static u8 IM_STOP_Flag[MOTO_NUM]={0 ,0 ,0 ,0};
      static u8 IM_STOP_Status[MOTO_NUM]={0 ,0 ,0 ,0};
      if(MOTO_485COMM_Timeout == 0)
      {
        if(moto_enum < MOTO_NUM)
        {
            if(IM_STOP_Flag[moto_enum] != 0)
            {
              MOTO_485COMM_Timeout = MOTO_CONTROL_CYCLE;
              IM_STOP_Flag[moto_enum] = 0;
              SetMotoStop(moto_enum, IM_STOP_Status[moto_enum]);
            }
            moto_enum += 1;
        }
        else
        {
          moto_enum = 0;
          pro = 2;
        }        
      }
    }
    break;
  default: pro = 0;
  }
  
  //电机速度为0后，持续[n]秒钟电机自由
  if(1)
  {
    static u32 NumOfSysTickIntBk = 0;
    u8 i;
    if(NumOfSysTickInt!=NumOfSysTickIntBk)
    {
      NumOfSysTickIntBk = NumOfSysTickInt;
      for(i = 0; i < MOTO_NUM; i++)
      {
        if(moto_speed_in_rpm_bk[i] == 0)
        {        
          if(moto_enable_status[i] != 0)
          {
            moto_disable_time_counter[i] += 1;
            if(moto_disable_time_counter[i] >= MOTO_ZERO_FREE_TIME_IN_MS)
            {
              moto_disable_time_counter[i] = 0;
              moto_enable_status[i] = 0;
              
              moto_enable_status_change_flag[i] = 1;

            }
          }
        }
        else
        {
          moto_disable_time_counter[i] = 0;
        }
      }
    }
  }
  

  //-- 最大电流设定 --//
  /*
  if(MOTO_CheckMaxCurrent_Timeout == 0)
  {
    static s32 dir_time = 0;
    s32 abs_left_rpm, abs_right_rpm, abs_max_rpm, diff_rpm_rate, dir;
    MOTO_CheckMaxCurrent_Timeout = DEFAULT_MOTO_CHECK_MAX_CURRENT_TIMEOUT; 
    abs_left_rpm = abs_32(moto_speed_in_rpm[LEFT_MOTO_INDEX]);
    abs_right_rpm = abs_32(moto_speed_in_rpm[RIGHT_MOTO_INDEX]);
    abs_max_rpm = (abs_left_rpm > abs_right_rpm) ? abs_left_rpm : abs_right_rpm;
    dir = abs_left_rpm - abs_right_rpm;
    diff_rpm_rate = abs_32(abs_left_rpm - abs_right_rpm) * 100 / abs_max_rpm;
    
    if((abs_max_rpm > 100) && (diff_rpm_rate >= 5)) // rpm > 100 , diff_rate > 5%
    {
      if(dir < 0) 
      {
        if(dir_time > -CHECK_MAX_CURRENT_HOLD_TIME) 
          dir_time += -DEFAULT_MOTO_CHECK_MAX_CURRENT_TIMEOUT;     
      }
      else 
      {
        if(dir_time < CHECK_MAX_CURRENT_HOLD_TIME) 
          dir_time += DEFAULT_MOTO_CHECK_MAX_CURRENT_TIMEOUT;     
      }
    }
    else
    {
      if(dir_time <= -DEFAULT_MOTO_CHECK_MAX_CURRENT_TIMEOUT) 
        dir_time += DEFAULT_MOTO_CHECK_MAX_CURRENT_TIMEOUT;
      else if(dir_time >= DEFAULT_MOTO_CHECK_MAX_CURRENT_TIMEOUT) 
        dir_time -= DEFAULT_MOTO_CHECK_MAX_CURRENT_TIMEOUT;
      else dir_time = 0;
    }
    
    if((dir_time <= -CHECK_MAX_CURRENT_HOLD_TIME) 
       && (Moto_MaxCurrent[LEFT_MOTO_INDEX] != MAX_MOTO_EA_THRESHOLD_1))
    {
      moto_reset_max_current_flag[LEFT_MOTO_INDEX] = 1;
      moto_reset_max_current_flag[LEFT_2_MOTO_INDEX] = 1;
      Moto_MaxCurrent[LEFT_MOTO_INDEX] = MAX_MOTO_EA_THRESHOLD_1;
      Moto_MaxCurrent[LEFT_2_MOTO_INDEX] = MAX_MOTO_EA_THRESHOLD_1;
      
      Left_A_TO_1000 += 1;
    }
    if((dir_time >= CHECK_MAX_CURRENT_HOLD_TIME) 
       && (Moto_MaxCurrent[RIGHT_MOTO_INDEX] != MAX_MOTO_EA_THRESHOLD_1))
    {
      moto_reset_max_current_flag[RIGHT_MOTO_INDEX] = 1;
      moto_reset_max_current_flag[RIGHT_2_MOTO_INDEX] = 1;
      Moto_MaxCurrent[RIGHT_MOTO_INDEX] = MAX_MOTO_EA_THRESHOLD_1;
      Moto_MaxCurrent[RIGHT_2_MOTO_INDEX] = MAX_MOTO_EA_THRESHOLD_1;
      
      RIGHT_A_TO_1000 += 1;
    }
    
    if((dir_time == 0)
       && (Moto_MaxCurrent[LEFT_MOTO_INDEX] != MAX_MOTO_EA_THRESHOLD_0))
    {
      moto_reset_max_current_flag[LEFT_MOTO_INDEX] = 1;
      moto_reset_max_current_flag[LEFT_2_MOTO_INDEX] = 1;
      Moto_MaxCurrent[LEFT_MOTO_INDEX] = MAX_MOTO_EA_THRESHOLD_0;
      Moto_MaxCurrent[LEFT_2_MOTO_INDEX] = MAX_MOTO_EA_THRESHOLD_0;
      
      Left_A_TO_3500 += 1;
    }
    
    if((dir_time == 0)
       && (Moto_MaxCurrent[RIGHT_MOTO_INDEX] != MAX_MOTO_EA_THRESHOLD_0))
    {
      moto_reset_max_current_flag[RIGHT_MOTO_INDEX] = 1;
      moto_reset_max_current_flag[RIGHT_2_MOTO_INDEX] = 1;
      Moto_MaxCurrent[RIGHT_MOTO_INDEX] = MAX_MOTO_EA_THRESHOLD_0;
      Moto_MaxCurrent[RIGHT_2_MOTO_INDEX] = MAX_MOTO_EA_THRESHOLD_0;
      
      RIGHT_A_TO_3500 += 1;
    }    
  }
  */
}