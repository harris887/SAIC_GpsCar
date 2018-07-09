#include "user_inc.h"
#include "string.h"
#include "stdlib.h"

#define PROGRAM_PRINTF_DEBUG          0

#define DEFAULT_CHARGE_LONG_TIME_OUT  (10*3600*1000)  //10hours
#define SYS_ON_Flag                   (1)

AGV_STATUS_LIST AGV_RUN_Pro=AGV_STATUS_INIT;
u8 AGV_RUN_SUB_Pro=0;
u32 AGV_Delay=0;
u32 ChargeLongTimeout=DEFAULT_CHARGE_LONG_TIME_OUT;
u16 RFID_STOP_ANGIN_Timeout=0;
u16 StartAutoChargeFlag=0;
u16 StopAutoChargeFlag=0;
u16 StartLeaveChargeHouse=0;
u16 ProgramControlCycle=0;
float VehicleWidth=DEFAULT_Vehicle_WIDTH_FLOAT;
float Displacement_coff=1.0;//1.0
float Angle_coff=1.0;
u32 Wonder_Disp_or_Angle_value=0;//
float Finish_Disp_or_Angle_value=0;
u16 SPEED_UP_Length=0;
SPEED_UP_OPTION SPEED_UP_OPTION_List[RunFuncNum][MAX_SPEED_UP_LIST_LENGTH];
u16 SuperMode=0;//碰撞后，由上位机控制

MOVEMENT_OPTION_LIST DISPLACEMENT_MOVEMENT_OPTION_List={0,0};
MOVEMENT_OPTION_LIST ANGLE_MOVEMENT_OPTION_List={0,0};
u16 IMU_Angle = 0;
u16 Software_IM_STOP = 0;

void MovementListInit(void)
{
  memset(&DISPLACEMENT_MOVEMENT_OPTION_List,0,sizeof(DISPLACEMENT_MOVEMENT_OPTION_List));
  memset(&ANGLE_MOVEMENT_OPTION_List,0,sizeof(ANGLE_MOVEMENT_OPTION_List));
}
void ClearMovementList(MOVEMENT_OPTION_LIST* pLIST)
{
  pLIST->Out_index=pLIST->In_index;
  SPEED_OPTION_List.OutIndex = SPEED_OPTION_List.InIndex;
}

void AGV_RUN_Task(void)
{
  if(AGV_RUN_Pro!=AGV_STATUS_INIT) LED_WATER_Display(500);
  
  switch(AGV_RUN_Pro)
  {
  /*----------初始化模式---------------*/    
  case AGV_STATUS_INIT:
    {
      if(AGV_RUN_SUB_Pro==0)
      {//复位所有外部
        MOTO_IM_STOP();
        LED_DISPLAY_Reset();
        AGV_Delay=1200;
        AGV_RUN_SUB_Pro+=1;
      }   
      else
      {
        if((SYS_ON_Flag!=0)&&(AGV_Delay==0))
        {
          SetBeep(3,300,700);
          AGV_Delay=3000;
          LED_DISPLAY_Reset();
          ClearMovementList(&DISPLACEMENT_MOVEMENT_OPTION_List);
          ClearMovementList(&ANGLE_MOVEMENT_OPTION_List);
          AGV_RUN_Pro=AGV_STATUS_IDLE;
          AGV_RUN_SUB_Pro=0;
        }
      }
    }
    break;
  /*----------空闲模式---------------*/    
  case AGV_STATUS_IDLE:
    {
      if(AGV_RUN_SUB_Pro==0)
      {
        if(AGV_Delay==0)
        {
#if (PROGRAM_PRINTF_DEBUG)
          printf("IDLE\n");
#endif
          AGV_Delay=1000;
          MODE_BUS_HALL_Addr=DEFAULT_MODE_BUS_HALL_ADDR;
          ResetMotoSpeedUpDownTime();
          AGV_RUN_SUB_Pro+=1;
        }
      }
      else if(AGV_RUN_SUB_Pro==1)
      {
        if(SYS_ON_Flag==0)
        {
          AGV_RUN_Pro=AGV_STATUS_INIT;
          AGV_RUN_SUB_Pro=0;
        }
        
        //(a)急停按钮按下，进入急停模式
        if(BUTTON_IM_STOP_Flag)
        {
          AGV_RUN_Pro=AGV_STATUS_IM_STOP;
          AGV_RUN_SUB_Pro=0;    
          break;
        }       
        
        if(Software_IM_STOP)
        {
          AGV_RUN_Pro=AGV_STATUS_IM_STOP_SOFT;
          AGV_RUN_SUB_Pro=0;  
          break;
        }         
        
        //(b)光电/触碰触发，进入臂章模式
        if((BARRIER_Flag & BARRIER_SENSOR_MASK) && (SuperMode == 0))
        {
          AGV_RUN_Pro=AGV_STATUS_BARRIER;
          AGV_RUN_SUB_Pro=0;    
          break;
        }          
        
        //(c)上位机启动信号&&霍尔检测到磁条
        if((StartAutoChargeFlag!=0)
           &&(ON_LINE_Flag)
             &&(AGV_Delay==0))
        {
          AGV_RUN_Pro=AGV_STATUS_FOLLOWLINE;
          AGV_RUN_SUB_Pro=0;
          break;
        }
           
        //(d)用户控制模式
        if((MOD_BUS_Reg.M_CONTROL_MODE==M_CONTROL_MODE_SPEED)
          ||((MOD_BUS_Reg.M_CONTROL_MODE==M_CONTROL_MODE_DISPLACEMENT)&&(DISPLACEMENT_MOVEMENT_OPTION_List.In_index!=DISPLACEMENT_MOVEMENT_OPTION_List.Out_index))
          ||((MOD_BUS_Reg.M_CONTROL_MODE==M_CONTROL_MODE_ANGLE)&&(ANGLE_MOVEMENT_OPTION_List.In_index!=ANGLE_MOVEMENT_OPTION_List.Out_index))
          //||(MOD_BUS_Reg.M_CONTROL_MODE==M_CONTROL_MODE_SPEC)
          )
        {
          AGV_RUN_Pro=AGV_STATUS_USER_PROGRAM;
          AGV_RUN_SUB_Pro=0;
          break;
        }
        
        //(e)遥控ON按钮按下进入，遥控状态
        if(REMOTE_SelectFlag)
        {
          AGV_RUN_Pro=AGV_STATUS_REMOTE;
          AGV_RUN_SUB_Pro=0;
          break;
        }

        //(f)低电压状态切换
        if(BatteryVolt_LowFlag>=2)
        {
          AGV_RUN_Pro=AGV_STATUS_LOW_POWER;
          AGV_RUN_SUB_Pro=0;
          break;
        }

        //(g)沿磁条退出充电坞
        if(StartLeaveChargeHouse)
        {
          StartLeaveChargeHouse=0;  
          AGV_RUN_Pro=AGV_STATUS_FOLLOWLINE_OUT;
          AGV_RUN_SUB_Pro=0;
          break;          
        }        
        
        BATT_LOW_LEVEL_1_Warning();        
      }     
    }
    break;
  /*----------巡线模式(前进去充电)---------------*/      
  case AGV_STATUS_FOLLOWLINE:
    {
      static u8 FollowLineReset=0;
      if(AGV_RUN_SUB_Pro==0)
      {
#if (PROGRAM_PRINTF_DEBUG)
        printf("FOLLOW LINE\n");
#endif
        PID_Init();
        LED_DISPLAY_Reset();
        AGV_RUN_SUB_Pro+=1;
        RFID_COMEIN_Flag&=~1;
        FollowLineReset=1;
      }
      else
      {
        //绿灯1HZ闪烁
        //LED_FOLLOW_LINE_Display(500);
        
        //巡线前进
        //FOLLOW_LINE_TASK(&FollowLineReset,1);
        RFID_COMEIN_Flag = FOLLOW_LINE_ADVANCE_TASK(&FollowLineReset,PGV_LANE_TACKING_Infor.XP/10,118);//118
        
        if(SYS_ON_Flag==0)
        {
          AGV_RUN_Pro=AGV_STATUS_INIT;
          AGV_RUN_SUB_Pro=0;
        }         
        
        //(a)急停按钮按下，进入急停模式
        if(BUTTON_IM_STOP_Flag)
        {
          AGV_RUN_Pro=AGV_STATUS_IM_STOP;
          AGV_RUN_SUB_Pro=0;    
          break;
        }
        
        if(Software_IM_STOP)
        {
          AGV_RUN_Pro=AGV_STATUS_IM_STOP_SOFT;
          AGV_RUN_SUB_Pro=0;  
          break;
        }         
        
        //(b)光电/触碰触发，进入臂章模式
        if(BARRIER_Flag & BARRIER_SENSOR_MASK)
        {
          AGV_RUN_Pro=AGV_STATUS_BARRIER;
          AGV_RUN_SUB_Pro=0;    
          break;
        }
        
        //(c)遥控ON按钮按下进入，遥控状态
        if(REMOTE_SelectFlag)
        {
          //结束充电流程，清除充电标志
          StartAutoChargeFlag=0;
          StopAutoChargeFlag=0;           
          AGV_RUN_Pro=AGV_STATUS_REMOTE;
          AGV_RUN_SUB_Pro=0;
          break;
        }        

        //(d)脱离磁带后，进入脱离磁条模式
        if(ON_LINE_Flag==0)
        {
          //结束充电流程，清除充电标志
          StartAutoChargeFlag=0;
          StopAutoChargeFlag=0;          
          //AGV_RUN_Pro=AGV_STATUS_OFF_LINE;
          
          RFID_COMEIN_Flag&=~1;
          AGV_RUN_Pro=AGV_STATUS_RFID_COMEIN;
         
          AGV_RUN_SUB_Pro=0;  
          break;
        }

        //(e)遇到RFID，进入[扫描到RFID模式]，刹车准备充电
        if(RFID_COMEIN_Flag&1)  
        {
          RFID_COMEIN_Flag&=~1;
          AGV_RUN_Pro=AGV_STATUS_RFID_COMEIN;
          AGV_RUN_SUB_Pro=0;   
          break;
        }  
        
        BATT_LOW_LEVEL_1_Warning(); 
      }
    }
    break;
    /*----------巡线模式 (自动退出充电区域)---------------*/  
  case AGV_STATUS_FOLLOWLINE_OUT:
    {
      static u8 FollowLineReset_1=0;
      static u32 time_bk;
      static u16 zero_counter;
      if(AGV_RUN_SUB_Pro==0)
      {
#if (PROGRAM_PRINTF_DEBUG)
        printf("FOLLOW LINE OUT CHARGE AREA\n");
#endif
        //PID_Init();
        zero_counter=0;
        AGV_Delay=30000;//30s超时退出
        LED_DISPLAY_Reset();
        AGV_RUN_SUB_Pro+=1;
        FollowLineReset_1=1;
      }
      else
      {
        LED_FOLLOW_LINE_Display(500);
        
        AGV_USER_PROGRAM_IN_DISPLACEMENT_Task(&FollowLineReset_1);
       
        //检测后退是否完毕
        if(time_bk!=NumOfSysTickInt)
        {
          time_bk=NumOfSysTickInt;
          if(RealRpm[LEFT_MOTO_INDEX] == 0)
          {
            zero_counter+=1;
          }
          else
          {
            zero_counter=0;
          }          
        }

        if(SYS_ON_Flag==0)
        {
          AGV_RUN_Pro=AGV_STATUS_INIT;
          AGV_RUN_SUB_Pro=0;
        }        
        
        //(a)急停按钮按下，进入急停模式
        if(BUTTON_IM_STOP_Flag)
        {
          AGV_RUN_Pro=AGV_STATUS_IM_STOP;
          AGV_RUN_SUB_Pro=0;    
          break;
        }
        
        if(Software_IM_STOP)
        {
          AGV_RUN_Pro=AGV_STATUS_IM_STOP_SOFT;
          AGV_RUN_SUB_Pro=0;  
          break;
        }         
        
        //(b)光电/触碰触发，进入臂章模式
        if(BARRIER_Flag & BARRIER_SENSOR_MASK)
        {
          AGV_RUN_Pro=AGV_STATUS_BARRIER;
          AGV_RUN_SUB_Pro=0;    
          break;
        }
        
        //(c)遥控ON按钮按下进入，遥控状态
        if(REMOTE_SelectFlag)
        {
          //结束充电流程，清除充电标志
          StartAutoChargeFlag=0;
          StopAutoChargeFlag=0;           
          AGV_RUN_Pro=AGV_STATUS_REMOTE;
          AGV_RUN_SUB_Pro=0;
          break;
        }        

        //(d)等待后退停稳后，进入空闲状态
        //if(ON_LINE_Flag==0)
        if((AGV_Delay==0)||(zero_counter>=1000))
        {
          AGV_Delay=500;
          //结束充电流程，清除充电标志
          StartAutoChargeFlag=0;
          StopAutoChargeFlag=0;             
          AGV_RUN_Pro=AGV_STATUS_IDLE;
          AGV_RUN_SUB_Pro=0;  
          break;
        }
        
        BATT_LOW_LEVEL_1_Warning();         
      }
    }
    break;
    /*巡线中，脱离磁条后的模式*/
  case AGV_STATUS_OFF_LINE:
    {
      static u8 slow_down_reset=0;
      if(AGV_RUN_SUB_Pro==0)
      {
#if (PROGRAM_PRINTF_DEBUG)
        printf("OFF LINE\n");
#endif
        //MOTO_IM_STOP();
        LED_DISPLAY_Reset();
        AGV_Delay=10000;//10s后，进入空闲模式
        slow_down_reset=1;
        AGV_RUN_SUB_Pro+=1;      
      }
      else
      {
        //LED_BARRIER_Display(1000);//黄灯闪烁
        SetBeepForever(500,1500);//警报音
        
        //1秒时间匀减速刹车，待测试**
        SLOW_DOWN_Task(&slow_down_reset,1000);
        
        if(SYS_ON_Flag==0)
        {
          AGV_RUN_Pro=AGV_STATUS_INIT;
          AGV_RUN_SUB_Pro=0;
        }        
        
        //(a)遥控ON按钮按下进入，遥控状态
        if(REMOTE_SelectFlag)
        {
          AGV_RUN_Pro=AGV_STATUS_REMOTE;
          AGV_RUN_SUB_Pro=0;
          break;
        }   
        
        //(b)急停按钮按下，进入急停模式
        if(BUTTON_IM_STOP_Flag)
        {
          AGV_RUN_Pro=AGV_STATUS_IM_STOP;
          AGV_RUN_SUB_Pro=0;    
          break;
        }     
        
        if(Software_IM_STOP)
        {
          AGV_RUN_Pro=AGV_STATUS_IM_STOP_SOFT;
          AGV_RUN_SUB_Pro=0;  
          break;
        }         
        
        //(c)10秒后，进入空闲模式
        if(AGV_Delay==0) 
        {
          LED_DISPLAY_Reset();
          AGV_RUN_Pro=AGV_STATUS_IDLE;
          AGV_RUN_SUB_Pro=0;
          break;
        }               
        
        BATT_LOW_LEVEL_1_Warning(); 
      }
    }
    break;
  /*----------RFID等待模式----------*/  
  case AGV_STATUS_RFID_COMEIN:
    {
      static u8 reset=0;
      if(AGV_RUN_SUB_Pro==0)
      {
#if (PROGRAM_PRINTF_DEBUG)
        printf("RFID COMEIN\n");
#endif
        AGV_Delay=2000;//等待2000ms
        reset=1;
        SetBeep(2,200,500);
        LED_DISPLAY_Reset();
        AGV_RUN_SUB_Pro+=1;
      }
      else
      {
        //蓝灯闪烁1HZ
        //LED_RFID_Display(500);        

        //1秒时间匀减速刹车，待测试**
        SLOW_DOWN_Task(&reset,1000);
        
        if(SYS_ON_Flag==0)
        {
          AGV_RUN_Pro=AGV_STATUS_INIT;
          AGV_RUN_SUB_Pro=0;
        }        
        
        //(a)遥控ON按钮按下进入，遥控状态
        if(REMOTE_SelectFlag)
        {
          AGV_RUN_Pro=AGV_STATUS_REMOTE;
          AGV_RUN_SUB_Pro=0;
          break;
        }
        
        //(b)急停按钮按下，进入急停模式
        if(BUTTON_IM_STOP_Flag)
        {
          AGV_RUN_Pro=AGV_STATUS_IM_STOP;
          AGV_RUN_SUB_Pro=0;    
          break;
        }
        
        if(Software_IM_STOP)
        {
          AGV_RUN_Pro=AGV_STATUS_IM_STOP_SOFT;
          AGV_RUN_SUB_Pro=0;  
          break;
        }         
        
        //(c)光电/触碰触发，进入臂章模式
        if(BARRIER_Flag & BARRIER_SENSOR_MASK)
        {
          AGV_RUN_Pro=AGV_STATUS_BARRIER;
          AGV_RUN_SUB_Pro=0;    
          break;
        }        
        
        //(d)2秒钟后，去充电模式
        if(AGV_Delay==0)
        {
          //关闭巡线传感器电源
          Relay_status&=~0x10;
          //SetRelay(RELAY_PGV_POWER_Index,RELAY_OFF);          
          MOTO_IM_STOP();
          RFID_STOP_ANGIN_Timeout=5000;
          AGV_Delay=1500;
          AGV_RUN_Pro=AGV_STATUS_CHARGE;
          AGV_RUN_SUB_Pro=0;  
          break;
        }        
      }
    }
    break;
  /*----------遥控模式---------------*/    
  case AGV_STATUS_REMOTE:
    {
      if(AGV_RUN_SUB_Pro==0)
      {
#if (PROGRAM_PRINTF_DEBUG)
        printf("REMOTE MODE\n");
#endif
        AGV_Delay = 100;
        LED_DISPLAY_Reset();
        AGV_RUN_SUB_Pro+=1;
      }
      else
      {
        //响应摇杆
        REMOTE_Task();
        
        if(SYS_ON_Flag==0)
        {
          AGV_RUN_Pro=AGV_STATUS_INIT;
          AGV_RUN_SUB_Pro=0;
        }        
        
        //(a)急停按钮按下，进入急停模式
        if(BUTTON_IM_STOP_Flag)
        {
          AGV_RUN_Pro=AGV_STATUS_IM_STOP;
          AGV_RUN_SUB_Pro=0;  
          break;
        }
        
        if(Software_IM_STOP)
        {
          AGV_RUN_Pro=AGV_STATUS_IM_STOP_SOFT;
          AGV_RUN_SUB_Pro=0;  
          break;
        }         
        
        //(b)进入空闲模式
        if(REMOTE_SelectFlag==0)
        {
          AGV_Delay=1500;
          LED_DISPLAY_Reset();
          AGV_RUN_Pro=AGV_STATUS_IDLE;
          AGV_RUN_SUB_Pro=0;  
          break;
        }         
        
        BATT_LOW_LEVEL_1_Warning(); 
      }
    }
    break;
  /*----------低电压模式---------------*/     
  case AGV_STATUS_LOW_POWER:
    {
      if(AGV_RUN_SUB_Pro==0)
      {
#if (PROGRAM_PRINTF_DEBUG)
        printf("LOW POWER\n");
#endif
        MOTO_IM_STOP();
        AGV_Delay=10000;
        MODE_BUS_HALL_Addr=DEFAULT_MODE_BUS_HALL_ADDR;        
        LED_DISPLAY_Reset();
        AGV_RUN_SUB_Pro+=1;
      }
      else
      {
        //4个指示灯1HZ闪动
        LED_LOW_POWER_Display(500);
        
        //蜂鸣器1HZ警报
        SetBeepForever(300,700);
        
        if(SYS_ON_Flag==0)
        {
          AGV_RUN_Pro=AGV_STATUS_INIT;
          AGV_RUN_SUB_Pro=0;
        }        
        
        //(a)遥控ON按钮按下进入，遥控状态
        if(REMOTE_SelectFlag)
        {
          AGV_RUN_Pro=AGV_STATUS_REMOTE;
          AGV_RUN_SUB_Pro=0;
          break;
        }
        
        //(b)电量大于10%，进入[空闲模式]
        if(BatteryVolt_LowFlag<2)
        {
          AGV_Delay=1500;
          LED_DISPLAY_Reset();
          AGV_RUN_Pro=AGV_STATUS_IDLE;
          AGV_RUN_SUB_Pro=0;  
          break;
        }

        //(c)上位机启动信号&&霍尔检测到磁条
        if((StartAutoChargeFlag!=0)
           &&(ON_LINE_Flag)
             &&(AGV_Delay==0))
        {
          AGV_RUN_Pro=AGV_STATUS_FOLLOWLINE;
          AGV_RUN_SUB_Pro=0;
          break;
        }
        
        //(d)用户控制模式
        if((MOD_BUS_Reg.M_CONTROL_MODE==M_CONTROL_MODE_SPEED)
          ||((MOD_BUS_Reg.M_CONTROL_MODE==M_CONTROL_MODE_DISPLACEMENT)&&(DISPLACEMENT_MOVEMENT_OPTION_List.In_index!=DISPLACEMENT_MOVEMENT_OPTION_List.Out_index))
          ||((MOD_BUS_Reg.M_CONTROL_MODE==M_CONTROL_MODE_ANGLE)&&(ANGLE_MOVEMENT_OPTION_List.In_index!=ANGLE_MOVEMENT_OPTION_List.Out_index))
            )
        {
          AGV_RUN_Pro=AGV_STATUS_USER_PROGRAM;
          AGV_RUN_SUB_Pro=0;
          break;
        }        
      }
    }
    break;
  /*----------臂章模式---------------*/     
  case AGV_STATUS_BARRIER:
    {
      if(AGV_RUN_SUB_Pro==0)
      {
#if (PROGRAM_PRINTF_DEBUG)
        printf("BARRIER\n");
#endif
        SetBeep(3,200,500);
        MOTO_IM_STOP();
        LED_DISPLAY_Reset();
        AGV_RUN_SUB_Pro++;
      }
      else
      {
        LED_BARRIER_Display(500);
        SetBeepForever(300,700);//
        
        if(SYS_ON_Flag==0)
        {
          AGV_RUN_Pro=AGV_STATUS_INIT;
          AGV_RUN_SUB_Pro=0;
        }        
        
        //(a)遥控ON按钮按下进入，遥控状态
        if(REMOTE_SelectFlag)
        {
          AGV_RUN_Pro=AGV_STATUS_REMOTE;
          AGV_RUN_SUB_Pro=0;
          break;
        } 
        
        //(b)急停按钮按下，进入急停模式
        if(BUTTON_IM_STOP_Flag)
        {
          AGV_RUN_Pro=AGV_STATUS_IM_STOP;
          AGV_RUN_SUB_Pro=0;    
          break;
        }        
        
        if(Software_IM_STOP)
        {
          AGV_RUN_Pro=AGV_STATUS_IM_STOP_SOFT;
          AGV_RUN_SUB_Pro=0;  
          break;
        }         
        
        //(c)没有障碍物后，进入空闲状态
        if((BARRIER_Flag & BARRIER_SENSOR_MASK) || (SuperMode != 0))
        {
          AGV_Delay=2400;
          LED_DISPLAY_Reset();
          AGV_RUN_Pro=AGV_STATUS_IDLE;
          AGV_RUN_SUB_Pro=0;    
          break;
        }
      }
    }
    break;
  case AGV_STATUS_IM_STOP_SOFT:  
    {
      if(AGV_RUN_SUB_Pro==0)
      {
#if (PROGRAM_PRINTF_DEBUG)
        printf("SOFT IM STOP\n");
#endif
        //SetBeep(1,1000,200);
        MOTO_IM_STOP();
        LED_DISPLAY_Reset();
        MONITOR_STATUS_Init();
        AGV_RUN_SUB_Pro++;
      }
      else
      {
        LED_IM_STOP_Display(500);
       
        
        //(a)急停按钮松开，进入空闲模式
        if(Software_IM_STOP==0)
        {
          //SetBeep(3,300,500);
          AGV_Delay=500;
          LED_DISPLAY_Reset();
          AGV_RUN_Pro=AGV_STATUS_IDLE;
          AGV_RUN_SUB_Pro=0;    
          //清除位移和角度控制
          memset(&U_M_CONTROL_Op,0,sizeof(U_M_CONTROL_OPTION));
          ClearMovementList(&DISPLACEMENT_MOVEMENT_OPTION_List);
          ClearMovementList(&ANGLE_MOVEMENT_OPTION_List);     
          ResetMotoSpeedUpDownTime();
          break;
        }
        
        BATT_LOW_LEVEL_1_Warning(); 
      }
    }
    break;    
  /*----------紧急停止模式---------------*/   
  case AGV_STATUS_IM_STOP:
    {
      if(AGV_RUN_SUB_Pro==0)
      {
#if (PROGRAM_PRINTF_DEBUG)
        printf("IM STOP\n");
#endif
        SetBeep(1,1000,200);
        MOTO_IM_STOP();
        LED_DISPLAY_Reset();
        MONITOR_STATUS_Init();
        AGV_RUN_SUB_Pro++;
      }
      else
      {
        LED_IM_STOP_Display(500);
       
        
        //(a)急停按钮松开，进入空闲模式
        if(BUTTON_IM_STOP_Flag==0)
        {
          SetBeep(3,300,500);
          AGV_Delay=2400;
          LED_DISPLAY_Reset();
          AGV_RUN_Pro=AGV_STATUS_IDLE;
          AGV_RUN_SUB_Pro=0;    
          //清除位移和角度控制
          memset(&U_M_CONTROL_Op,0,sizeof(U_M_CONTROL_OPTION));
          ClearMovementList(&DISPLACEMENT_MOVEMENT_OPTION_List);
          ClearMovementList(&ANGLE_MOVEMENT_OPTION_List);     
          ResetMotoSpeedUpDownTime();
          break;
        }
        
        BATT_LOW_LEVEL_1_Warning(); 
      }
    }
    break;
  /*----------用户控制模式---------------*/   
  case AGV_STATUS_USER_PROGRAM:
    {
      static u8 reset=0;
      if(AGV_RUN_SUB_Pro==0)
      {
#if (PROGRAM_PRINTF_DEBUG)
        printf("USER PROGRAM\n");
#endif
        AGV_Delay=5000;
        MODE_BUS_HALL_Addr=DEFAULT_MODE_BUS_HALL_ADDR;        
        LED_DISPLAY_Reset();
        reset=1;
        AGV_RUN_SUB_Pro+=1;
      }
      else
      {
        if(MOD_BUS_Reg.M_CONTROL_MODE==M_CONTROL_MODE_SPEED)
        {//速度模式
          LED_FOLLOW_LINE_Display(250);
          AGV_USER_PROGRAM_IN_SPEED_Task(&reset);
        }
        else if(MOD_BUS_Reg.M_CONTROL_MODE==M_CONTROL_MODE_DISPLACEMENT)
        {//位移模式
          LED_FOLLOW_LINE_Display(500);
          AGV_USER_PROGRAM_IN_DISPLACEMENT_Task(&reset);
        }
        else if(MOD_BUS_Reg.M_CONTROL_MODE==M_CONTROL_MODE_ANGLE)
        {//角度模式
          LED_FOLLOW_LINE_Display(1000);
          AGV_USER_PROGRAM_IN_DISPLACEMENT_Task(&reset);
        }
        else if(MOD_BUS_Reg.M_CONTROL_MODE==M_CONTROL_MODE_SPEC)
        {
          LED_FOLLOW_LINE_Display(2000);
          AGV_USER_PROGRAM_IN_SPEC_Task(&reset);
        }
        else
        {
          LED_FOLLOW_LINE_Display(100);
        }
        
        if(SYS_ON_Flag==0)
        {
          AGV_RUN_Pro=AGV_STATUS_INIT;
          AGV_RUN_SUB_Pro=0;
        }        
        
        //(a)急停按钮按下，进入急停模式
        if(BUTTON_IM_STOP_Flag)
        {
          AGV_RUN_Pro=AGV_STATUS_IM_STOP;
          AGV_RUN_SUB_Pro=0;  
          break;
        }
        
        if(Software_IM_STOP)
        {
          AGV_RUN_Pro=AGV_STATUS_IM_STOP_SOFT;
          AGV_RUN_SUB_Pro=0;  
          break;
        }        
        
        //(b)遥控ON按钮按下进入，遥控状态
        if(REMOTE_SelectFlag)
        {
          AGV_RUN_Pro=AGV_STATUS_REMOTE;
          AGV_RUN_SUB_Pro=0;
          break;
        }          
         
        //(c)光电/触碰触发，进入[蔽障模式]
        if((BARRIER_Flag & BARRIER_SENSOR_MASK) && (SuperMode == 0))
        {
          AGV_RUN_Pro=AGV_STATUS_BARRIER;
          AGV_RUN_SUB_Pro=0;    
          break;
        }        
        
        //(d)上位机启动信号&&霍尔检测到磁条
        if((StartAutoChargeFlag!=0)
           &&(ON_LINE_Flag)
             &&(AGV_Delay==0))
        {
          AGV_RUN_Pro=AGV_STATUS_FOLLOWLINE;
          AGV_RUN_SUB_Pro=0;
          break;
        }
        
        //(g)沿磁条退出充电坞
        if(StartLeaveChargeHouse)
        {
          StartLeaveChargeHouse=0;  
          AGV_RUN_Pro=AGV_STATUS_FOLLOWLINE_OUT;
          AGV_RUN_SUB_Pro=0;
          break;          
        }
        
        // test only
        /*
        if(StartAutoChargeFlag!=0)
        {
          //关闭巡线传感器电源
          Relay_status&=~0x10;
          SetRelay(RELAY_PGV_POWER_Index,RELAY_OFF);          
          MOTO_IM_STOP();
          RFID_STOP_ANGIN_Timeout=5000;
          AGV_Delay=1500;
          AGV_RUN_Pro=AGV_STATUS_CHARGE;
          AGV_RUN_SUB_Pro=0;  
          break;
        }           
        */
        
        BATT_LOW_LEVEL_1_Warning(); 
      }
    }
    break;
  case AGV_STATUS_CHARGE: //充电
    {
      if(AGV_RUN_SUB_Pro==0)
      {
#if (PROGRAM_PRINTF_DEBUG)
        printf("CHARGE MODE\n");
#endif                
        LED_DISPLAY_Reset();
        AGV_RUN_SUB_Pro+=1;
        AGV_Delay=2500;
      }
      else
      {
        if(SYS_ON_Flag==0)
        {
          AGV_RUN_Pro=AGV_STATUS_INIT;
          AGV_RUN_SUB_Pro=0;
        }
        
        switch(AGV_RUN_SUB_Pro)
        {
        case 1:
          {
            if(AGV_Delay==0)
            {
#if (PROGRAM_PRINTF_DEBUG)
        printf("RELAY 1 ON \n");
#endif                  
              //打开正极继电器，待续
              //SetRelay(RELAY_CHARGE_P_Index,RELAY_ON);
              AGV_Delay=1000;
              AGV_RUN_SUB_Pro+=1;
            }
          }
          break;
        case 2:
          {
            if(AGV_Delay==0)
            {
#if (PROGRAM_PRINTF_DEBUG)
        printf("RELAY 2 ON \n");
#endif              
              //打开负极极继电器，待续
              //SetRelay(RELAY_CHARGE_N_Index,RELAY_ON);
              ChargeLongTimeout=DEFAULT_CHARGE_LONG_TIME_OUT;
              AGV_Delay=180000;//180s等待充电机开始充电
              AGV_RUN_SUB_Pro+=1;
            }
          }
          break;      
        case 3:
          { //检测电流,小于额定充电电流，结束充电
            if(AGV_Delay==0)
            {
              if(ChargeCurrentFixFlag==0)
              {
                AGV_Delay=1000;
                StartAutoChargeFlag=0;
                StopAutoChargeFlag=0;
                AGV_RUN_SUB_Pro+=1;
              }
            }
#if (PROGRAM_PRINTF_DEBUG)
            if(debug_show)
            {
              debug_show=0;
              printf("eAD = %d  \n",eCurrent_Filterd);
            }
#endif
            
            //上位机命令结束充电
            if((StopAutoChargeFlag)||(ChargeLongTimeout==0))
            {
              AGV_Delay=1000;
              StopAutoChargeFlag=0;
              AGV_RUN_SUB_Pro+=1;
            }
          }
          break;
        case 4:
          {
            if(AGV_Delay==0)
            {
#if (PROGRAM_PRINTF_DEBUG)
        printf("RELAY 1 2 OFF \n");
#endif              
              //断开充电继电器
              //SetRelay(RELAY_CHARGE_P_Index,RELAY_OFF);
              //SetRelay(RELAY_CHARGE_N_Index,RELAY_OFF);
              SetBeep(2,800,1200);
              AGV_Delay=4000;
              AGV_RUN_SUB_Pro=0;
              AGV_RUN_Pro = AGV_STATUS_IDLE;
            }        
          }
          break;
        }
      }
    }
    break;
  }
  //记录位移
  ROAD_RECORD_Task();
}

void AGV_USER_PROGRAM_IN_SPEED_Task(u8* pReset)
{
  if(*pReset)
  {
    *pReset=0;
    U_M_CONTROL_Op.M_CONTROL_OPTION.M_FreshFlag=1;
  }
  if(U_M_CONTROL_Op.M_CONTROL_OPTION.M_FreshFlag)
  {
    float left,right;
    U_M_CONTROL_Op.M_CONTROL_OPTION.M_FreshFlag=0;
    left=U_M_CONTROL_Op.M_CONTROL_OPTION.LeftSpeed;
    right=U_M_CONTROL_Op.M_CONTROL_OPTION.RightSpeed;
    left = roundf(left * COFF_MMS_TO_D1RPM);
    right = roundf(right * COFF_MMS_TO_D1RPM);
      
    SetD1Rpm(LEFT_MOTO_INDEX, (s16)left);
    SetD1Rpm(RIGHT_MOTO_INDEX, (s16)right); 
  }
}

void AGV_USER_PROGRAM_IN_DISPLACEMENT_Task(u8* pReset)
{
  if(*pReset)
  {
    *pReset=0;
  }
  //执行机构，处理最底层设置PWM  
  if(ProgramControlCycle==0)
  {
    ProgramControlCycle=DEFAULT_PROGRAM_CYCLE_IN_MS;
    if(SPEED_OPTION_List.InIndex!=SPEED_OPTION_List.OutIndex)
    {
      if(SPEED_OPTION_List.buf[SPEED_OPTION_List.OutIndex].repeat_times!=0)
      {
        s16 LP=SPEED_OPTION_List.buf[SPEED_OPTION_List.OutIndex].L_Speed;
        s16 RP=SPEED_OPTION_List.buf[SPEED_OPTION_List.OutIndex].R_Speed;
      
        SetD1Rpm(LEFT_MOTO_INDEX, (float)LP*0.001*(float)MAX_MOTO_SPEED_IN_D1RPM);
        SetD1Rpm(RIGHT_MOTO_INDEX, (float)RP*0.001*(float)MAX_MOTO_SPEED_IN_D1RPM);

        SPEED_OPTION_List.buf[SPEED_OPTION_List.OutIndex].repeat_times-=1;
        if(SPEED_OPTION_List.buf[SPEED_OPTION_List.OutIndex].repeat_times==0)
        {
          SPEED_OPTION_List.OutIndex+=1;
        }
        //角度和位移模式更新相关，20160921  
        {
          u32 abs_speed0,abs_speed1;
          abs_speed0=abs(LP);
          abs_speed1=abs(RP);
          if(abs_speed0<abs_speed1) abs_speed0=abs_speed1;
          
          Finish_Disp_or_Angle_value+=(abs_speed0*MAX_WHEEL_RUN_LENGTH_IN_CM_PER_10MS/FULL_SPEED_STEP);
        }
      }
      else
      {
        SPEED_OPTION_List.OutIndex+=1;        
      }
    }
    else
    {//执行完毕
      //角度和位移模式更新相关，20160921  
      if(Wonder_Disp_or_Angle_value>(u32)Finish_Disp_or_Angle_value)
      {
        Finish_Disp_or_Angle_value=Wonder_Disp_or_Angle_value;
      }   
    }
  }
  
  //计算位移编程的执行流程
  if(SPEED_OPTION_List.InIndex==SPEED_OPTION_List.OutIndex)
  {
    if(DISPLACEMENT_MOVEMENT_OPTION_List.In_index!=DISPLACEMENT_MOVEMENT_OPTION_List.Out_index)
    {
      MOVEMENT_OPTION* pM=&DISPLACEMENT_MOVEMENT_OPTION_List.buf[DISPLACEMENT_MOVEMENT_OPTION_List.Out_index&LIST_LENGTH_MASK];
      Caculate_DisplacmentProcess(pM,&SPEED_OPTION_List,1,SPEED_UP_OPTION_List[DirectRun]);
      DISPLACEMENT_MOVEMENT_OPTION_List.Out_index+=1;
    }
  }
  
  //计算角度编程的执行流程
  if(SPEED_OPTION_List.InIndex==SPEED_OPTION_List.OutIndex)
  {
    if(ANGLE_MOVEMENT_OPTION_List.In_index!=ANGLE_MOVEMENT_OPTION_List.Out_index)
    {
      MOVEMENT_OPTION* pM=&ANGLE_MOVEMENT_OPTION_List.buf[ANGLE_MOVEMENT_OPTION_List.Out_index&LIST_LENGTH_MASK];
      Caculate_AngleProcess(
           pM,&SPEED_OPTION_List);    
      ANGLE_MOVEMENT_OPTION_List.Out_index+=1;
    }
  }   
}

RECIPROCATE_OPTION RECIPROCATE_Op = 
{
  .run_speed = 500,//500mm/s
  .forth_back_displacement = 2000, //5000
  .turn_angle = 180,
  .wonder_reciprocate_times = 8,
  .current_reciprocate_times = 0,
};
void AGV_USER_PROGRAM_IN_SPEC_Task(u8* pReset)
{
  static u8 process = 0;
  static u8 disp_reset = 0;
  static u32 NumOfSysTickIntBk;
  static u32 DelayMs;
  static u32 angle_start,angle_end;
  static u16 IMU_bk;
  if(*pReset)
  {
    *pReset=0;
    process = 0;
  }
  switch(process)
  {
  case 0:
    {
      if(RECIPROCATE_Op.current_reciprocate_times != RECIPROCATE_Op.wonder_reciprocate_times)
      {
        //RECIPROCATE_Op.run_speed = 500;//500mm/s
        //RECIPROCATE_Op.forth_back_displacement = 5000;
        //RECIPROCATE_Op.turn_angle = 180;
        //RECIPROCATE_Op.wonder_reciprocate_times = 10;
        //RECIPROCATE_Op.current_reciprocate_times = 0;
        process += 1;
      }
    }
    break;
  case 1: // forth
    {
      disp_reset = 1;
      MOVEMENT_OPTION* pM=
            &DISPLACEMENT_MOVEMENT_OPTION_List.buf[DISPLACEMENT_MOVEMENT_OPTION_List.In_index&LIST_LENGTH_MASK];
      pM->dir = 0;
      pM->value = abs(RECIPROCATE_Op.forth_back_displacement) / 10;
      DISPLACEMENT_MOVEMENT_OPTION_List.In_index += 1;   
      process += 1;
    }
    break;
  case 2:
    {
      AGV_USER_PROGRAM_IN_DISPLACEMENT_Task(&disp_reset);
      if(SPEED_OPTION_List.InIndex == SPEED_OPTION_List.OutIndex)
      {
        process += 1;
        DelayMs = 1000;
      }
    }
    break;
  case 3: 
    {
      AGV_USER_PROGRAM_IN_DISPLACEMENT_Task(&disp_reset);
      if(NumOfSysTickIntBk != NumOfSysTickInt)
      {
        NumOfSysTickIntBk = NumOfSysTickInt;
        DelayMs -= 1;
        if(DelayMs == 0)  process += 1;
      }
    }
    break;
  case 4: // turn
    {
      disp_reset = 1;
      MOVEMENT_OPTION* pM=
            &ANGLE_MOVEMENT_OPTION_List.buf[ANGLE_MOVEMENT_OPTION_List.In_index&LIST_LENGTH_MASK];
      pM->dir = 0;
      pM->value = RECIPROCATE_Op.turn_angle;
      ANGLE_MOVEMENT_OPTION_List.In_index += 1;
      
      angle_start = IMU_Angle;
      angle_end = angle_start + RECIPROCATE_Op.turn_angle * 100;
      angle_end %= 36000; 
      process += 1;
    }
    break;
  case 5:
    {
      AGV_USER_PROGRAM_IN_DISPLACEMENT_Task(&disp_reset);
      if(SPEED_OPTION_List.InIndex == SPEED_OPTION_List.OutIndex)
      {
        process += 1;
        DelayMs = 1000;
      }
      if(IMU_bk != IMU_Angle)
      {
        u16 min,max,flag=0;
#define STOP_RANGE 200 //+-2度误差
        IMU_bk = IMU_Angle;
        min = (36000 + angle_end - 200) % 36000;
        max = ( angle_end + 200 ) % 36000;
        if(min>max)
        {
          if((IMU_bk>min)||(IMU_bk<max)) flag = 1;
        }
        else
        {
          if((IMU_bk>min)&&(IMU_bk<max)) flag = 1;
        }
        if(flag)
        {
          SPEED_OPTION_List.OutIndex = SPEED_OPTION_List.InIndex;
          MOTO_IM_STOP();
          process += 1;
          DelayMs = 1000;
        }
      }
    }
    break;
  case 6: 
    {
      AGV_USER_PROGRAM_IN_DISPLACEMENT_Task(&disp_reset);
      if(NumOfSysTickIntBk != NumOfSysTickInt)
      {
        NumOfSysTickIntBk = NumOfSysTickInt;
        DelayMs -= 1;
        if(DelayMs == 0)  
        {
          RECIPROCATE_Op.current_reciprocate_times += 1;
          if(RECIPROCATE_Op.current_reciprocate_times != RECIPROCATE_Op.wonder_reciprocate_times)
          {
            process = 1;
          }
          else
          {
            process = 0;
          }
        }
      }
    }
    break;    
  }
}


// 计算车体位移流程的加减速过程
void Caculate_DisplacmentProcess(MOVEMENT_OPTION* pM,SPEED_OPTION_LIST* pS,u8 coff_enable,SPEED_UP_OPTION* pSPEED)
{
  u32 total_displacement=pM->value;
  u32 dir=pM->dir;
  u32 index=0,i;
  //角度和位移模式更新相关，20160921
  Wonder_Disp_or_Angle_value=total_displacement;//
  Finish_Disp_or_Angle_value=0;  
  
  if(coff_enable)
  {
    //位移补偿，20160813
    total_displacement=(u32)(((float)total_displacement)*Displacement_coff);
  }
  if(total_displacement<(pSPEED[0].total_disp*2))
  {
	;
  }
  else if(total_displacement>(pSPEED[SPEED_UP_Length-2].total_disp*2))
  {//加速+匀速+减速
    for(i=0;i<(SPEED_UP_Length-1);i++)
    {
      pS->buf[index].L_Speed=speed_to_pwm(dir?-pSPEED[i].current_speed:pSPEED[i].current_speed);//cm/s->pwm_persent
      pS->buf[index].R_Speed=pS->buf[index].L_Speed;
      pS->buf[index].repeat_times=100/DEFAULT_PROGRAM_CYCLE_IN_MS;
      index+=1;
    }
    {
      pS->buf[index].L_Speed=speed_to_pwm(dir?-pSPEED[SPEED_UP_Length-1].current_speed:pSPEED[SPEED_UP_Length-1].current_speed);;
      pS->buf[index].R_Speed=pS->buf[index].L_Speed;
      pS->buf[index].repeat_times=(total_displacement-(pSPEED[SPEED_UP_Length-2].total_disp*2))*(1000/DEFAULT_PROGRAM_CYCLE_IN_MS)/pSPEED[SPEED_UP_Length-1].current_speed;
      index+=1;    
    }
    for(i=0;i<(SPEED_UP_Length-1);i++)
    {
      pS->buf[index].L_Speed=speed_to_pwm(dir?-pSPEED[(SPEED_UP_Length-2)-i].current_speed:pSPEED[(SPEED_UP_Length-2)-i].current_speed);//cm/s->pwm_persent
      pS->buf[index].R_Speed=pS->buf[index].L_Speed;
      pS->buf[index].repeat_times=100/DEFAULT_PROGRAM_CYCLE_IN_MS;
      index+=1;
    }    
  }
  else
  {//加速-减速
    u32 speed_up_step;
    for(i=1;i<SPEED_UP_Length;i++)
    {
      if(total_displacement<(pSPEED[i].total_disp*2)) break;
    }
    speed_up_step=i;
    
    for(i=0;i<speed_up_step;i++)
    {
      pS->buf[index].L_Speed=speed_to_pwm(dir?-pSPEED[i].current_speed:pSPEED[i].current_speed);//cm/s->pwm_persent
      pS->buf[index].R_Speed=pS->buf[index].L_Speed;
      pS->buf[index].repeat_times=100/DEFAULT_PROGRAM_CYCLE_IN_MS;
      index+=1;
    }
    
    {
      pS->buf[index].L_Speed=speed_to_pwm(dir?-pSPEED[speed_up_step-1].current_speed:pSPEED[speed_up_step-1].current_speed);;
      pS->buf[index].R_Speed=pS->buf[index].L_Speed;
      pS->buf[index].repeat_times=(total_displacement-(pSPEED[speed_up_step-1].total_disp*2))*(1000/DEFAULT_PROGRAM_CYCLE_IN_MS)/pSPEED[speed_up_step-1].current_speed;
      index+=1;    
    }

    for(i=0;i<speed_up_step;i++)
    {
      pS->buf[index].L_Speed=speed_to_pwm(dir?-pSPEED[(speed_up_step-1)-i].current_speed:pSPEED[(speed_up_step-1)-i].current_speed);//cm/s->pwm_persent
      pS->buf[index].R_Speed=pS->buf[index].L_Speed;
      pS->buf[index].repeat_times=100/DEFAULT_PROGRAM_CYCLE_IN_MS;
      index+=1;
    }
  
  }

  {
    pS->buf[index].L_Speed=0;		
    pS->buf[index].R_Speed=pS->buf[index].L_Speed;
    pS->buf[index].repeat_times=10;
    index+=1;
  }
    
  pS->InIndex=index;
  pS->OutIndex=0;
}

s16 speed_to_pwm(float speed_in_cmps)
{
  return (s16)(speed_in_cmps*FULL_SPEED_STEP/MAX_WHEEL_RUN_LENGTH_IN_CM_PER_SECOND);
}

//计算车体位移流程的加减速过程
void Caculate_AngleProcess(MOVEMENT_OPTION* pM,SPEED_OPTION_LIST* pS)
{
  u8 i;
  u32 total_angle=pM->value;
  u32 dir=pM->dir;  
  MOVEMENT_OPTION temp;
  float disp;
  disp=(3.14*VehicleWidth*total_angle*Angle_coff)/360;//180
  temp.dir=0;
  temp.value=disp;
  
  Caculate_DisplacmentProcess(&temp,pS,0,SPEED_UP_OPTION_List[CircleRun]);
#if (1)
  if(dir==0)//方向：逆时针
  {
    for(i=0;i<pS->InIndex;i++)
    {
      pS->buf[i].L_Speed=-pS->buf[i].L_Speed;
    }
  }
  else //顺时针
  {
    for(i=0;i<pS->InIndex;i++) 
    {
      pS->buf[i].R_Speed=-pS->buf[i].R_Speed;
    }
  }
#endif
/*
#if (WHEEL_TYPE == WHEEL_TYPE_NOMAL) 
  if((dir&0xFF)==0)//方向：逆时针
  {
    switch(dir>>8)//模式
    {
    case 1:
      {//1-逆时针，左轮反转
        for(i=0;i<pS->InIndex;i++)
        {
          pS->buf[i].L_Speed=-pS->buf[i].L_Speed,pS->buf[i].R_Speed=0;
        }
      }
      break;
    case 2:
      {//2-逆时针，右轮正转
        for(i=0;i<pS->InIndex;i++) 
          pS->buf[i].L_Speed=0;
      }
      break;
    default:
      {
        for(i=0;i<pS->InIndex;i++) 
          pS->buf[i].L_Speed=0;
      }
    }
    
  }
  else//顺时针
  {
    switch(dir>>8)//模式
    {
    case 1:
      {//1-顺时针，左轮正转
        for(i=0;i<pS->InIndex;i++) 
          pS->buf[i].R_Speed=0;
      }
      break;
    case 2:
      {//2-顺时针，右轮反转
        for(i=0;i<pS->InIndex;i++) 
          pS->buf[i].L_Speed=0,pS->buf[i].R_Speed=-pS->buf[i].R_Speed;
      }
      break;
    default:
      {
        for(i=0;i<pS->InIndex;i++) 
          pS->buf[i].R_Speed=0;
      }
    }    
  }
#endif
*/
}


/************************************************************
 ** 功能：加减速过程初始化---
 ** 参数：accelerated_speed_cmps  - 加速度，单位cm/s
 **       max_speed_cmps          - 最大速度，单位cm/s
 **       cycle_time              - 电机的调整时间周期，单位s
*************************************************************/
void SPEED_UP_DOWN_STRUCT_Init(float accelerated_speed_cmps,float max_speed_cmps,float cycle_time,SPEED_UP_OPTION* pSPEED)
{
  int i;
  float speed=0;
  float time=0;
  float disp=0;
  for(i=0;i<MAX_SPEED_UP_LIST_LENGTH;i++)
  {
    time+=cycle_time;
    speed+=(accelerated_speed_cmps*cycle_time);
    disp+=cycle_time*speed;
    pSPEED[i].total_time=time;
    pSPEED[i].current_speed=speed;
    pSPEED[i].total_disp=disp;
    if(speed>=max_speed_cmps) 
    {
      i+=1;
      break;
    }
  }
  SPEED_UP_Length=i;
}

// 电池电量低报警：每隔10分钟，蜂鸣器响5声，频率1HZ
void BATT_LOW_LEVEL_1_Warning(void)
{
  static u32 counter=0;
  static u32 NumOfSysTickIntBk;
  if(NumOfSysTickInt!=NumOfSysTickIntBk)
  {
    NumOfSysTickIntBk=NumOfSysTickInt;
  }
  else return;  
  
  if(BatteryVolt_LowFlag==1)
  {
    counter++;
    if(counter>=600000)//10分钟
    {
      counter=0;
      SetBeep(5,300,700);//响5声
    }
  }
  else
  {
    counter=0;
  }
}