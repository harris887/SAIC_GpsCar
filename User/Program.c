#include "user_inc.h"
#include "string.h"
#include "stdlib.h"

#define PROGRAM_PRINTF_DEBUG          1

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
u16 SuperMode=0;//��ײ������λ������

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
  /*----------��ʼ��ģʽ---------------*/    
  case AGV_STATUS_INIT:
    {
      if(AGV_RUN_SUB_Pro==0)
      {//��λ�����ⲿ
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
  /*----------����ģʽ---------------*/    
  case AGV_STATUS_IDLE:
    {
      if(AGV_RUN_SUB_Pro==0)
      {
        if(AGV_Delay==0)
        {
#if (PROGRAM_PRINTF_DEBUG)
          printf("IDLE\n");
#endif
          SET_DIDO_Relay(DIDO_BREAK_0, BREAK_OFF);
          SET_DIDO_Relay(DIDO_BREAK_1, BREAK_OFF);
          SET_DIDO_Relay(DIDO_BREAK_2, BREAK_OFF);
          SET_DIDO_Relay(DIDO_BREAK_3, BREAK_OFF);
          SET_DIDO_Relay(DIDO_MOTO_EN_0, DIDO_MOTO_ON);
          SET_DIDO_Relay(DIDO_MOTO_EN_1, DIDO_MOTO_ON);
          SET_DIDO_Relay(DIDO_MOTO_EN_2, DIDO_MOTO_ON);
          SET_DIDO_Relay(DIDO_MOTO_EN_3, DIDO_MOTO_ON);          
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
        
        //(a)��ͣ��ť���£����뼱ͣģʽ
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
        
        //(b)���/�����������������ģʽ
        if((BARRIER_Flag & BARRIER_SENSOR_MASK) && (SuperMode == 0))
        {
          AGV_RUN_Pro=AGV_STATUS_BARRIER;
          AGV_RUN_SUB_Pro=0;    
          break;
        }          
        
        //(c)��λ�������ź�&&������⵽����
        if((StartAutoChargeFlag!=0)
           &&(ON_LINE_Flag)
             &&(AGV_Delay==0))
        {
          AGV_RUN_Pro=AGV_STATUS_FOLLOWLINE;
          AGV_RUN_SUB_Pro=0;
          break;
        }
           
        //(d)�û�����ģʽ
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
        
        //(e)ң��ON��ť���½��룬ң��״̬
        if(REMOTE_SelectFlag)
        {
          AGV_RUN_Pro=AGV_STATUS_REMOTE;
          AGV_RUN_SUB_Pro=0;
          break;
        }

        //(f)�͵�ѹ״̬�л�
        if(BatteryVolt_LowFlag>=2)
        {
          AGV_RUN_Pro=AGV_STATUS_LOW_POWER;
          AGV_RUN_SUB_Pro=0;
          break;
        }

        //(g)�ش����˳������
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
  /*----------Ѳ��ģʽ(ǰ��ȥ���)---------------*/      
  case AGV_STATUS_FOLLOWLINE:
    {
    }
    break;
    /*----------Ѳ��ģʽ (�Զ��˳��������)---------------*/  
  case AGV_STATUS_FOLLOWLINE_OUT:
    {
    }
    break;
    /*Ѳ���У�����������ģʽ*/
  case AGV_STATUS_OFF_LINE:
    {
    }
    break;
  /*----------RFID�ȴ�ģʽ----------*/  
  case AGV_STATUS_RFID_COMEIN:
    {
    }
    break;
  /*----------ң��ģʽ---------------*/    
  case AGV_STATUS_REMOTE:
    {
      if(AGV_RUN_SUB_Pro==0)
      {
#if (PROGRAM_PRINTF_DEBUG)
        printf("REMOTE MODE\n");
#endif
        AGV_Delay = 1000;
        SET_DIDO_Relay(DIDO_BREAK_0, BREAK_OFF);
        SET_DIDO_Relay(DIDO_BREAK_1, BREAK_OFF);
        SET_DIDO_Relay(DIDO_BREAK_2, BREAK_OFF);
        SET_DIDO_Relay(DIDO_BREAK_3, BREAK_OFF);
        SET_DIDO_Relay(DIDO_MOTO_EN_0, DIDO_MOTO_ON);
        SET_DIDO_Relay(DIDO_MOTO_EN_1, DIDO_MOTO_ON);
        SET_DIDO_Relay(DIDO_MOTO_EN_2, DIDO_MOTO_ON);
        SET_DIDO_Relay(DIDO_MOTO_EN_3, DIDO_MOTO_ON);
        
        
        LED_DISPLAY_Reset();
        AGV_RUN_SUB_Pro+=1;
      }
      else
      {
        //��Ӧҡ��
        if(AGV_Delay == 0)
        {
          REMOTE_Task();
        }
        
        if(SYS_ON_Flag==0)
        {
          AGV_RUN_Pro=AGV_STATUS_INIT;
          AGV_RUN_SUB_Pro=0;
        }        
        
        //(a)��ͣ��ť���£����뼱ͣģʽ
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
        
        //(b)�������ģʽ
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
  /*----------�͵�ѹģʽ---------------*/     
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
        //4��ָʾ��1HZ����
        //LED_LOW_POWER_Display(500);
        
        //������1HZ����
        SetBeepForever(300,700);
        
        if(SYS_ON_Flag==0)
        {
          AGV_RUN_Pro=AGV_STATUS_INIT;
          AGV_RUN_SUB_Pro=0;
        }        
        
        //(a)ң��ON��ť���½��룬ң��״̬
        if(REMOTE_SelectFlag)
        {
          AGV_RUN_Pro=AGV_STATUS_REMOTE;
          AGV_RUN_SUB_Pro=0;
          break;
        }
        
        //(b)��������10%������[����ģʽ]
        if(BatteryVolt_LowFlag<2)
        {
          AGV_Delay=1500;
          LED_DISPLAY_Reset();
          AGV_RUN_Pro=AGV_STATUS_IDLE;
          AGV_RUN_SUB_Pro=0;  
          break;
        }

        //(c)��λ�������ź�&&������⵽����
        if((StartAutoChargeFlag!=0)
           &&(ON_LINE_Flag)
             &&(AGV_Delay==0))
        {
          AGV_RUN_Pro=AGV_STATUS_FOLLOWLINE;
          AGV_RUN_SUB_Pro=0;
          break;
        }
        
        //(d)�û�����ģʽ
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
  /*----------����ģʽ---------------*/     
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
        
        //(a)ң��ON��ť���½��룬ң��״̬
        if(REMOTE_SelectFlag)
        {
          AGV_RUN_Pro=AGV_STATUS_REMOTE;
          AGV_RUN_SUB_Pro=0;
          break;
        } 
        
        //(b)��ͣ��ť���£����뼱ͣģʽ
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
        
        //(c)û���ϰ���󣬽������״̬
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
       
        
        //(a)��ͣ��ť�ɿ����������ģʽ
        if(Software_IM_STOP==0)
        {
          //SetBeep(3,300,500);
          AGV_Delay=500;
          LED_DISPLAY_Reset();
          AGV_RUN_Pro=AGV_STATUS_IDLE;
          AGV_RUN_SUB_Pro=0;    
          //���λ�ƺͽǶȿ���
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
  /*----------����ֹͣģʽ---------------*/   
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
       
        
        //(a)��ͣ��ť�ɿ����������ģʽ
        if(BUTTON_IM_STOP_Flag==0)
        {
          SetBeep(3,300,500);
          AGV_Delay=2400;
          LED_DISPLAY_Reset();
          AGV_RUN_Pro=AGV_STATUS_IDLE;
          AGV_RUN_SUB_Pro=0;    
          //���λ�ƺͽǶȿ���
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
  /*----------�û�����ģʽ---------------*/   
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
        {//�ٶ�ģʽ
          LED_FOLLOW_LINE_Display(250);
          AGV_USER_PROGRAM_IN_SPEED_Task(&reset);
        }
        else if(MOD_BUS_Reg.M_CONTROL_MODE==M_CONTROL_MODE_DISPLACEMENT)
        {//λ��ģʽ
          LED_FOLLOW_LINE_Display(500);
          AGV_USER_PROGRAM_IN_DISPLACEMENT_Task(&reset);
        }
        else if(MOD_BUS_Reg.M_CONTROL_MODE==M_CONTROL_MODE_ANGLE)
        {//�Ƕ�ģʽ
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
        
        //(a)��ͣ��ť���£����뼱ͣģʽ
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
        
        //(b)ң��ON��ť���½��룬ң��״̬
        if(REMOTE_SelectFlag)
        {
          AGV_RUN_Pro=AGV_STATUS_REMOTE;
          AGV_RUN_SUB_Pro=0;
          break;
        }          
         
        //(c)���/��������������[����ģʽ]
        if((BARRIER_Flag & BARRIER_SENSOR_MASK) && (SuperMode == 0))
        {
          AGV_RUN_Pro=AGV_STATUS_BARRIER;
          AGV_RUN_SUB_Pro=0;    
          break;
        }        
        
        BATT_LOW_LEVEL_1_Warning(); 
      }
    }
    break;
  case AGV_STATUS_CHARGE: //���
    {
    }
    break;
  }
  //��¼λ��
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
    SetD1Rpm(LEFT_2_MOTO_INDEX, (s16)left);
    SetD1Rpm(RIGHT_MOTO_INDEX, (s16)right); 
    SetD1Rpm(RIGHT_2_MOTO_INDEX, (s16)right);
  }
}

void AGV_USER_PROGRAM_IN_DISPLACEMENT_Task(u8* pReset)
{
  if(*pReset)
  {
    *pReset=0;
  }
  //ִ�л�����������ײ�����PWM  
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
        //�ǶȺ�λ��ģʽ������أ�20160921  
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
    {//ִ�����
      //�ǶȺ�λ��ģʽ������أ�20160921  
      if(Wonder_Disp_or_Angle_value>(u32)Finish_Disp_or_Angle_value)
      {
        Finish_Disp_or_Angle_value=Wonder_Disp_or_Angle_value;
      }   
    }
  }
  
  //����λ�Ʊ�̵�ִ������
  if(SPEED_OPTION_List.InIndex==SPEED_OPTION_List.OutIndex)
  {
    if(DISPLACEMENT_MOVEMENT_OPTION_List.In_index!=DISPLACEMENT_MOVEMENT_OPTION_List.Out_index)
    {
      MOVEMENT_OPTION* pM=&DISPLACEMENT_MOVEMENT_OPTION_List.buf[DISPLACEMENT_MOVEMENT_OPTION_List.Out_index&LIST_LENGTH_MASK];
      Caculate_DisplacmentProcess(pM,&SPEED_OPTION_List,1,SPEED_UP_OPTION_List[DirectRun]);
      DISPLACEMENT_MOVEMENT_OPTION_List.Out_index+=1;
    }
  }
  
  //����Ƕȱ�̵�ִ������
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
#define STOP_RANGE 200 //+-2�����
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


// ���㳵��λ�����̵ļӼ��ٹ���
void Caculate_DisplacmentProcess(MOVEMENT_OPTION* pM,SPEED_OPTION_LIST* pS,u8 coff_enable,SPEED_UP_OPTION* pSPEED)
{
  u32 total_displacement=pM->value;
  u32 dir=pM->dir;
  u32 index=0,i;
  //�ǶȺ�λ��ģʽ������أ�20160921
  Wonder_Disp_or_Angle_value=total_displacement;//
  Finish_Disp_or_Angle_value=0;  
  
  if(coff_enable)
  {
    //λ�Ʋ�����20160813
    total_displacement=(u32)(((float)total_displacement)*Displacement_coff);
  }
  if(total_displacement<(pSPEED[0].total_disp*2))
  {
	;
  }
  else if(total_displacement>(pSPEED[SPEED_UP_Length-2].total_disp*2))
  {//����+����+����
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
  {//����-����
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

//���㳵��λ�����̵ļӼ��ٹ���
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
  if(dir==0)//������ʱ��
  {
    for(i=0;i<pS->InIndex;i++)
    {
      pS->buf[i].L_Speed=-pS->buf[i].L_Speed;
    }
  }
  else //˳ʱ��
  {
    for(i=0;i<pS->InIndex;i++) 
    {
      pS->buf[i].R_Speed=-pS->buf[i].R_Speed;
    }
  }
#endif
/*
#if (WHEEL_TYPE == WHEEL_TYPE_NOMAL) 
  if((dir&0xFF)==0)//������ʱ��
  {
    switch(dir>>8)//ģʽ
    {
    case 1:
      {//1-��ʱ�룬���ַ�ת
        for(i=0;i<pS->InIndex;i++)
        {
          pS->buf[i].L_Speed=-pS->buf[i].L_Speed,pS->buf[i].R_Speed=0;
        }
      }
      break;
    case 2:
      {//2-��ʱ�룬������ת
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
  else//˳ʱ��
  {
    switch(dir>>8)//ģʽ
    {
    case 1:
      {//1-˳ʱ�룬������ת
        for(i=0;i<pS->InIndex;i++) 
          pS->buf[i].R_Speed=0;
      }
      break;
    case 2:
      {//2-˳ʱ�룬���ַ�ת
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
 ** ���ܣ��Ӽ��ٹ��̳�ʼ��---
 ** ������accelerated_speed_cmps  - ���ٶȣ���λcm/s
 **       max_speed_cmps          - ����ٶȣ���λcm/s
 **       cycle_time              - ����ĵ���ʱ�����ڣ���λs
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

// ��ص����ͱ�����ÿ��10���ӣ���������5����Ƶ��1HZ
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
    if(counter>=600000)//10����
    {
      counter=0;
      SetBeep(5,300,700);//��5��
    }
  }
  else
  {
    counter=0;
  }
}