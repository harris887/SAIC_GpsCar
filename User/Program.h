#ifndef program_h_
#define program_h_

#define DEFAULT_Vehicle_WIDTH_FLOAT   31.0    //车体宽度，51.0 ,56.0 , 45.0
#define DEFAULT_Vehicle_WIDTH_10_TIME 310   //510

#define DEFAULT_RFID_WAIT_TIME_IN_MS  3000 //3S  

typedef enum
{
  AGV_STATUS_INIT=0,
  AGV_STATUS_IDLE,
  AGV_STATUS_FOLLOWLINE,
  AGV_STATUS_REMOTE,
  AGV_STATUS_LOW_POWER,
  AGV_STATUS_IM_STOP,
  AGV_STATUS_BARRIER,
  AGV_STATUS_RFID_COMEIN,
  AGV_STATUS_USER_PROGRAM,
  AGV_STATUS_OFF_LINE,
  AGV_STATUS_CHARGE,
  AGV_STATUS_FOLLOWLINE_OUT,
  AGV_STATUS_IM_STOP_SOFT,
  
}AGV_STATUS_LIST;
extern AGV_STATUS_LIST AGV_RUN_Pro;

#define MAX_LIST_LENGTH 32
#define LIST_LENGTH_MASK  (MAX_LIST_LENGTH-1)
typedef struct
{
  u32 dir;
  u32 value;
}MOVEMENT_OPTION;

typedef struct
{
  u8 In_index;
  u8 Out_index;
  MOVEMENT_OPTION buf[MAX_LIST_LENGTH];
}MOVEMENT_OPTION_LIST;

extern u32 ChargeLongTimeout;
extern u32 AGV_Delay;
extern MOVEMENT_OPTION_LIST DISPLACEMENT_MOVEMENT_OPTION_List;
extern MOVEMENT_OPTION_LIST ANGLE_MOVEMENT_OPTION_List;
extern u16 ProgramControlCycle;

extern void AGV_RUN_Task(void);
extern u16 RFID_STOP_ANGIN_Timeout;
extern u16 StartAutoChargeFlag;
extern u16 StopAutoChargeFlag;
extern u16 StartLeaveChargeHouse;
extern void AGV_USER_PROGRAM_IN_SPEED_Task(u8* pReset);
extern float VehicleWidth;
extern float Displacement_coff;
extern float Angle_coff;
extern u32 Wonder_Disp_or_Angle_value;//
extern float Finish_Disp_or_Angle_value;
extern u16 Software_IM_STOP;

void BATT_LOW_LEVEL_1_Warning(void);
void MovementListInit(void);
void AGV_USER_PROGRAM_IN_DISPLACEMENT_Task(u8* pReset);
typedef enum
{
  DirectRun = 0,
  CircleRun,
  RunFuncNum,
}VEHICLE_RUN_FUNC;

typedef struct
{
  float total_time;//加速总时间
  float total_disp;//加速过程的位移
  float current_speed;//此时的速度
}SPEED_UP_OPTION;

#define DEFAULT_PROGRAM_CYCLE_IN_MS 10  
void Caculate_DisplacmentProcess(MOVEMENT_OPTION* pM,SPEED_OPTION_LIST* pS,u8 coff_enable,SPEED_UP_OPTION* pSPEED);
void Caculate_AngleProcess(MOVEMENT_OPTION* pM,SPEED_OPTION_LIST* pS);


#define MAX_SPEED_UP_LIST_LENGTH  64 //32
extern u16 SPEED_UP_Length;
extern SPEED_UP_OPTION SPEED_UP_OPTION_List[RunFuncNum][MAX_SPEED_UP_LIST_LENGTH];
extern void SPEED_UP_DOWN_STRUCT_Init(float accelerated_speed_cmps,float max_speed_cmps,float cycle_time,SPEED_UP_OPTION* pSPEED);
s16 speed_to_pwm(float speed_in_cmps);
extern u16 SuperMode;
extern u16 IMU_Angle;

typedef struct
{
  s32 run_speed;
  s32 forth_back_displacement;
  s32 turn_angle;
  s32 wonder_reciprocate_times;
  s32 current_reciprocate_times;
}RECIPROCATE_OPTION;
extern RECIPROCATE_OPTION RECIPROCATE_Op;
void AGV_USER_PROGRAM_IN_SPEC_Task(u8* pReset);
#endif