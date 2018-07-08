#ifndef _moto_h_
#define _moto_h_

extern float max_wheel_speed;

#define WHEEL_SMALL     0
#define WHEEL_BIG       1
#define WHEEL_SELECT    WHEEL_SMALL   

/*电机_驱动器相关设置*/
#define FULL_SPEED_STEP 1000.0
#define ROAD_RECORD_ONCE_TIME_MS  100
#define MAX_WHEEL_RUN_LENGTH_IN_CM_PER_SECOND (max_wheel_speed*1.0)
#define MAX_WHEEL_RUN_LENGTH_IN_CM_PER_100MS  (max_wheel_speed*0.1)
#define MAX_WHEEL_RUN_LENGTH_IN_CM_PER_10MS  (max_wheel_speed*0.01)

#if (WHEEL_SELECT == WHEEL_SMALL)
#define DEFAULT_WHEEL_DIAMETER_IN_MM  121
#define MAX_MOTO_SPEED_IN_RPM     200 
#else
#define DEFAULT_WHEEL_DIAMETER_IN_MM  170 //big: 170, small: 121
#define MAX_MOTO_SPEED_IN_RPM     142 
#endif
#define WHEEL_DIAMETER_IN_CM    (MOD_BUS_Reg.WHEEL_DIAMETER_IN_MM * 0.1)//12.1  //25 20 31 22.5 ，20.3 
#define MAX_MOTO_SPEED_IN_D1RPM  (MAX_MOTO_SPEED_IN_RPM * 10)
#define SPEED_DOWN_RATIO        1.0        //电机齿轮箱减速比


//高电平使能，低电平去除falut
#define LEFT_MOTO_EN_PIN    GPIO_Pin_3
#define LEFT_MOTO_EN_PORT    GPIOA
#define RIGHT_MOTO_EN_PIN    GPIO_Pin_1
#define RIGHT_MOTO_EN_PORT    GPIOA
#define LEFT_2_MOTO_EN_PIN    GPIO_Pin_1
#define LEFT_2_MOTO_EN_PORT    GPIOB
#define RIGHT_2_MOTO_EN_PIN    GPIO_Pin_7
#define RIGHT_2_MOTO_EN_PORT    GPIOB

#define LEFT_MOTO_FAULT_PIN    GPIO_Pin_2
#define LEFT_MOTO_FAULT_PORT    GPIOA
#define RIGHT_MOTO_FAULT_PIN    GPIO_Pin_0
#define RIGHT_MOTO_FAULT_PORT    GPIOA
#define LEFT_2_MOTO_FAULT_PIN    GPIO_Pin_0
#define LEFT_2_MOTO_FAULT_PORT    GPIOB
#define RIGHT_2_MOTO_FAULT_PIN    GPIO_Pin_6
#define RIGHT_2_MOTO_FAULT_PORT    GPIOB

#define LEFT_MOTO_EN_High() LEFT_MOTO_EN_PORT->BSRR=LEFT_MOTO_EN_PIN
#define LEFT_MOTO_EN_Low() LEFT_MOTO_EN_PORT->BRR=LEFT_MOTO_EN_PIN
#define RIGHT_MOTO_EN_High() RIGHT_MOTO_EN_PORT->BSRR=RIGHT_MOTO_EN_PIN
#define RIGHT_MOTO_EN_Low() RIGHT_MOTO_EN_PORT->BRR=RIGHT_MOTO_EN_PIN
#define LEFT_2_MOTO_EN_High() LEFT_2_MOTO_EN_PORT->BSRR=LEFT_2_MOTO_EN_PIN
#define LEFT_2_MOTO_EN_Low() LEFT_2_MOTO_EN_PORT->BRR=LEFT_2_MOTO_EN_PIN
#define RIGHT_2_MOTO_EN_High() RIGHT_2_MOTO_EN_PORT->BSRR=RIGHT_2_MOTO_EN_PIN
#define RIGHT_2_MOTO_EN_Low() RIGHT_2_MOTO_EN_PORT->BRR=RIGHT_2_MOTO_EN_PIN


#define GET_LEFT_MOTO_Fault()  GPIO_ReadInputDataBit(LEFT_MOTO_FAULT_PORT, LEFT_MOTO_FAULT_PIN)
#define GET_RIGHT_MOTO_Fault()  GPIO_ReadInputDataBit(RIGHT_MOTO_FAULT_PORT, RIGHT_MOTO_FAULT_PIN)
#define GET_2_LEFT_MOTO_Fault()  GPIO_ReadInputDataBit(LEFT_2_MOTO_FAULT_PORT, LEFT_2_MOTO_FAULT_PIN)
#define GET_2_RIGHT_MOTO_Fault()  GPIO_ReadInputDataBit(RIGHT_2_MOTO_FAULT_PORT, RIGHT_2_MOTO_FAULT_PIN)
//pwm引脚
#define LEFT_MOTO_PWM_PIN     GPIO_Pin_6
#define RIGHT_MOTO_PWM_PIN    GPIO_Pin_7
#define LEFT_2_MOTO_PWM_PIN     GPIO_Pin_8
#define RIGHT_2_MOTO_PWM_PIN    GPIO_Pin_9

#define LEFT_MOTO_PWM_PORT    GPIOC
#define RIGHT_MOTO_PWM_PORT   GPIOC
#define LEFT_2_MOTO_PWM_PORT    GPIOC
#define RIGHT_2_MOTO_PWM_PORT   GPIOC

#define LEFT_MOTO_TIM_ENUM    TIM8
#define RIGHT_MOTO_TIM_ENUM   TIM8
#define LEFT_2_MOTO_TIM_ENUM    TIM8
#define RIGHT_2_MOTO_TIM_ENUM   TIM8

#define MAX_SPEED_STEP    1000
#define MAX_SPEED_IN_MMS  10000 //10000mm/S

typedef enum
{
  LEFT_MOTO_INDEX=0,
  RIGHT_MOTO_INDEX,
  MOTO_NUM,
  //LEFT_2_MOTO_INDEX,
  //RIGHT_2_MOTO_INDEX,
}MOTO_INDEX_ENUM;

#define PWM_CYCLE_COUNTER 10000
#define ZERO_SPEED_PWM_COUNTER  (PWM_CYCLE_COUNTER>>1)
//速度，模式，脉冲数


#define CONTROL_MODE_PROGRAM 0
#define CONTROL_MODE_REMOTE   1

#define WHEEL_L1_ENUM     0
#define WHEEL_L2_ENUM     1
#define WHEEL_R1_ENUM     2
#define WHEEL_R2_ENUM     3
#define WHEEL_RUN_DIR_POSTIVE 1
#define WHEEL_RUN_DIR_NEGTIVE (-1)

#define MOVE_MODE_NULL        0
#define MOVE_MODE_FORWARD     1
#define MOVE_MODE_BACKWARD    2
#define MOVE_MODE_LEFTWARD    3
#define MOVE_MODE_RIGHTWARD   4
#define MOVE_MODE_UP_ROLL     5
#define MOVE_MODE_DOWN_ROLL   6


#define PROGRAM_RUN_STATUS_IDEL   0
#define PROGRAM_RUN_STATUS_ING    1
#define PROGRAM_RUN_STATUS_FINISH 2
extern u8 PROGRAM_RUN_Status;//0-空闲，1-运行一次，2-运行完毕
extern u8 PROGRAM_RUN_Index;

//设置这个值会改变遥控的最大速度，10--1kHZ,5--2KHZ,1-10KHZ
//最小为1，最大请在100以内
#define REMOTE_MIN_PULSE_TIME 10
typedef struct
{
  u32 PulseNum;
  u32 PulseUpTime;
  u32 PulseDownTime;
  u32 PulseDelayCounter;
  u8  ActiveFlag;
  u8  ActivePro;
  u8  rev[2];
}MOTO_OPTION;

extern MOTO_OPTION* pMOTO_OPTION[4];
extern MOTO_OPTION  MOTO_OPTION_List[1][4];
//遥控时使用，不影响固定路线
extern MOTO_OPTION  REMOTE_MOTO_OPTION_List[4];
extern s16 RealRpm[MOTO_NUM];

void MOTO_Init(void);
void MOTO_TEST(void);
void PROGRAM_TASK(void);
void MOTO_IM_STOP(void);
void MOTO_FaultCheck_TASK(void);
void ROAD_RECORD_Task(void);

//============== 位移&角度模式抽象 ================== 
//暂定30%速度,有加减速过程。
#define MAX_SPEED_UP_STEP 32
#define MAX_SPEED_DOWN_STEP 32
#define MAX_SPEED_HOLD_STEP 32
#define MAX_SPEED_OPTION_NUM  (MAX_SPEED_UP_STEP+MAX_SPEED_HOLD_STEP+MAX_SPEED_DOWN_STEP)
typedef struct
{
  u32 repeat_times;//100ms的次数
  s16 L_Speed;
  s16 R_Speed;
}SPEED_OPTION;

typedef struct
{
  u16 InIndex;
  u16 OutIndex;
  SPEED_OPTION buf[MAX_SPEED_OPTION_NUM];  
}SPEED_OPTION_LIST;
extern SPEED_OPTION_LIST SPEED_OPTION_List;

typedef struct
{
  u16 FaultPro;
  u16 FaultStatus;
  u16 HoldTime;
  u16 FaultStatusChangFlag;
}MOTO_FAULT_OPTION;

extern s16 LeftRealSpeed;
extern u32 follow_speed_persent;
void SLOW_DOWN_Task(u8* reset,u16 time_in_ms);
void Moto_PowerFree_Task(void);
void MOTO_SPEED_CONTROL_TASK(void);

extern u16 MOTO_485COMM_Timeout;
#define MOTO_ZERO_FREE_TIME_IN_MS     3000  //电机停止后5s进入自由状态

extern s16 moto_speed_in_rpm[MOTO_NUM];
extern float RoadLength[MOTO_NUM];
//extern void SetRpm(MOTO_INDEX_ENUM MOTO_SELECT,s16 rpm);
extern void SetD1Rpm(MOTO_INDEX_ENUM MOTO_SELECT,s16 d01rpm);
extern u32 ReadMotoRpmTimes[MOTO_NUM];
extern u8 moto_enable_status[MOTO_NUM];
void ResetMotoSpeedUpDownTime(void);
#endif