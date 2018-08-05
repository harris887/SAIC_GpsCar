#include "user_inc.h"

u16 PID_TimeOut;

PID_OPTION Pid={0,0,0,0,0,0,0,
  3.0,  //Kp
  0.1,  //Ki
  0.5   //Kd
  };


void PID_Init(void)
{
  Pid.Integral=0;
  PID_TimeOut=200;
}

#if (FOLLOW_LINE_SENSOR == HALL_SENSOR) 
#define MAX_INTERGE 100     //积分限定值
s32 SPEED_PID(u8 expect_mid,u8 current_mid)
{
  if((current_mid<MIN_SENSOR_VALUE)||(current_mid>MAX_SENSOR_VALUE)) return 0;//没有差速脉冲
  
  Pid.Setvalue=expect_mid;
  Pid.CurrentValue=current_mid;
  Pid.Err_A1=Pid.CurrentValue-Pid.Setvalue;
  Pid.Integral+=Pid.Err_A1;
  if(Pid.Integral>MAX_INTERGE) Pid.Integral=MAX_INTERGE;
  
  Pid.Diff=Pid.Err_A1-Pid.Err_A0;
  Pid.Err_A0=Pid.Err_A1;
  Pid.Out=(Pid.Kp*Pid.Err_A1)+(Pid.Ki*Pid.Integral)+(Pid.Kd*Pid.Diff); 
  
  return Pid.Out;
}
#else
#define MAX_PGV_Y_OFFSET        (60)
#define MIN_PGV_Y_OFFSET        (-60)
#define MAX_INTERGE             300     //积分限定值 ,600
s32 SPEED_PID(s32 expect_offset,s32 current_offset)
{
  if((current_offset<MIN_PGV_Y_OFFSET)||(current_offset>MAX_PGV_Y_OFFSET)) return 0;
  Pid.Setvalue=expect_offset;
  Pid.CurrentValue=current_offset;
  Pid.Err_A1=Pid.CurrentValue-Pid.Setvalue;
  Pid.Integral+=Pid.Err_A1;
  if(Pid.Integral>MAX_INTERGE) Pid.Integral=MAX_INTERGE;
  else if(Pid.Integral<-MAX_INTERGE) Pid.Integral=-MAX_INTERGE;
  
  Pid.Diff=Pid.Err_A1-Pid.Err_A0;
  Pid.Err_A0=Pid.Err_A1;
  Pid.Out=(Pid.Kp*Pid.Err_A1)+(Pid.Ki*Pid.Integral)+(Pid.Kd*Pid.Diff); 
  
  return Pid.Out;  
}

#endif