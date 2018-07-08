#ifndef _Pid_h_
#define _Pid_h_

#include "user_inc.h"


typedef struct
{
  s32 Setvalue;
  s32 CurrentValue;
  s32 Diff;
  s32 Err_A0;
  s32 Err_A1;
  s32 Integral;
  s32 Out;
  float Kp;
  float Ki;
  float Kd;
}PID_OPTION;

extern PID_OPTION Pid;

extern u16 PID_TimeOut;
extern void PID_Init(void);

extern s32 SPEED_PID(s32 expect_offset,s32 current_offset);










#endif