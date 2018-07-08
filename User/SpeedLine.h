#ifndef _speed_line_h
#define _speed_line_h


typedef struct
{
  u16 UpTime;
  u16 DownTime;
  u16 Round_100MS_PulseNum;//100MS,�����ߵ�����������Ϊϣ����������Ϊ100ms�����趨��ֵ
}SPEED_STEP_OPTION;

#define SPEED_STEP_NUM  100
extern const SPEED_STEP_OPTION SPEED_STEP_OPTION_List[SPEED_STEP_NUM];

SPEED_STEP_OPTION* GetSpeedStepOption(u16 speed_1_100);



u8 FOLLOW_LINE_ADVANCE_TASK(u8* pFollowLineReset,u16 current_point,u16 temianl_point);





#endif