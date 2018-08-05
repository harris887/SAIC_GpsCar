#include "user_inc.h"

float followline_max_speed_cm_s=15.0;//Ѳ������ٶ�15cm/s
float followline_max_acc_cm_ss=5.0;//�����ٶ�5cm/ss
float followline_PID_Timer=0.1;//ÿ0.1s����һ��

//Ѳ�ߵ��յ�ֹͣ
u8 FOLLOW_LINE_ADVANCE_TASK(u8* pFollowLineReset,u16 current_point,u16 temianl_point)
{
  static float SpeedPersent;//���Ƽ��ٺͼ��ٹ��̵ı���ֵ,0~1.0
  static float SpeedUpStep;//������������
  static s32 SpeedInPwm;//��׼�ٶȵ�PWM��ʾ
  static u16 current_point_filter;
  u8 teminal_flag=0;
  
  if(PID_TimeOut==0)
  {
    s32 pid_out;
    s32 offset;
    s32 left_speed;
    s32 right_speed;
    
    PID_TimeOut=100;
    
    //��ʼ�����ټ�����
    if(*pFollowLineReset!=0) 
    {
      *pFollowLineReset=0;
      SpeedUpStep = 1.0f / ((followline_max_speed_cm_s/followline_max_acc_cm_ss)/followline_PID_Timer);
      SpeedPersent = 0.0f;
      SpeedInPwm= (followline_max_speed_cm_s * MAX_SPEED_STEP) / max_wheel_speed;
      
      current_point_filter=0;
      //printf("SpeedUpStep=%.3f,SpeedInPwm=%d\n",SpeedUpStep,SpeedInPwm);
    }
    else
    {
      if(current_point>current_point_filter) current_point_filter=current_point;
      
      if(current_point_filter==0)
      {
        if(SpeedPersent<1.0) SpeedPersent+=SpeedUpStep;
        else SpeedPersent=1.0;
      }
      else
      {//������ά����ٵ��յ�
        if(current_point_filter>=temianl_point)
        {
          SpeedPersent=0.0f;
          teminal_flag=1;
        }
        else
        {
          SpeedPersent=((float)(temianl_point-current_point_filter+2)/followline_max_speed_cm_s);
          if(SpeedPersent>1.0f) SpeedPersent=1.0f;
        }
      }
    }

    
    pid_out=-SPEED_PID(0,PGV_LANE_TACKING_Infor.YP);

    if(pid_out<-100) pid_out=-100;
    if(pid_out>100) pid_out=100;
    offset=SpeedInPwm*pid_out/100/2;
    
    if(pid_out<0)//��ƫ�������ҿ�
    {
      left_speed = SpeedInPwm + offset;
      right_speed = SpeedInPwm - offset;
    }
    else//��ƫ���������
    {
      left_speed = SpeedInPwm + offset;
      right_speed = SpeedInPwm - offset;
    }
   
    
    left_speed = left_speed * SpeedPersent;
    right_speed = right_speed * SpeedPersent;

    SetD1Rpm(LEFT_MOTO_INDEX, (float)left_speed*0.001*(float)MAX_MOTO_SPEED_IN_D1RPM);
    SetD1Rpm(RIGHT_MOTO_INDEX, (float)right_speed*0.001*(float)MAX_MOTO_SPEED_IN_D1RPM);  

  }
  return teminal_flag;
}
