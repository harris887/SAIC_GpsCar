#include "user_inc.h"
#include "string.h"
#define PGV_SENSOR_PRINTF_DEBUG     0


#define PGV_READ_CYCLE  (500)//(1000) //in 100us
#define PGV_CMD_RESET_CYCLE (80000)
#define PGV_START_DELAY   (80000) //启动8s

u32 PGV_TimeOutCounter=PGV_START_DELAY;
u32 PGV_CommandResetCounter=0;
u8 Current_PgvAddr=0;
PGV_DIR_MODE Current_PgvDir=PGV_DIR_LEFT;
PGV_COLOR Current_PgvLaneColor=PGV_COLOR_RED;
PGV_LANE_TACKING_INFOR PGV_LANE_TACKING_Infor=
{
  .FreshFlag=0,
};

/**************************
****** 异或校验 ***********
**************************/
u8 DATA_XOR_Check(u8* data,u8 length)
{
	u8 i=1;
	u8 result=data[0];
	for(i=1;i<length;i++)
	{
		result^=data[i];
	}
	return result;
}

/****************************
********设置轨道的方向*******
*****************************/
void PGV_SelectDirection(u8 pgv_addr,PGV_DIR_MODE dir_mode)
{
	u8 infor[3];
	infor[0] = 0xE0 | ((dir_mode&3)<<2) | (pgv_addr&3);
	infor[1] = ~infor[0];
        infor[2] = 0;
        FillUartTxBufN(infor,2,PGV_UART_PORT);
}



/****************************
********设置轨道的颜色*******
*****************************/
void PGV_SelectLaneColor(u8 pgv_addr,PGV_COLOR color)
{
	const u8 color_code[PGV_LANE_COLOR_NUM]={0xC4,0x88,0x90};
	u8 infor[3];
	infor[0] = color_code[color<PGV_LANE_COLOR_NUM?color:(PGV_LANE_COLOR_NUM-1)] | (pgv_addr&3);
	infor[1] = ~infor[0];
        infor[2] = 0;
	FillUartTxBufN(infor,2,PGV_UART_PORT);
}


/****************************
********请求位置信息*******
*****************************/
void PGV_RequestInfor(u8 pgv_addr)
{
	u8 infor[6];
	infor[0] = 0xC8 | (pgv_addr&3);
	infor[1] = ~infor[0];
        infor[2] = 0;
        infor[3] = 0;
	FillUartTxBufN(infor,2,PGV_UART_PORT);
}


/*****************************
**********接收信息。解析************
*****************************/
u8 PGV_AckInfor_Analysis(PGV_COMMAND_MODE com_mode,u8* infor,u8 infor_length)
{
	u8 status=FALSE;
	if(com_mode==PGV_COMMAND_SelectDirection)
	{
		if(infor_length==3)
		{
			if(DATA_XOR_Check(infor,infor_length-1)==infor[infor_length-1])
			{
				if((infor[1]&3)==Current_PgvDir)
				{
					status = TRUE;
				}
			}
		}
	}
	else if(com_mode==PGV_COMMAND_SelectLaneColor)
	{
		if(infor_length==2)
		{
			if(infor[0]==infor[1])
			{
				if((infor[0]&7)==(1<<Current_PgvLaneColor))
				{
					status = TRUE;
				}
			}
		}		
	}
	else if(com_mode==PGV_COMMAND_RequestInfor)
	{
		if(infor_length==21)
		{
			if(DATA_XOR_Check(infor,infor_length-1)==infor[infor_length-1])
			{
                          u32 temp;
				memcpy(&PGV_LANE_TACKING_Infor,infor,infor_length);

                                PGV_LANE_TACKING_Infor.XP = ((PGV_LANE_TACKING_Infor.X_Position[0]&0x7)<<21)\
                                  | ((PGV_LANE_TACKING_Infor.X_Position[1]&0x7F)<<14)\
                                  | ((PGV_LANE_TACKING_Infor.X_Position[2]&0X7F)<<7)\
                                  | ((PGV_LANE_TACKING_Infor.X_Position[3]&0X7F)<<0);
                                
                                if(PGV_LANE_TACKING_Infor.Status_NP)
                                {
                                  PGV_LANE_TACKING_Infor.XP=0;
                                }
                                
                                //if((PGV_LANE_TACKING_Infor.Status_NP==0)&&
                                //   ((PGV_LANE_TACKING_Infor.XP>=1080)&&(PGV_LANE_TACKING_Infor.XP<=1120)) )
                                //{
                                //  RFID_COMEIN_Flag |=1 ;
                                //}
                                
                                temp = ((PGV_LANE_TACKING_Infor.Y_Position[0]&0x7F)<<7)\
                                  | ((PGV_LANE_TACKING_Infor.Y_Position[1]&0X7F)<<0);  
                                PGV_LANE_TACKING_Infor.YP = (temp>0x2000)?(((s32)temp)-((s32)0x4000)):((s16)temp);
                                
                                PGV_LANE_TACKING_Infor.Angle = (PGV_LANE_TACKING_Infor.Angle_Positon[0]<<7)\
                                  | (PGV_LANE_TACKING_Infor.Angle_Positon[1]<<0);  
                                
                                PGV_LANE_TACKING_Infor.Tag = (PGV_LANE_TACKING_Infor.ControlCode_Or_TagCode[0]<<21)\
                                  | (PGV_LANE_TACKING_Infor.ControlCode_Or_TagCode[1]<<14)\
                                  | (PGV_LANE_TACKING_Infor.ControlCode_Or_TagCode[2]<<7)\
                                  | (PGV_LANE_TACKING_Infor.ControlCode_Or_TagCode[3]<<0);
                                PGV_LANE_TACKING_Infor.CC = (PGV_LANE_TACKING_Infor.ControlCode_Or_TagCode[0]<<7)\
                                  | (PGV_LANE_TACKING_Infor.ControlCode_Or_TagCode[1]<<0);  
                                PGV_LANE_TACKING_Infor.WRN = ((PGV_LANE_TACKING_Infor.WarningCodeHigh&0x7F)<<7)\
                                  | ((PGV_LANE_TACKING_Infor.WarningCodeLow&0X7F)<<0); 
//#if (PGV_SENSOR_PRINTF_DEBUG)
#if (0)
                                {
                                  u8 temp=0;
                                    
                                  if(PGV_LANE_TACKING_Infor.Status_NL==0) temp|=1;
                                  if(PGV_LANE_TACKING_Infor.Status_NP==0) temp|=2;
                                  if(PGV_LANE_TACKING_Infor.Status_TAG)   temp|=4;
                                  if(PGV_LANE_TACKING_Infor.Status_WRN)   temp|=8;
                                  printf("%x",temp);
                                }
                                
                                if(PGV_LANE_TACKING_Infor.Status_WRN)
                                {
                                  printf("<%x>",PGV_LANE_TACKING_Infor.WRN);
                                }
#endif                                

//#if (PGV_SENSOR_PRINTF_DEBUG)        
#if (0)                                
                                //SHOW position
                                if(PGV_LANE_TACKING_Infor.Status_NP==0)
                                {
                                  static u32 XP_Bk=0;
                                  if(XP_Bk!=PGV_LANE_TACKING_Infor.XP)
                                  {
                                    XP_Bk=PGV_LANE_TACKING_Infor.XP;
                                    printf("[%d] ",XP_Bk);
                                  }
                                } 
#endif
                                
                                PGV_LANE_TACKING_Infor.FreshFlag=1;

			}
		}		
	}
	return status;
}



void Receive_From_PGV(u8 rst_flag,u8 data)
{
#define PGV_RCV_DATA_BUF_LENGTH 32
  static u8 buf[PGV_RCV_DATA_BUF_LENGTH];
  static u8 byte_index=0;
  static u8 rst=0;
  if(rst_flag==0)
  {
    buf[byte_index++] = data;
    byte_index&=(PGV_RCV_DATA_BUF_LENGTH-1);
    rst=0;
  }
  else
  {
    if(rst==0)
    {
      PGV_AckInfor_Analysis(PGV_COMMAND_RequestInfor,buf,byte_index);
      byte_index=0;
      rst=1;
    }
  }
}


void PGV_COMM_TASK(void)
{
  static u8 pro=0;
  static u8 commnd_index=0;
  switch(pro)
  {
  case 0:
    {
      if((PGV_TimeOutCounter==0)&&(Relay_status&0x10))
      {
        PGV_RS485_TX_ACTIVE();
        PGV_TimeOutCounter=20;
        if((PGV_CommandResetCounter==0)&&(ON_LINE_Flag==0))
        {//没检测到线的时候重新设置一下
          commnd_index=0;
          PGV_CommandResetCounter=PGV_CMD_RESET_CYCLE;
        }
        pro++;
      }
    }
    break;
  case 1:
    {
      //发送读取sensor的指令
      if(PGV_TimeOutCounter==0)
      {
        Modebus_read_cmd_tx_finish=0;
        if(commnd_index==0)
        {
//#if (PGV_SENSOR_PRINTF_DEBUG)
#if (0)          
          printf("SET_PGV_DIR\n");
#endif
          PGV_SelectDirection(0,PGV_DIR_LEFT);
          commnd_index+=1;
        }
        else if(commnd_index==1)
        {
//#if(PGV_SENSOR_PRINTF_DEBUG)    
#if (0)           
          printf("SET_PGV_COLOR\n");
#endif
          PGV_SelectLaneColor(0,PGV_COLOR_RED);
          commnd_index+=1;
        }
        else
        {
          PGV_RequestInfor(0);
        }
        PGV_TimeOutCounter=2000;
        pro++;
      }
    }
    break;
  case 2:
    {
      if(PGV_TimeOutCounter==0)
      {
        PGV_RS485_RX_ACTIVE();
        PGV_TimeOutCounter = PGV_READ_CYCLE;
        pro++;
      }  
    }
    break;
  case 3:
    {
      if(PGV_TimeOutCounter!=0)
      {
        //这里应该收到数据，什么也不做
      }
      else
      {
        pro = 0;
      }
    }
    break;
  default:  
    {
      pro=0; 
    }
  }
  
  if(((Relay_status&0x10)==0)&&(ON_LINE_Flag))
  {
          ON_LINE_Flag=0;
          ON_LINE_Counter=0;
#if (PGV_SENSOR_PRINTF_DEBUG)          
          printf("PGV POWER OFF ! Force OFF LINE\n");
#endif    
  }
  
  //检测结果归纳
  if(PGV_LANE_TACKING_Infor.FreshFlag)
  {
    PGV_LANE_TACKING_Infor.FreshFlag = 0;
    
    if(ON_LINE_Flag)
    {
      if((PGV_LANE_TACKING_Infor.Status_NL!=0)&&(PGV_LANE_TACKING_Infor.Status_NP!=0))
      {
        ON_LINE_Counter+=1;
        if(ON_LINE_Counter>=20)
        {
          ON_LINE_Flag=0;
          ON_LINE_Counter=0;
#if (PGV_SENSOR_PRINTF_DEBUG)          
          printf("PGV OFF LINE!\n");
#endif
        }
      }
      else
      {
        ON_LINE_Counter=0;
      }
    }
    else
    {
      if((PGV_LANE_TACKING_Infor.Status_NL==0)||(PGV_LANE_TACKING_Infor.Status_NP==0))
      {
        ON_LINE_Counter+=1;
        if(ON_LINE_Counter>=10)
        {
          ON_LINE_Flag=1;
          ON_LINE_Counter=0;
          
#if (PGV_SENSOR_PRINTF_DEBUG)
          printf("PGV ON LINE!\n");
#endif
        }
      }
      else
      {
        ON_LINE_Counter=0;
      }
    }
  }
}