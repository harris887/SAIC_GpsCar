#ifndef _PGV_H_
#define _PGV_H_

#define PGV_RS485_TX_ACTIVE   RS485_1_TX_Active
#define PGV_RS485_RX_ACTIVE   RS485_1_RX_Active
#define PGV_UART_PORT         4

typedef enum
{
  PGV_DIR_ERROR=0,
  PGV_DIR_RIGHT,
  PGV_DIR_LEFT,
  PGV_DIR_AUTO
}PGV_DIR_MODE;


typedef enum
{
  PGV_COLOR_BLUE=0,
  PGV_COLOR_GREEN,
  PGV_COLOR_RED,
  PGV_LANE_COLOR_NUM
}PGV_COLOR;


typedef enum
{
  PGV_COMMAND_SelectDirection=0,
  PGV_COMMAND_SelectLaneColor,
  PGV_COMMAND_RequestInfor,
  PGV_COMMAND_NUM
}PGV_COMMAND_MODE;


typedef struct
{
  //状态字节1,低位 -> 高位
  u8 Status_ERR:1;
  u8 Status_NP:1;
  u8 Status_WRN:1;
  u8 Status_CC1:1;
  u8 Status_A0A1:2;
  u8 Status_RSV0:2;
  
  //状态字节2
  u8 Status_RL:1;
  u8 Status_LL:1;
  u8 Status_NL:1;
  u8 Status_RSV1:1;
  u8 Status_LC0LC1:2;
  u8 Status_TAG:1;
  u8 Status_RSV2:1;

  u8 X_Position[4]; //Byte3~6
  u8 Y_Position[2]; //Byte7~8
  u8 Reserved_0[2]; //Byte9~10
  u8 Angle_Positon[2];//Byte11~12
  u8 Reserved_1[2]; //Byte13~14
  u8 ControlCode_Or_TagCode[4];////Byte15~18
  u8 WarningCodeHigh;
  u8 WarningCodeLow;
  u8 Check_Xor;
  
  //信息提取部分
  u8 FreshFlag;
  u32 XP;
  s16 YP;
  u16 Angle;
  u32 Tag;
  u16 CC;//控制码
  u16 WRN;
  
}PGV_LANE_TACKING_INFOR;

extern PGV_COLOR Current_PgvLaneColor;
extern PGV_LANE_TACKING_INFOR PGV_LANE_TACKING_Infor;
extern u32 PGV_TimeOutCounter;
extern u32 PGV_CommandResetCounter;
extern u32 tag_bk;

void Receive_From_PGV(u8 rst_flag,u8 data);
void PGV_COMM_TASK(void);

#endif