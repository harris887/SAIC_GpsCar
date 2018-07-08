#ifndef _bms_h_
#define _bms_h_


#define DEFAULT_BMS_READ_START_DELAY  2000
#define DEFAULT_BMS_READ_CYCLE        1000
#define DEFAULT_BMS_RESET_TIME        10000 //10S
#define DEFAULT_BMS_RESET_LOSE_NUM    6


#define BMS_SOI 0x3A
#define BMS_EOI 0x7E

#define DEFAULT_BMS_ADDR 0
#define DEFAULT_BMS_VAR  0

#define BMS_FUNC_CODE_STATUS  0x10
#define BMS_FUNC_OPEN_IN_MOS  0x41

//��Bit7-0:��ѹ Ƿѹ  ������  �ŵ����/��·  �����  �ŵ���  ���� ���� :	
//��Bit7-0: xx   xx    xx    xx    xx   xx    ���MOS����  �ŵ�MOS����;
#define BMS_ST_BIT_OVER_VOL_MASK      0x8000
#define BMS_ST_BIT_LOW_VOL_MASK       0x4000
#define BMS_ST_BIT_IN_OVER_CUR_MASK   0x2000
#define BMS_ST_BIT_OUT_OVER_CUR_MASK  0x1000
#define BMS_ST_BIT_IN_CHARGE_MASK     0x0800
#define BMS_ST_BIT_OUT_ELE_MASK       0x0400
#define BMS_ST_BIT_HIGH_TEMP_MASK     0x0200
#define BMS_ST_BIT_LOW_TEMP_MASK      0x0100
#define BMS_ST_MOS_IN_ELE_OPEN_MASK   0x0002
#define BMS_ST_MOS_OUT_ELE_OPEN_MASK  0x0001

typedef enum
{
  CMD_GET_PROTECT_PARAM=1,
  CMD_REALTIME_PARAM,
  RSV0,
  RSV1,
  CMD_SET_PROTECT_PARAM,
  CMD_FET,
  RSV2,
  RSV3,
  CMD_GET_VAR,
}BMS_CMD_LIST;

typedef struct
{
  u8 SOI;
  char Addr[2];
  char Cmd[2];
  char Ver[2];
  char Len[4];
  //char* Info;
  char Crc[2];
  u8 EOI;
}BMS_FRAME;


typedef struct
{
  u8 length;
  u8 func_code;
  u8 voltage_h;
  u8 voltage_l;
  u8 curr_h;
  u8 curr_l;
  u8 out_ele_num_h;
  u8 out_ele_num_l;    
  u8 in_ele_num_h;
  u8 in_ele_num_l;
  u8 status_h; 
  u8 status_l; 
  u8 temprature;
  u8 capacity;
  u8 check_sum;
}BMS_STATUS_FRAME;

typedef struct
{
  u16 voltage_mv;
  u16 curr_ma;
  u16 out_ele_num;
  u16 in_ele_num;
  u16 status; 
  u8 temprature;
  u8 capacity;
  u32 ack_num;
  u32 tx_num;
  u32 reset_num;
}BMS_STATUS;

extern u16 BMS_TimeOutCounter;
extern BMS_STATUS BMS_St;
//extern u8 BMS_TX_Buf[32];
//extern const u8 BMS_READ_STATUS_ALL[6];

extern u8 NewBMSFrame(u8 cmd, u8* buf);
extern void Handle_BmsRx(u8* data, u8 len);
extern u8 BMS_ReadStatus(void);
extern void Flush_BmsRx(void);
extern u8 BMS_ResetCheck(void);

#endif