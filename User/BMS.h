#ifndef _bms_h_
#define _bms_h_


#define DEFAULT_BMS_READ_START_DELAY  2000
#define DEFAULT_BMS_READ_CYCLE        1000










typedef struct
{
  u16 VCELL_MV[16];
  u32 BAT_MV;
  u32 BAT_MA;
  u32 BAT_TEMP[4];
  
  u8 RefreshFlag;
}BMS_STATUS;

extern u16 BMS_TimeOutCounter;
extern BMS_STATUS BMS_St;
extern MODBUS_SAMPLE MODBUS_Bms;
//extern u8 BMS_TX_Buf[32];
//extern const u8 BMS_READ_STATUS_ALL[6];

extern u8 NewBMSFrame(u8 cmd, u8* buf);
extern void Handle_BmsRx(u8* data, u8 len);
extern u8 BMS_ReadStatus(void);
extern void Flush_BmsRx(void);
extern u8 BMS_ResetCheck(void);

extern void Analysis_Receive_From_BMS(u8 data,MODBUS_SAMPLE* pMODBUS, void* st);
#endif