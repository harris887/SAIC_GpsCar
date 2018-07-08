#ifndef _UltraSonic_h
#define _UltraSonic_h

/*超声波模组只能使用MODBUS地址-0x01*/
#define DEFAULT_MODE_BUS_ULTRA_SONIC_ADDR 0x01
//#define ULTRA_SONIC_REG_ADDR_OFFSET       0x60
#define ULTRA_SONIC_SENSOR_NUM  6
#define DEFAULT_ULTRA_SONIC_CHECK_TIME_IN_MS    100
#define DEFAULT_ULTRA_SONIC_DISTANCE            0 //100
#define DEFAULT_ULTRA_SONIC_NO_DATA_TIME_OUT    500

typedef struct 
{
  u8 MachineState;
  u8 BufIndex;
  u8 read_receive_timer;
  u8 receive_CRC_H;
  u8 receive_CRC_L;
  u8 rev0;
  
  u8 ModBus_CMD;
  u8 err_state;
  u32 read_success_num;
  u32 write_success_num;
  u16 Read_Register_Num;
  u8 DataBuf[256];
}MODBUS_SAMPLE;

extern u8 UltraSonicMachineState;
extern u16 UltraSonicCheckTimer;
extern MODBUS_SAMPLE UltraSonic_Modbus[2];
extern u16 UltraSonicDistance[ULTRA_SONIC_SENSOR_NUM];
extern u16 US_0_Timeout;
extern u16 US_1_Timeout;

u8 AckWriteCmdToMaster(u8 CMD_ModBus,u8 r_code);
u8 AckReadCmdToMaster(u8* pDATA,u8 byte_length);
void Analysis_Receive_From_UltraSonic(u8 data,MODBUS_SAMPLE* pMODBUS,u16* pUltraSonicDistance);
void Check_UltraSonic_TASK(void);













#endif