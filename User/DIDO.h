#ifndef _DIDO_H_
#define _DIDO_H_



#define CMD_ModBus_Wire_Read         0x01
#define CMD_ModBus_Wire_ReadEx       0x02
#define CMD_ModBus_Wire_Write        0x05
#define CMD_ModBus_Wire_WriteMore    0x15

typedef struct
{
  u8 RefreshFlag;
  u8 RelayStatus;
  u8 LightStatus;
}DIDO_D_INPUT_STATUS;

typedef struct
{
  u8 RefreshFlag; 
  u8 RelayStatus;
  u16 Analog[4];
}DIDO_A_INPUT_STATUS;

typedef enum
{
  DIDO_LED_Blue = 0,
  DIDO_LED_Green,
  DIDO_LED_Red,
  DIDO_LED_Eye,
  DIDO_Fan,
  DIDO_Buzzer,
  DIDO_Breath,
  DIDO_Reserve
}DIDO_OUT_INDEX;

extern u16 DIDO_COMM_Timeout;
extern u16 DIDO_READ_LIGHT_Timeout;
extern u16 DIDO_ENABLE_Timeout;
extern MODBUS_SAMPLE MODBUS_Dido_0;
extern MODBUS_SAMPLE MODBUS_Dido_1;
extern DIDO_D_INPUT_STATUS DIDO_D_INPUT_Status;
extern DIDO_A_INPUT_STATUS DIDO_A_INPUT_Status;
extern u16 DIDO_RelayStatus;

void Analysis_Receive_From_Dido(u8 data,MODBUS_SAMPLE* pMODBUS, void* stt);
void Check_DIDO_TASK(void);
void SET_DIDO_Relay(u8 index,u8 status);

#endif