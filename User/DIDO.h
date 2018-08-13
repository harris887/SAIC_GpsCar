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
  DIDO_Buzzer = 0,
  DIDO_LIGHT,
  DIDO_Reserve_0,
  DIDO_Reserve_1,
  
  DIDO_BREAK_0,
  DIDO_BREAK_1,
  DIDO_BREAK_2,
  DIDO_BREAK_3,
  DIDO_MOTO_EN_0,
  DIDO_MOTO_EN_1,
  DIDO_MOTO_EN_2,
  DIDO_MOTO_EN_3
  
}DIDO_OUT_INDEX;

#define BREAK_ON   0
#define BREAK_OFF  1

#define DIDO_MOTO_ON 1
#define DIDO_MOTO_OFF 0

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