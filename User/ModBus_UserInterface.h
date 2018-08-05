#ifndef _ModBus_2_h_
#define _ModBus_2_h_


#define DEFAULT_RFID_ONLINE_TIME_IN_MS  3000
#define DEFAULT_UN_FORWARD_THRESHOLD    25
#define DEFAULT_UN_BACKWARD_THRESHOLD    25
#define DEFAULT_COFF_DISTANCE_1000TIME  1000
typedef struct
{
  u16 M_CONTROL_MODE;
  u16 COMM_BD ;
  u16 SLAVE_ADDR;
  u16 WHEEL_DIAMETER_IN_MM;//轮子直径
  u16 COFF_DISTANCE_1000TIME;//
  u16 rsv_3;
  u16 VEHICLE_WIDTH;//车体宽度10倍，单位cm
//--------------------------------


  
//--------------------------------  
  u16 EVEN_ODD_FILL;//保证是偶数，可能有，可能无
  u32 MOD_REG_MAGIC_WORD;
}MOD_BUS_REG;
#define MAGIC_WORD  0x1A2B3C4D
extern const MOD_BUS_REG DEFAULT_MOD_BUS_Reg;
extern MOD_BUS_REG MOD_BUS_Reg;
#define MOD_BUS_BD_LIST_LENGTH  9
extern const u32 MOD_BUS_BD_LIST[MOD_BUS_BD_LIST_LENGTH];
extern u16 RecoverFlash_Timeout;
u8 AckModBusReadReg(u16 reg_addr,u16 reg_num);
u8 AckModBusCrcError(u8 CMD_ModBus);
u8 AckModBusWriteOneReg(u16 reg_addr,u16 reg_value);
u8 AckModBusWriteMultiReg(u16 reg_addr,u16 reg_num,u8* pData);
u8 AckModBusFunctionError(u8 CMD_ModBus);
void GetFlashModBusData(MOD_BUS_REG* pMOD_BUS_Reg);
void MOD_BUS_REG_Backup(void);
void MOD_BUS_REG_MODIFY_Check(void);
extern FLASH_Status SaveFlashModBusData(MOD_BUS_REG* pMOD_BUS_Reg);

extern u16 M_Status;


#define M_CONTROL_MODE_SPEED          0
#define M_CONTROL_MODE_DISPLACEMENT   1
#define M_CONTROL_MODE_ANGLE          2
#define M_CONTROL_MODE_SPEC           4

#define M_STATUS_NOMAL  0
#define M_STATUS_STOP   1
#define M_STATUS_IM_STOP  2
#define M_STATUS_CHARGE  3

#define M_CMD_STOP      0
#define M_CMD_FORWARD   1
#define M_CMD_BACKWARD  2
#define M_CMD_LEFT      3
#define M_CMD_RIGHT     4

#define M_CMD_LEFT_MOV            5
#define M_CMD_RIGHT_MOV           6
#define M_CMD_LEFT_FOWARD_MOV     7
#define M_CMD_RIGHT_FOWARD_MOV    8
#define M_CMD_LEFT_BACKWARD_MOV   9
#define M_CMD_RIGHT_BACKWARD_MOV  10


typedef union
{
  u16 AsU16[2+1+4];
  struct
  {
    u16 M_Dir;
    u16 M_Speed;
    u16 M_FreshFlag;
    s16 LeftSpeed;
    s16 RightSpeed;
    s16 Left_2_Speed;
    s16 Right_2_Speed;    
  }M_CONTROL_OPTION;
}U_M_CONTROL_OPTION;

extern u16 M_BAT_Precent;
extern u16 M_LightSensorStatus[8];
extern U_M_CONTROL_OPTION U_M_CONTROL_Op;
extern u8 machine_state;

void Analysis_Receive_From_A8(u8 data);
#endif