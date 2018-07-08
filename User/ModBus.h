#ifndef __ModBus_H_
#define __ModBus_H_
#include "init.h"

#define DEFAULT_MODE_BUS_HALL_ADDR   0x0001
#define BACKWARD_MODE_BUS_HALL_ADDR   2
// ����һ��page   2K
// д֮ǰҪ��page ���в���	  ���磺 һҳ��СΪ2K������洢10�����ݵ�λ���Ȳ�һ����������Ȼ��ֱ�д��10�����ݵ�λ��
//                                   ������������ݣ����ܱ�֤������ȷ��
#define return_OK                 0x00  // һ֡���� OK 
#define illegal_function         0x01  // ���������
#define illegal_register         0x02  // �Ĵ�����ַ����
#define illegal_data              0x03  // ���ݴ���
#define crc_err                   0x04  //CRC У�����
#define switch_err                0x05  //д����ʱ����û��
#define switch_value_err          0x06  //д����ֵ����

#define CMD_ModBus_ReadEx       0x03
#define CMD_ModBus_Read         0x04
#define CMD_ModBus_Write        0x06
#define CMD_ModBus_WriteMore    0x16


#define MOTO_CONTROL_MODE_SPEED 0
#define MOTO_CONTROL_MODE_RUN_PULSE 1



extern u16 modebus_timeout;
extern u8 Modebus_read_cmd_tx_finish;
extern u16 Modebus_tx_rx_change_delay;




#define LINE_SENSOR_NUM 16
#define WONDER_MID_SENSOR_INDEX 17//(8.5*2)Ϊ������Ԥ�㣬����2
#define MAX_SENSOR_VALUE        32
#define MIN_SENSOR_VALUE        2

extern u8 HallStatusFresh;
extern u8 HallValue[LINE_SENSOR_NUM];
typedef struct
{
  u8 black_sensor_num;
  u8 black_sensor_serial_flag;
  u8 Middle_Index;
  u8 black_sensor_index_list[LINE_SENSOR_NUM];
}SENSOR_STATUS;

typedef struct
{
  u8 head_index;
  u8 tail_index;
  u16 middle_index;
}SEGMENT_OPTION;
#define MAX_SEGMENT_NUM 8
typedef struct
{
  //u8 black_sensor_num;
  u8 black_sensor_serial_flag;
  u8 Segment_Num;
  SEGMENT_OPTION seg_list[MAX_SEGMENT_NUM];
}SENSOR_STATUS_NEW;

//extern SENSOR_STATUS SENSOR_Status;
extern SENSOR_STATUS_NEW SENSOR_STATUS_New;
//ͨ�����ݸ�ʽ����
struct MODBUS
{
   u8  ModBus_CMD;
   u8  Probe_Slave_Add;
   u16 Read_Register_Add;
   u16 Write_Register_Add;
   u16 Read_Register_Num;
   u16 Read_index;
   u8  Write_Register_Data[120];
   u16 Write_Register_Num;
   u16 Write_Register_Data_One;
   //u8  A8_Buffer_1[256];
   //u8  A8_Buffer_2[256];
   //u8  A8_Buffer_3[256];
   u8  Probe_state;
   u8  err_state;//���ش�����롣
};


extern struct  MODBUS  A8_Modbus;
extern u8 Receive_Data_From_A8[256];
extern u8 HallSensorMachineState;

extern u8 calculate_CRC_H;
extern u8 calculate_CRC_L;
extern u8 Send_Data_A8_array[256];

extern u8 ON_LINE_Flag;
extern u8 ON_LINE_Counter;

//ModBus ͨ�Ÿ�ʽ
//���ӿ����� A8���յ������ݣ�����״̬�����������ݼ�����
extern void Analysis_Receive_From_ModeBusSlaveDev(unsigned char data);
extern u16 ModBus_CRC16_Calculate(unsigned char *aStr , unsigned char alen);






void MODBUS_READ_SERSOR_BOARD_TASK(void);
u8 CheckHallOnListNum(u8* hall_list,u8 total_num,SENSOR_STATUS* St);
u8 CheckHallOnListNumNew(u8* hall_list,u8 total_num,SENSOR_STATUS_NEW* St);
u8 GetSensorMiddleIndex(SENSOR_STATUS_NEW* st);

/////////////�ֲ�////////////////////////////
#define SELECT_DIR_NULL   0
#define SELECT_DIR_LEFT   1
#define SELECT_DIR_RIGHT  2

#define SELECT_DIR_MASK   0x03
#define SELECT_DIR_START_FLAG 0x80
#define SELECT_DIR_SINGLE_LINE_TIMES_MASK 0x7c
#define SELECT_DIR_SINGLE_LINE_TIMES_UP_STEP 0x4

extern u8 SelectDir;
extern u16 MB_LINE_DIR_SELECT;
extern u8 MODE_BUS_HALL_Addr;
#endif
