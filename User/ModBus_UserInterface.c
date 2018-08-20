#include "user_inc.h"
#include "stdlib.h"
#include "string.h"

#define MODBUS_USER_INTERFACE_PRINTF_DEBUG  0

#define U_TX_INDEX              2   //ʹ�ô���2����
#define DEFAULT_BACKWARD_DISP   150

//���豸�ļĴ���
#define MOD_BUS_REG_START_ADDR  0x0001
#define DEFAULT_MODE_BUS_AGV_ADDR   0x0001
#define MOD_BUS_REG_NUM         ((sizeof(MOD_BUS_REG)>>1)-3)//0x000C

struct  MODBUS  A8_Modbus;

u8 machine_state = 0;
static u8 index = 0;
static u8 Receive_Data_From_A8[256];
static u8 Send_Data_A8_array[256];
static u8 read_receive_timer = 0;
static u8 write_one_receive_timer = 0;
static u8 write_more_receive_timer = 0;

static u8 receive_CRC_H = 0;
static u8 receive_CRC_L = 0;
//u8 calculate_CRC_H = 0;
//u8 calculate_CRC_L = 0;

u8 Potentiometer_Position_aready_read_flag=0;

MOD_BUS_REG MOD_BUS_Reg;
MOD_BUS_REG MOD_BUS_Reg_Backup;
u8 MOD_BUS_REG_FreshFlag=0;
#define MOD_BUS_BD_LIST_LENGTH  9
const u32 MOD_BUS_BD_LIST[MOD_BUS_BD_LIST_LENGTH]=
{
  115200,//DEFAULT
  1200,2400,4800,9600,
  19200,38400,57600,115200,
}; 
const MOD_BUS_REG DEFAULT_MOD_BUS_Reg=
{
  M_CONTROL_MODE_SPEED,
  8,//8
  DEFAULT_MODE_BUS_AGV_ADDR,
  DEFAULT_WHEEL_DIAMETER_IN_MM,
  DEFAULT_COFF_DISTANCE_1000TIME,
  0,
  DEFAULT_Vehicle_WIDTH_10_TIME,
//--------------------------------

//--------------------------------
  0,
  MAGIC_WORD,
};

////////////////////////////////
u16 M_Status=M_STATUS_STOP;
u16 M_BAT_Precent=100;//�����ٷֱ�
u16 M_LightSensorStatus[8]={0,0,0,0,0,0,0,0};
U_M_CONTROL_OPTION U_M_CONTROL_Op={M_CMD_STOP,0,0,0,0,0,0};

u32 master_read_num = 0;
/*******************************************************************
��������:void Analysis_Receive_From_A8(u8 data)
��������:������λ�� ����������� ��״̬����
*******************************************************************/
void Analysis_Receive_From_A8(u8 data)
{
    switch(machine_state)//��ʼ�� Ĭ�� Ϊ 00;
    {
        case 0x00: 
        {
            if(data == MOD_BUS_Reg.SLAVE_ADDR)//�ӻ���ַ
            {
                machine_state = 0x01;//�ӻ���ַ�ɱ䣬ͨ��A8���ġ�
                index = 0;
                A8_Modbus.Probe_Slave_Add = data;
                Receive_Data_From_A8[index++] = data;
            }
            else
            {
                machine_state = 0x0B;//����������������Ҫ�����м�����Ϊ01������Ϊ��Ҫ�ӻ���ַ��
                index = 0;
            }  
        }break;
	case 0x01:
        {	 
            Receive_Data_From_A8[index++] = data;
            if(data == CMD_ModBus_Read) //ִ�ж�ȡ���������Ĵ�������  0x04 
            {
                machine_state = 0x02; 
                A8_Modbus.ModBus_CMD = CMD_ModBus_Read;
                read_receive_timer = 0;
            }
            else if(data == CMD_ModBus_Write)//ִ��д�����Ĵ�������   0x06
            {
                machine_state = 0x03;
                A8_Modbus.ModBus_CMD = CMD_ModBus_Write;
                write_one_receive_timer = 0;
            }
            else if(data == CMD_ModBus_WriteMore)//ִ��д����Ĵ�������    0x16
            {
                machine_state = 0x04;
                A8_Modbus.ModBus_CMD = CMD_ModBus_WriteMore;
                write_more_receive_timer = 0;
            }    
            else
            { 
                machine_state = 0x0A;
                A8_Modbus.ModBus_CMD = data;
                A8_Modbus.err_state = 0x01;  // ���������
            }
        }break;
	case 0x02: 
        {    
            //���յ������ܵĵ�ַ���ֽ���
            Receive_Data_From_A8[index++] = data;
            read_receive_timer++;
            if( read_receive_timer == 4 )
            {
                machine_state = 0x06;
                A8_Modbus.Read_Register_Add = Receive_Data_From_A8[index-4]*256 + Receive_Data_From_A8[index-3];
                A8_Modbus.Read_Register_Num = Receive_Data_From_A8[index-2]*256 + Receive_Data_From_A8[index-1];
                read_receive_timer = 0;
            } 
        }break;
	case 0x03: 
        {   //���յ�д���ܵ�ַ������(�����ֽ�)
            Receive_Data_From_A8[index++] = data;
            write_one_receive_timer++;
            if( write_one_receive_timer == 4 )
            {
                machine_state = 0x07;
                A8_Modbus.Write_Register_Add = Receive_Data_From_A8[index-4]*256 + Receive_Data_From_A8[index-3];
                A8_Modbus.Write_Register_Data_One = Receive_Data_From_A8[index-2]*256 + Receive_Data_From_A8[index-1];
                write_one_receive_timer = 0;
            }	 	 
        }break;
	case 0x04: 
        {   //���յ�д���ܵ�ַ�����ݸ���(����ֽ�)
            Receive_Data_From_A8[index++] = data;
            write_more_receive_timer++;
            if( write_more_receive_timer == 4 )
            {
                machine_state = 0x08;
                write_more_receive_timer = 0;
                A8_Modbus.Write_Register_Add = Receive_Data_From_A8[index-4]*256 + Receive_Data_From_A8[index-3];
                A8_Modbus.Write_Register_Num = (Receive_Data_From_A8[index-2]*256 + Receive_Data_From_A8[index-1])*2;
            }			      	 
        }break;
        case 0x06: 
        {   //��������һ�������Ĵ����ж�
            Receive_Data_From_A8[index++] = data;	
            read_receive_timer++;
            if(read_receive_timer == 2)
            {			
              u16 cal_crc;
                //������յ������� CRC�����
                cal_crc=ModBus_CRC16_Calculate(Receive_Data_From_A8,6);
                //�жϽ��յ���CRC�����������Ƿ���ͬ��
                receive_CRC_L = Receive_Data_From_A8[6];
                receive_CRC_H = Receive_Data_From_A8[7];
                if(((cal_crc>>8) == receive_CRC_H) 
                   && ((cal_crc&0xff) == receive_CRC_L) )
                {
                    A8_Modbus.err_state = 0x00;//CRCУ����ȷ 
                    //printf("uart2_rx\n");
                    AckModBusReadReg(A8_Modbus.Read_Register_Add,A8_Modbus.Read_Register_Num);
                    master_read_num += 1;
                }
                else	  
                {
                    A8_Modbus.err_state = 0x04;
                    AckModBusCrcError(CMD_ModBus_Read);
                }   
                index = 0;  
                read_receive_timer = 0;  
                machine_state = 0x00;
            }
        }break;
	case 0x07: 
        {   //д����  ��һ�Ĵ���
            Receive_Data_From_A8[index++] = data;	
        	write_one_receive_timer++;
            if(write_one_receive_timer == 2)
            {
              u16 cal_crc;
                //������յ������� CRC�����
                cal_crc=ModBus_CRC16_Calculate(Receive_Data_From_A8,6);
            	//�жϽ��յ���CRC�����������Ƿ���ͬ��
                receive_CRC_L = Receive_Data_From_A8[6];
                receive_CRC_H = Receive_Data_From_A8[7];
                if(((cal_crc>>8) == receive_CRC_H) 
                   && ((cal_crc&0xff) == receive_CRC_L) )
                {
                    A8_Modbus.err_state = 0x00;//CRCУ����ȷ 		
                    AckModBusWriteOneReg(A8_Modbus.Write_Register_Add,A8_Modbus.Write_Register_Data_One);
                }
                else	  
                {
                    A8_Modbus.err_state = 0x04;//CRCУ�����    
                    AckModBusCrcError(CMD_ModBus_Write);
                }

                machine_state = 0x00;
                index = 0;  
                write_one_receive_timer = 0;  					
            }
        }break;
        case 0x08: 
        {   //д����������Ĵ���   
            Receive_Data_From_A8[index++] = data;
            A8_Modbus.Write_Register_Num--;					   
            if(A8_Modbus.Write_Register_Num == 0)
            {
                machine_state = 0x09;
                write_more_receive_timer = 2;					   
            }   
        }break;
        case 0x09: 
        {    
            Receive_Data_From_A8[index++] = data;
            write_more_receive_timer--;					   
            if(write_more_receive_timer == 0)
            {
              u16 cal_crc;
                //������յ������� CRC�����
                cal_crc=ModBus_CRC16_Calculate(Receive_Data_From_A8,((Receive_Data_From_A8[4]*256 + Receive_Data_From_A8[5])*2 + 6));
                //�жϽ��յ���CRC�����������Ƿ���ͬ��
                receive_CRC_L = Receive_Data_From_A8[index-2];
                receive_CRC_H = Receive_Data_From_A8[index-1];	                   
                if(((cal_crc>>8) == receive_CRC_H) && 
                   ((cal_crc&0xff) == receive_CRC_L) )
                {
                    A8_Modbus.err_state = 0x00;//CRCУ����ȷ 	
                    A8_Modbus.Write_Register_Num = (Receive_Data_From_A8[4]*256 + Receive_Data_From_A8[5]);
                    AckModBusWriteMultiReg(A8_Modbus.Write_Register_Add,A8_Modbus.Write_Register_Num,&Receive_Data_From_A8[6]);
                }
                else	  
                {
                    A8_Modbus.err_state = 0x04;  
                    AckModBusCrcError(CMD_ModBus_WriteMore);
                }
                machine_state = 0x00;
                index = 0;  
                write_more_receive_timer = 0;				   
            }   
        }break;
        case 0x0A:
        {
            AckModBusFunctionError(A8_Modbus.ModBus_CMD);
            machine_state = 0x00;
        }break;
        case 0x0B:
        {//�����һ���ֽڲ��ǵ�ַ��������ȴ�����������������ͬ��
          //����ʲô���������ȴ���ʱ����״̬���Զ��ָ�
        }break;
    }
}


/*��ȡ�Ĵ���*/
u8 AckModBusReadReg(u16 reg_addr,u16 reg_num)
{
  u16 index=0;
  u16 loop;
  if(((reg_addr+reg_num)<=(MOD_BUS_REG_START_ADDR+MOD_BUS_REG_NUM))&&(reg_num<=MOD_BUS_REG_NUM)
     &&(reg_addr!=0x06)&&(reg_addr!=0x07))
  {
    u16* pBuf=&MOD_BUS_Reg.M_CONTROL_MODE;
    u16 cal_crc;
    pBuf=&pBuf[reg_addr-MOD_BUS_REG_START_ADDR];
    Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
    Send_Data_A8_array[index++]=CMD_ModBus_Read;
    Send_Data_A8_array[index++]=(reg_num<<1)>>8;//byte length ,MSB
    Send_Data_A8_array[index++]=(reg_num<<1)&0xFF;//byte length ,LSB
    for(loop=0;loop<reg_num;loop++)
    {
      Send_Data_A8_array[index++]=pBuf[loop]>>8;//MSB
      Send_Data_A8_array[index++]=pBuf[loop]&0xFF;//LSB
    }
    cal_crc = ModBus_CRC16_Calculate(Send_Data_A8_array , index);
    Send_Data_A8_array[index++]=cal_crc&0xFF;
    Send_Data_A8_array[index++]=cal_crc>>8;
    FillUartTxBufN(Send_Data_A8_array,index,U_TX_INDEX);
    return 1;
  }

  else if((reg_addr==0x07)&&(reg_num==1))
  {//��ȡAGV������
    u16 cal_crc;
    //u16 width;
    Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
    Send_Data_A8_array[index++]=CMD_ModBus_Read;
    Send_Data_A8_array[index++]=(reg_num<<1)>>8;//byte length ,MSB
    Send_Data_A8_array[index++]=(reg_num<<1)&0xFF;//byte length ,LSB
    //width=MOD_BUS_Reg.VEHICLE_WIDTH;
    //for(loop=0;loop<reg_num;loop++)
    {
      Send_Data_A8_array[index++]=MOD_BUS_Reg.VEHICLE_WIDTH>>8;
      Send_Data_A8_array[index++]=MOD_BUS_Reg.VEHICLE_WIDTH&0xff;
    }
    cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
    Send_Data_A8_array[index++]=cal_crc&0xFF;
    Send_Data_A8_array[index++]=cal_crc>>8;
    FillUartTxBufN(Send_Data_A8_array,index,U_TX_INDEX);
    return 1;
  }
  else if((reg_addr==0x08)&&(reg_num==1))
  {//��ȡ��������״̬
    u16 cal_crc;
    Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
    Send_Data_A8_array[index++]=CMD_ModBus_Read;
    Send_Data_A8_array[index++]=(reg_num<<1)>>8;//byte length ,MSB
    Send_Data_A8_array[index++]=(reg_num<<1)&0xFF;//byte length ,LSB
    //for(loop=0;loop<reg_num;loop++)
    {
      Send_Data_A8_array[index++]=SuperMode>>8;
      Send_Data_A8_array[index++]=SuperMode&0xFF;
    }
    cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
    Send_Data_A8_array[index++]=cal_crc&0xFF;
    Send_Data_A8_array[index++]=cal_crc>>8;
    FillUartTxBufN(Send_Data_A8_array,index,U_TX_INDEX);
    return 1;
  }
  else if((reg_addr==0x0E)&&(reg_num==1))
  {//��ȡ������AD
    u16 cal_crc;
    Send_Data_A8_array[index++] = MOD_BUS_Reg.SLAVE_ADDR;
    Send_Data_A8_array[index++] = CMD_ModBus_Read;
    Send_Data_A8_array[index++] = (reg_num<<1)>>8;//byte length ,MSB
    Send_Data_A8_array[index++] = (reg_num<<1)&0xFF;//byte length ,LSB
    //for(loop = 0; loop < reg_num; loop++)
    {
      Send_Data_A8_array[index++] = eCurrent_Filterd >> 8;
      Send_Data_A8_array[index++] = eCurrent_Filterd & 0xff;
    }
    cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
    Send_Data_A8_array[index++] = cal_crc&0xFF;
    Send_Data_A8_array[index++] = cal_crc>>8;
    FillUartTxBufN(Send_Data_A8_array, index, U_TX_INDEX);
    return 1;
  }
  else if((reg_addr==0x0F)&&(reg_num==1))
  {//��ȡ�¶�
    u16 cal_crc;
    u16 temp = *(u16*)&Temperature_100times; 
    Send_Data_A8_array[index++] = MOD_BUS_Reg.SLAVE_ADDR;
    Send_Data_A8_array[index++] = CMD_ModBus_Read;
    Send_Data_A8_array[index++] = (reg_num<<1)>>8;//byte length ,MSB
    Send_Data_A8_array[index++] = (reg_num<<1)&0xFF;//byte length ,LSB
    //for(loop = 0; loop < reg_num; loop++)
    {
      Send_Data_A8_array[index++] = temp >> 8;
      Send_Data_A8_array[index++] = temp & 0xff;
    }
    cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
    Send_Data_A8_array[index++] = cal_crc&0xFF;
    Send_Data_A8_array[index++] = cal_crc>>8;
    FillUartTxBufN(Send_Data_A8_array, index, U_TX_INDEX);
    return 1;
  }  
  else if((reg_addr==0x10)&&(reg_num==1))
  {//��ȡ�������
    u16 cal_crc;
    u16 temp = M_BAT_Precent; //M_BAT_Precent
    Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
    Send_Data_A8_array[index++]=CMD_ModBus_Read;
    Send_Data_A8_array[index++]=(reg_num<<1)>>8;//byte length ,MSB
    Send_Data_A8_array[index++]=(reg_num<<1)&0xFF;//byte length ,LSB
    //for(loop=0;loop<reg_num;loop++)
    {
      Send_Data_A8_array[index++]=temp>>8;
      Send_Data_A8_array[index++]=temp&0xff;
    }
    cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
    Send_Data_A8_array[index++]=cal_crc&0xFF;
    Send_Data_A8_array[index++]=cal_crc>>8;
    FillUartTxBufN(Send_Data_A8_array,index,U_TX_INDEX);
    return 1;
  }  
  else if((reg_addr==0x19)&&(reg_num==1))
  {
    //��ȡAGV���̵���״̬
    u16 cal_crc;
    Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
    Send_Data_A8_array[index++]=CMD_ModBus_Read;
    Send_Data_A8_array[index++]=(reg_num<<1)>>8;//byte length ,MSB
    Send_Data_A8_array[index++]=(reg_num<<1)&0xFF;//byte length ,LSB
    
    //for(loop=0;loop<reg_num;loop++)
    {
      Send_Data_A8_array[index++]=Relay_status>>8;
      Send_Data_A8_array[index++]=Relay_status&0xff;
    }
    cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
    Send_Data_A8_array[index++]=cal_crc&0xFF;
    Send_Data_A8_array[index++]=cal_crc>>8;
    FillUartTxBufN(Send_Data_A8_array,index,U_TX_INDEX);
    return 1;
  }
  else if((reg_addr==0x20)&&(reg_num==1))
  {//��ȡ����״̬
    u16 cal_crc;
    Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
    Send_Data_A8_array[index++]=CMD_ModBus_Read;
    Send_Data_A8_array[index++]=(reg_num<<1)>>8;//byte length ,MSB
    Send_Data_A8_array[index++]=(reg_num<<1)&0xFF;//byte length ,LSB
    
    if(AGV_RUN_Pro==AGV_STATUS_IM_STOP) 
    {
      M_Status=M_STATUS_IM_STOP; 
    }
    else if(AGV_RUN_Pro==AGV_STATUS_CHARGE)
    {
      M_Status = M_STATUS_CHARGE;
    }
    else
    {
      if((LEFT_MOTO_TIM_ENUM->CCR1==ZERO_SPEED_PWM_COUNTER)
        &&(RIGHT_MOTO_TIM_ENUM->CCR2==ZERO_SPEED_PWM_COUNTER))
      {
        M_Status=M_STATUS_STOP;
      }
      else
      {
        M_Status=M_STATUS_NOMAL;
      }
    }
    
    //for(loop=0;loop<reg_num;loop++)
    {
      Send_Data_A8_array[index++]=M_Status>>8;
      Send_Data_A8_array[index++]=M_Status&0xff;
    }
    cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
    Send_Data_A8_array[index++]=cal_crc&0xFF;
    Send_Data_A8_array[index++]=cal_crc>>8;
    FillUartTxBufN(Send_Data_A8_array,index,U_TX_INDEX);
    return 1;
  }
  else if((reg_addr==0x21)&&(reg_num==1))
  {//��ȡ��ͣ���ش�����
    u16 cal_crc;
    u16 temp;
    temp = BUTTON_IM_STOP_Flag;
    Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
    Send_Data_A8_array[index++]=CMD_ModBus_Read;
    Send_Data_A8_array[index++]=(reg_num<<1)>>8;//byte length ,MSB
    Send_Data_A8_array[index++]=(reg_num<<1)&0xFF;//byte length ,LSB
    //for(loop=0;loop<reg_num;loop++)
    {
      Send_Data_A8_array[index++]=temp>>8;
      Send_Data_A8_array[index++]=temp&0xff;
    }
    cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
    Send_Data_A8_array[index++]=cal_crc&0xFF;
    Send_Data_A8_array[index++]=cal_crc>>8;
    FillUartTxBufN(Send_Data_A8_array,index,U_TX_INDEX);
    return 1;
  }  
  else if((reg_addr==0x22)&&(reg_num==1))
  {//��ȡ��ͣ���ش�����
    u16 cal_crc;
    u16 temp;
    temp = BARRIER_Flag;
    Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
    Send_Data_A8_array[index++]=CMD_ModBus_Read;
    Send_Data_A8_array[index++]=(reg_num<<1)>>8;//byte length ,MSB
    Send_Data_A8_array[index++]=(reg_num<<1)&0xFF;//byte length ,LSB
    //for(loop=0;loop<reg_num;loop++)
    {
      Send_Data_A8_array[index++]=temp>>8;
      Send_Data_A8_array[index++]=temp&0xff;
    }
    cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
    Send_Data_A8_array[index++]=cal_crc&0xFF;
    Send_Data_A8_array[index++]=cal_crc>>8;
    FillUartTxBufN(Send_Data_A8_array,index,U_TX_INDEX);
    return 1;
  }   
  else if((reg_addr==0x21)&&(reg_num==2))
  {//��ȡ��ͣ����/����������
    u16 cal_crc;
    u16 temp[2];
    temp[0] = BUTTON_IM_STOP_Flag;
    temp[1] = BARRIER_Flag;
    Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
    Send_Data_A8_array[index++]=CMD_ModBus_Read;
    Send_Data_A8_array[index++]=(reg_num<<1)>>8;//byte length ,MSB
    Send_Data_A8_array[index++]=(reg_num<<1)&0xFF;//byte length ,LSB
    for(loop=0;loop<reg_num;loop++)
    {
      Send_Data_A8_array[index++]=temp[loop]>>8;
      Send_Data_A8_array[index++]=temp[loop]&0xff;
    }
    cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
    Send_Data_A8_array[index++]=cal_crc&0xFF;
    Send_Data_A8_array[index++]=cal_crc>>8;
    FillUartTxBufN(Send_Data_A8_array,index,U_TX_INDEX);
    return 1;
  }    
  else if((reg_addr==0x30)&&(reg_num==2))
  {//��ȡ�����ٶȿ���
    u16 cal_crc;
    u16 temp[2];
    s16 L,R;
    L = MONITOR_St[LEFT_MOTO_INDEX].real_mms;
    R = MONITOR_St[RIGHT_MOTO_INDEX].real_mms;
    
    temp[0] = U_M_CONTROL_Op.M_CONTROL_OPTION.M_Dir;
    temp[1] = (abs(L) + abs(R) + 1) >> 1;
    Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
    Send_Data_A8_array[index++]=CMD_ModBus_Read;
    Send_Data_A8_array[index++]=(reg_num<<1)>>8;//byte length ,MSB
    Send_Data_A8_array[index++]=(reg_num<<1)&0xFF;//byte length ,LSB
      
    for(loop=0;loop<reg_num;loop++)
    {
      Send_Data_A8_array[index++]=temp[loop]>>8;
      Send_Data_A8_array[index++]=temp[loop]&0xff;
    }
    cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
    Send_Data_A8_array[index++]=cal_crc&0xFF;
    Send_Data_A8_array[index++]=cal_crc>>8;
    FillUartTxBufN(Send_Data_A8_array,index,U_TX_INDEX);
    return 1;
  }
  else if((reg_addr==0x32)&&(reg_num==3))
  {
    //��ȡ����λ�ƿ���
    u16 cal_crc;
    Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
    Send_Data_A8_array[index++]=CMD_ModBus_Read;
    Send_Data_A8_array[index++]=(reg_num<<1)>>8;//byte length ,MSB
    Send_Data_A8_array[index++]=(reg_num<<1)&0xFF;//byte length ,LSB
    //for(loop=0;loop<reg_num;loop++)
    {
      u32 displacement=DISPLACEMENT_MOVEMENT_OPTION_List.buf[(DISPLACEMENT_MOVEMENT_OPTION_List.In_index-1)&LIST_LENGTH_MASK].value;
      Send_Data_A8_array[index++]=0;
      Send_Data_A8_array[index++]=
        DISPLACEMENT_MOVEMENT_OPTION_List.buf[(DISPLACEMENT_MOVEMENT_OPTION_List.In_index-1)&LIST_LENGTH_MASK].dir&0xFF;
      
      //�ǶȺ�λ��ģʽ������أ�20160921  
      if(MOD_BUS_Reg.M_CONTROL_MODE==M_CONTROL_MODE_DISPLACEMENT)
      {
        displacement=Finish_Disp_or_Angle_value;
      }
      
      Send_Data_A8_array[index++]=(displacement>>24)&0XFF;
      Send_Data_A8_array[index++]=(displacement>>16)&0XFF;
      Send_Data_A8_array[index++]=(displacement>>8)&0XFF;
      Send_Data_A8_array[index++]=(displacement>>0)&0XFF;
    }
    cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
    Send_Data_A8_array[index++]=cal_crc&0xFF;
    Send_Data_A8_array[index++]=cal_crc>>8;
    FillUartTxBufN(Send_Data_A8_array,index,U_TX_INDEX);
    return 1;
  }
  /**/
  else if((reg_addr==0x36)&&(reg_num==2))
  {
    //��ȡ����Ƕȿ���
    u16 cal_crc;
    Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
    Send_Data_A8_array[index++]=CMD_ModBus_Read;
    Send_Data_A8_array[index++]=(reg_num<<1)>>8;//byte length ,MSB
    Send_Data_A8_array[index++]=(reg_num<<1)&0xFF;//byte length ,LSB
    //for(loop=0;loop<reg_num;loop++)
    {
      u32 angle=ANGLE_MOVEMENT_OPTION_List.buf[(ANGLE_MOVEMENT_OPTION_List.In_index-1)&LIST_LENGTH_MASK].value;
      Send_Data_A8_array[index++]=ANGLE_MOVEMENT_OPTION_List.buf[(ANGLE_MOVEMENT_OPTION_List.In_index-1)&LIST_LENGTH_MASK].dir>>8;
      Send_Data_A8_array[index++]=
        ANGLE_MOVEMENT_OPTION_List.buf[(ANGLE_MOVEMENT_OPTION_List.In_index-1)&LIST_LENGTH_MASK].dir&0xFF;
      
      //�ǶȺ�λ��ģʽ������أ�20160921  
      if(MOD_BUS_Reg.M_CONTROL_MODE==M_CONTROL_MODE_ANGLE)
      {
        u32 temp;
        if(Wonder_Disp_or_Angle_value!=0)
        {
          temp=(float)angle*(Finish_Disp_or_Angle_value/(float)Wonder_Disp_or_Angle_value);
          if(temp<angle) angle=temp;
        }
      }
      Send_Data_A8_array[index++]=(angle>>8)&0xFF;
      Send_Data_A8_array[index++]=(angle>>0)&0xFF;
    }
    cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
    Send_Data_A8_array[index++]=cal_crc&0xFF;
    Send_Data_A8_array[index++]=cal_crc>>8;
    FillUartTxBufN(Send_Data_A8_array,index,U_TX_INDEX);
    return 1;
  }  
  else if((reg_addr==0x38)&&(reg_num==4))
  {//��ȡ�����ٶ�
    u16 cal_crc;
    u16 temp[4];
    temp[0] = (MONITOR_St[LEFT_MOTO_INDEX].real_mms<0)?1:0;
    temp[2] = (MONITOR_St[RIGHT_MOTO_INDEX].real_mms<0)?1:0;
    temp[1] = abs(MONITOR_St[LEFT_MOTO_INDEX].real_mms);
    temp[3] = abs(MONITOR_St[RIGHT_MOTO_INDEX].real_mms);
    Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
    Send_Data_A8_array[index++]=CMD_ModBus_Read;
    Send_Data_A8_array[index++]=(reg_num<<1)>>8;//byte length ,MSB
    Send_Data_A8_array[index++]=(reg_num<<1)&0xFF;//byte length ,LSB
    for(loop=0;loop<reg_num;loop++)
    {
      Send_Data_A8_array[index++]=temp[loop]>>8;
      Send_Data_A8_array[index++]=temp[loop]&0xff;      
    }
    cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
    Send_Data_A8_array[index++]=cal_crc&0xFF;
    Send_Data_A8_array[index++]=cal_crc>>8;
    FillUartTxBufN(Send_Data_A8_array,index,U_TX_INDEX);
    return 1;    
  }
  else if((reg_addr==0x3C)&&(reg_num==4))
  {
    //��ȡ���ӵ��ۼ�λ��
    u16 cal_crc;
    u32 L,R;
    s32 LS = (s32)RoadLength[0];
    s32 RS = (s32)RoadLength[1];
    L=*(u32*)(&LS);
    R=*(u32*)(&RS);
    Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
    Send_Data_A8_array[index++]=CMD_ModBus_Read;
    Send_Data_A8_array[index++]=(reg_num<<1)>>8;//byte length ,MSB
    Send_Data_A8_array[index++]=(reg_num<<1)&0xFF;//byte length ,LSB
    //for(loop=0;loop<reg_num;loop++)
    {
      Send_Data_A8_array[index++]=(L>>24)&0xFF;
      Send_Data_A8_array[index++]=(L>>16)&0xFF;
      Send_Data_A8_array[index++]=(L>>8)&0xFF;
      Send_Data_A8_array[index++]=(L>>0)&0xFF;;
      Send_Data_A8_array[index++]=(R>>24)&0xFF;
      Send_Data_A8_array[index++]=(R>>16)&0xFF;
      Send_Data_A8_array[index++]=(R>>8)&0xFF;
      Send_Data_A8_array[index++]=(R>>0)&0xFF;      
    }
    cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
    Send_Data_A8_array[index++]=cal_crc&0xFF;
    Send_Data_A8_array[index++]=cal_crc>>8;
    FillUartTxBufN(Send_Data_A8_array,index,U_TX_INDEX);
    return 1;  
  }    
  else if((reg_addr==0x70)&&(reg_num==1))
  {
    //λ�Ʋ���ϵ��
    u16 cal_crc;
    u16 value=Displacement_coff*1000;
    Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
    Send_Data_A8_array[index++]=CMD_ModBus_Read;
    Send_Data_A8_array[index++]=(reg_num<<1)>>8;//byte length ,MSB
    Send_Data_A8_array[index++]=(reg_num<<1)&0xFF;//byte length ,LSB
    //for(loop=0;loop<reg_num;loop++)
    {
      Send_Data_A8_array[index++]=(value>>8)&0xFF;
      Send_Data_A8_array[index++]=(value>>0)&0xFF;      
    }
    cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
    Send_Data_A8_array[index++]=cal_crc&0xFF;
    Send_Data_A8_array[index++]=cal_crc>>8;
    FillUartTxBufN(Send_Data_A8_array,index,U_TX_INDEX);
    return 1;   
  }      
  else if((reg_addr==0x71)&&(reg_num==1))
  {
    //�ǶȲ���ϵ��
    u16 cal_crc;
    u16 value=Angle_coff*1000;
    Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
    Send_Data_A8_array[index++]=CMD_ModBus_Read;
    Send_Data_A8_array[index++]=(reg_num<<1)>>8;//byte length ,MSB
    Send_Data_A8_array[index++]=(reg_num<<1)&0xFF;//byte length ,LSB
    //for(loop=0;loop<reg_num;loop++)
    {
      Send_Data_A8_array[index++]=(value>>8)&0xFF;
      Send_Data_A8_array[index++]=(value>>0)&0xFF;      
    }
    cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
    Send_Data_A8_array[index++]=cal_crc&0xFF;
    Send_Data_A8_array[index++]=cal_crc>>8;
    FillUartTxBufN(Send_Data_A8_array,index,U_TX_INDEX);
    return 1;   
  }      
  else if((reg_addr==0x80)&&(reg_num==3))
  {
    //��ȡǰPID����
    u16 cal_crc;
    s16 PID_List[3];
    PID_List[0]=(Pid.Kp*1000);
    PID_List[1]=(Pid.Ki*1000);
    PID_List[2]=(Pid.Kd*1000);
    Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
    Send_Data_A8_array[index++]=CMD_ModBus_Read;
    Send_Data_A8_array[index++]=(reg_num<<1)>>8;//byte length ,MSB
    Send_Data_A8_array[index++]=(reg_num<<1)&0xFF;//byte length ,LSB
    for(loop=0;loop<reg_num;loop++)
    {
      Send_Data_A8_array[index++]=(PID_List[loop]>>8)&0xFF;
      Send_Data_A8_array[index++]=PID_List[loop]&0xFF;      
    }
    cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
    Send_Data_A8_array[index++]=cal_crc&0xFF;
    Send_Data_A8_array[index++]=cal_crc>>8;
    FillUartTxBufN(Send_Data_A8_array,index,U_TX_INDEX);
    return 1; 
  }

  else if((reg_addr==0x98)&&(reg_num==1))
  {//��ȡ�����ͣ���ش�����
    u16 cal_crc;
    u16 temp;
    temp = Software_IM_STOP;
    Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
    Send_Data_A8_array[index++]=CMD_ModBus_Read;
    Send_Data_A8_array[index++]=(reg_num<<1)>>8;//byte length ,MSB
    Send_Data_A8_array[index++]=(reg_num<<1)&0xFF;//byte length ,LSB
    //for(loop=0;loop<reg_num;loop++)
    {
      Send_Data_A8_array[index++]=temp>>8;
      Send_Data_A8_array[index++]=temp&0xff;
    }
    cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
    Send_Data_A8_array[index++]=cal_crc&0xFF;
    Send_Data_A8_array[index++]=cal_crc>>8;
    FillUartTxBufN(Send_Data_A8_array,index,U_TX_INDEX);
    return 1;
  }
  else if((reg_addr == 0xB0) && (reg_num == 7))  
  { //��ȡ DAM0808_INPUT and DAM0808_OUTPUT and DAM0404_INPUT[4] and DAM0404_OUTPUT
    u16 cal_crc;
    u16 temp[7];
    temp[0] = DIDO_D_INPUT_Status.LightStatus;
    temp[1] = DIDO_RelayStatus & 0xFF;
#if (READ_RPM_RATE_1000)
    temp[2] = rpm_rate[0];
    temp[3] = rpm_rate[1];
    temp[4] = rpm_rate[2];
    temp[5] = rpm_rate[3];
#else
    temp[2] = DIDO_A_INPUT_Status.Analog[0];
    temp[3] = DIDO_A_INPUT_Status.Analog[1];
    temp[4] = DIDO_A_INPUT_Status.Analog[2];
    temp[5] = DIDO_A_INPUT_Status.Analog[3];
#endif
    temp[6] = (DIDO_RelayStatus >> 8) & 0xFF;
    
    Send_Data_A8_array[index++] = MOD_BUS_Reg.SLAVE_ADDR;
    Send_Data_A8_array[index++] = CMD_ModBus_Read;
    Send_Data_A8_array[index++] = (reg_num<<1)>>8;//byte length ,MSB
    Send_Data_A8_array[index++] = (reg_num<<1)&0xFF;//byte length ,LSB
    for(loop = 0; loop < reg_num; loop++)
    {
      Send_Data_A8_array[index++] = temp[loop] >> 8;
      Send_Data_A8_array[index++] = temp[loop] & 0xff;
    }
    cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
    Send_Data_A8_array[index++]=cal_crc&0xFF;
    Send_Data_A8_array[index++]=cal_crc>>8;
    FillUartTxBufN(Send_Data_A8_array,index,U_TX_INDEX);
    return 1;
  }
  else if((reg_addr == 0xC0) && (reg_num == 16))  
  {
    u16 cal_crc;
    Send_Data_A8_array[index++] = MOD_BUS_Reg.SLAVE_ADDR;
    Send_Data_A8_array[index++] = CMD_ModBus_Read;
    Send_Data_A8_array[index++] = (reg_num << 1) >> 8;   //byte length ,MSB
    Send_Data_A8_array[index++] = (reg_num << 1) & 0xFF; //byte length ,LSB
    
    memcpy(Send_Data_A8_array + index, BMS_RX_Bytes + 16 * sizeof(u16), 15 * sizeof(u16));
    index += (15 * sizeof(u16));
    Send_Data_A8_array[index++] = 0;
    Send_Data_A8_array[index++] = BMS_St.Valid;
 
    cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
    Send_Data_A8_array[index++]=cal_crc&0xFF;
    Send_Data_A8_array[index++]=cal_crc>>8;
    FillUartTxBufN(Send_Data_A8_array,index,U_TX_INDEX);
    return 1;
  }
  else
  {//���ݴ��󡢳�����Χ illegal_data;Return-Code=0x03
    u16 cal_crc;
    Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
    Send_Data_A8_array[index++]=CMD_ModBus_Read;
    Send_Data_A8_array[index++]=illegal_register;
    cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
    Send_Data_A8_array[index++]=cal_crc&0xFF;
    Send_Data_A8_array[index++]=cal_crc>>8;
    FillUartTxBufN(Send_Data_A8_array,index,U_TX_INDEX);
  }
  return 0;
}

u8 AckModBusCrcError(u8 CMD_ModBus)
{
  u16 index=0;
  u16 cal_crc;
  Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
  Send_Data_A8_array[index++]=CMD_ModBus;
  Send_Data_A8_array[index++]=crc_err;
  cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
  Send_Data_A8_array[index++]=cal_crc&0xFF;
  Send_Data_A8_array[index++]=cal_crc>>8;
  FillUartTxBufN(Send_Data_A8_array,index,U_TX_INDEX);
  return 0;  
}

u8 AckModBusFunctionError(u8 CMD_ModBus)
{
  u16 index=0;
  u16 cal_crc;
  Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
  Send_Data_A8_array[index++]=CMD_ModBus;
  Send_Data_A8_array[index++]=illegal_function;
  cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
  Send_Data_A8_array[index++]=cal_crc&0xff;
  Send_Data_A8_array[index++]=cal_crc>>8;
  FillUartTxBufN(Send_Data_A8_array,index,U_TX_INDEX);
  return 0;  
}

u8 AckModBusWriteOneReg(u16 reg_addr,u16 reg_value)
{
  u16 index=0;
  u8 return_code=return_OK;
  u16 cal_crc;
  switch(reg_addr)
  {
  case 0x01://���ÿ���ģʽ
    {
      if(reg_value <= M_CONTROL_MODE_SPEC) //M_CONTROL_MODE_DISPLACEMENT
      {
        MOD_BUS_Reg.M_CONTROL_MODE=reg_value;
        //MOD_BUS_REG_FreshFlag=1;
        return_code=return_OK;    
      }
      else
      {
        return_code=illegal_data;
      }    
    }
    break;  
  case 0x02://���ò�����
    {
      if((reg_value>=1)&&(reg_value<=8))
      {
        MOD_BUS_Reg.COMM_BD=reg_value;
        MOD_BUS_REG_FreshFlag=1;
        return_code=return_OK;
      }
      else
      {
        return_code=illegal_data;
      }
    }
    break;    
  case 0x03://���ôӻ���ַ
    {
      if(reg_value!=MOD_BUS_Reg.SLAVE_ADDR)
      {
        MOD_BUS_Reg.SLAVE_ADDR=reg_value;
        MOD_BUS_REG_FreshFlag=1;
      }
      return_code=return_OK;
    }
    break;   
  case 0xA0://��������ֱ��
    {
      if(reg_value != MOD_BUS_Reg.WHEEL_DIAMETER_IN_MM)
      {
        MOD_BUS_Reg.WHEEL_DIAMETER_IN_MM = reg_value;
        MOD_BUS_REG_FreshFlag=1;
      }
      return_code=return_OK;
    }
    break;    
  case 0xA1://����λ�ƻ���ϵ��
    {
      if((reg_value>=100)&&(reg_value<=10000))//ϵ��0.1~10֮��
      {
        if(reg_value != MOD_BUS_Reg.COFF_DISTANCE_1000TIME)
        {
          MOD_BUS_Reg.COFF_DISTANCE_1000TIME = reg_value;
          MOD_BUS_REG_FreshFlag=1;
        }
        return_code=return_OK;
      }
      else
      {
        return_code=illegal_data;
      }      
    }
    break;     
  case 0x04:
    {
      if(reg_value==1)
      {
        StartAutoChargeFlag=reg_value;
        Relay_status|=0x10;
        //SetRelay(RELAY_PGV_POWER_Index,RELAY_ON);
        StopAutoChargeFlag=0;
        return_code=return_OK;
      }
      else
      {
        return_code=illegal_data;
      }
    }
    break;
  case 0x05:
    {
      if(reg_value&3)
      {
        if(reg_value&1)
        {
          StartAutoChargeFlag=0;
          StopAutoChargeFlag=reg_value;
          Relay_status&=~0x10;
          //SetRelay(RELAY_PGV_POWER_Index,RELAY_OFF); 
        }
        if(reg_value&2)
        {
          StartLeaveChargeHouse=1;
          //����xcm
          MOVEMENT_OPTION* pM=
            &DISPLACEMENT_MOVEMENT_OPTION_List.buf[DISPLACEMENT_MOVEMENT_OPTION_List.In_index&LIST_LENGTH_MASK];
          pM->dir=1;
          pM->value=DEFAULT_BACKWARD_DISP;
          DISPLACEMENT_MOVEMENT_OPTION_List.In_index+=1;          
          
        }        
        return_code=return_OK;
      }
      else
      {
        return_code=illegal_data;
      }
    }
    break;    
  case 0x06:    
    {//�����۾�
      if(reg_value<=1)
      {
        //SET_DIDO_Relay(DIDO_LED_Eye, reg_value);
        return_code=return_OK;
      }
      else
      {
        return_code=illegal_data;
      }  
    }    
  case 0x07:
    {
      if((reg_value>=100)&&(reg_value<=3000))//�����ȴ���10~300cm֮��
      {
        MOD_BUS_Reg.VEHICLE_WIDTH=reg_value;
        VehicleWidth=MOD_BUS_Reg.VEHICLE_WIDTH/10;
        MOD_BUS_REG_FreshFlag=1;
        return_code=return_OK;
      }
      else
      {
        return_code=illegal_data;
      }    
    }
    break;
  case 0x08:
    {//��ȡ��������״̬
      if(reg_value<=1)//�����ȴ���10~300cm֮��
      {
        SuperMode = reg_value;  
        return_code=return_OK;
      }
      else
      {
        return_code=illegal_data;
      }    
    }
    break;    
  case 0x11:
    {
      if(reg_value<=1)
      {
        u16 temp;
        temp=(reg_value)?0x10:0x0;        
        Relay_status&=~(0x10);
        Relay_status|=temp;
        if(Relay_status&0x10)
        {
          //SetRelay(RELAY_PGV_POWER_Index,RELAY_ON);
        }
        else
        {
          //SetRelay(RELAY_PGV_POWER_Index,RELAY_OFF);
        }
        return_code=return_OK;
      }
      else
      {
        return_code=illegal_data;
      }            
    }
    break;
  case 0x19:
    {//���м̵�������
      if(reg_value<=0x3F)
      {
        Relay_status=reg_value;
        if(Relay_status&0x1)
        {
          //SetRelay(RELAY_CHARGE_N_Index,RELAY_ON);
        }
        else
        {
          //SetRelay(RELAY_CHARGE_N_Index,RELAY_OFF); 
        }        
        if(Relay_status&0x2)
        {
          //SetRelay(RELAY_CHARGE_P_Index,RELAY_ON);
        }
        else
        {
          //SetRelay(RELAY_CHARGE_P_Index,RELAY_OFF);
        }          
        if(Relay_status&0x10)
        {
          //SetRelay(RELAY_PGV_POWER_Index,RELAY_ON);
        }
        else
        {
          //SetRelay(RELAY_PGV_POWER_Index,RELAY_OFF);
        }  
        if(Relay_status&0x20)
        {
          //SetRelay(RELAY_SPOWER_Index,RELAY_ON);
        }
        else
        {
          //SetRelay(RELAY_SPOWER_Index,RELAY_OFF);
        }          
        
        return_code=return_OK;
      }
      else
      {
        return_code=illegal_data;
      }       
    }
    break;
  case 0x70:
    {//λ��ģʽ����
      if((reg_value>=100)&&(reg_value<=10000))//ϵ��0.1~10֮��
      {
        Displacement_coff=((float)reg_value)/1000.0;
        return_code=return_OK;
      }
      else
      {
        return_code=illegal_data;
      }    
    }
    break;  
  case 0x71:
    {//�Ƕ�ģʽ����
      if((reg_value>=100)&&(reg_value<=10000))//ϵ��0.1~10֮��
      {
        Angle_coff=((float)reg_value)/1000.0;
        return_code=return_OK;
      }
      else
      {
        return_code=illegal_data;
      }    
    }
    break;   
  case 0x80:
    {//PID.Kp
      if(reg_value<32767)
      {
        Pid.Kp=((float)reg_value)/1000.0f;
        return_code=return_OK;
      }
      else
      {
        return_code=illegal_data;
      }    
    }
    break;     
  case 0x81:
    {//PID.Ki
      if(reg_value<32767)
      {
        Pid.Ki=((float)reg_value)/1000.0f;
        return_code=return_OK;
      }
      else
      {
        return_code=illegal_data;
      }    
    }
    break;   
  case 0x82:
    {//PID.Kd
      if(reg_value<32767)
      {
        Pid.Kd=((float)reg_value)/1000.0f;
        return_code=return_OK;
      }
      else
      {
        return_code=illegal_data;
      }    
    }
    break;       
  case 0x90:
    {//���Ƶƴ�
      if(reg_value<=0x7)
      {
        //if(reg_value&0x1) SET_DIDO_Relay(DIDO_LED_Red,1);
        //else SET_DIDO_Relay(DIDO_LED_Red,0);
        //if(reg_value&0x2) SET_DIDO_Relay(DIDO_LED_Green,1);
        //else SET_DIDO_Relay(DIDO_LED_Green,0);
        //if(reg_value&0x4) SET_DIDO_Relay(DIDO_LED_Blue,1);
        //else SET_DIDO_Relay(DIDO_LED_Blue,0);        
        return_code=return_OK;
      }
      else
      {
        return_code=illegal_data;
      }    
    }
    break;
  case 0x91:
    {//�����۾�
      if(reg_value<=1)
      {
        //SET_DIDO_Relay(DIDO_LED_Eye, reg_value);
        return_code=return_OK;
      }
      else
      {
        return_code=illegal_data;
      }  
    }
    break;
  case 0x92:
    {//���Ʒ���
      if(reg_value<=1)
      {
        //SET_DIDO_Relay(DIDO_Fan, reg_value);
        return_code=return_OK;
      }
      else
      {
        return_code=illegal_data;
      }  
    }
    break;    
  case 0x93:   
    {//���Ʒ�����
      if(reg_value<=1)
      {
        SET_DIDO_Relay(DIDO_Buzzer, reg_value);
        return_code=return_OK;
      }
      else
      {
        return_code=illegal_data;
      }  
    }    
  case 0x94:    
    {//���ƺ�����
      if(reg_value<=1)
      {
        //SET_DIDO_Relay(DIDO_Breath, reg_value);
        return_code=return_OK;
      }
      else
      {
        return_code=illegal_data;
      }    
    }
    break;     
  case 0x95:    
    {//IMU�Ƕ�
      if(reg_value<=36000)
      {
        IMU_Angle = reg_value;
        return_code=return_OK;
      }
      else
      {
        return_code=illegal_data;
      }    
    }
    break;    
  case 0x96:
    {//�����˶��Ĵ���
      if(reg_value<=10)
      {
        RECIPROCATE_Op.wonder_reciprocate_times += reg_value;
        return_code=return_OK;
      }
      else
      {
        return_code=illegal_data;
      }    
    }    
    break;
  case 0x98:
    {//�����ͣ
      if(reg_value <= 1)
      {
        Software_IM_STOP = reg_value;
        return_code=return_OK;
      }
      else
      {
        return_code=illegal_data;
      }    
    }
    break;
  default:
    return_code=illegal_register;
  }
  
  //�ظ��û�
  Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
  Send_Data_A8_array[index++]=CMD_ModBus_Write;
  Send_Data_A8_array[index++]=return_code;
  cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
  Send_Data_A8_array[index++]=cal_crc&0xff;
  Send_Data_A8_array[index++]=cal_crc>>8;
  FillUartTxBufN(Send_Data_A8_array,index,U_TX_INDEX);
  return 0;    
}

u8 AckModBusWriteMultiReg(u16 reg_addr,u16 reg_num,u8* pData)
{
  u16 index=0;
  u8 return_code=return_OK;  
  u16 cal_crc;
  switch(reg_addr)
  {
  case 0x30://�����ٶȿ���
    {
      if(reg_num==2)
      {  
        u16 dir,speed;
        dir=(((u16)pData[0])<<8)|(((u16)pData[1])<<0);
        speed=(((u16)pData[2])<<8)|(((u16)pData[3])<<0);
        
        if((dir<=M_CMD_RIGHT)&&(speed<=MAX_SPEED_IN_MMS))
        {
          U_M_CONTROL_Op.M_CONTROL_OPTION.M_Dir=dir;
          U_M_CONTROL_Op.M_CONTROL_OPTION.M_Speed=speed;
          U_M_CONTROL_Op.M_CONTROL_OPTION.M_FreshFlag=1;
          switch(dir)
          {
          case M_CMD_STOP:
            {
              U_M_CONTROL_Op.M_CONTROL_OPTION.LeftSpeed=0;
              U_M_CONTROL_Op.M_CONTROL_OPTION.RightSpeed=0;
            }
            break;
          case M_CMD_FORWARD:
            {
              U_M_CONTROL_Op.M_CONTROL_OPTION.LeftSpeed=speed;
              U_M_CONTROL_Op.M_CONTROL_OPTION.RightSpeed=speed;  
            }
            break;
          case M_CMD_BACKWARD:
            {
              U_M_CONTROL_Op.M_CONTROL_OPTION.LeftSpeed=-speed;
              U_M_CONTROL_Op.M_CONTROL_OPTION.RightSpeed=-speed;   
            }
            break;
          case M_CMD_LEFT:
            {          
              U_M_CONTROL_Op.M_CONTROL_OPTION.LeftSpeed=speed * DIFF_COFF;
              U_M_CONTROL_Op.M_CONTROL_OPTION.RightSpeed=speed;
            }
            break;
          case M_CMD_RIGHT:
            {            
              U_M_CONTROL_Op.M_CONTROL_OPTION.LeftSpeed=speed;
              U_M_CONTROL_Op.M_CONTROL_OPTION.RightSpeed=speed * DIFF_COFF;
            }
            break;          
          }
          return_code=return_OK; 
        }
        else return_code=illegal_data;
      }
      else 
      {
        return_code=illegal_data;
      }
    }
    break;
  case 0x32:
    {//����-����λ��
      if(reg_num==3)
      {  
        u32 dir,displacement;
        dir=(((u32)pData[0])<<8)|(((u32)pData[1])<<0);
        displacement=(((u32)pData[2])<<24)|(((u32)pData[3])<<16)|(((u32)pData[4])<<8)|(((u32)pData[5])<<0);
        
        if((dir<=1)&&(MOD_BUS_Reg.M_CONTROL_MODE==M_CONTROL_MODE_DISPLACEMENT)&&(displacement>0))
        {
          MOVEMENT_OPTION* pM=
            &DISPLACEMENT_MOVEMENT_OPTION_List.buf[DISPLACEMENT_MOVEMENT_OPTION_List.In_index&LIST_LENGTH_MASK];
          pM->dir=dir;
          pM->value=displacement;
          DISPLACEMENT_MOVEMENT_OPTION_List.In_index+=1;

          return_code=return_OK; 
        }
        else return_code=illegal_data;
      }
      else 
      {
        return_code=illegal_data;
      }
    }
    break;
  case 0x36:
    {//����-����Ƕ�
      /**/
      if(reg_num==2)
      {  
        u32 dir,angle;
        dir=(((u32)pData[0])<<8)|(((u32)pData[1])<<0);
        angle=(((u32)pData[2])<<8)|(((u32)pData[3])<<0);
        
        if(((dir>>8)<=2)&&((dir&0xFF)<=1)&&(MOD_BUS_Reg.M_CONTROL_MODE==M_CONTROL_MODE_ANGLE)&&(angle>0)&&(angle<=360))
        {
          MOVEMENT_OPTION* pM=
            &ANGLE_MOVEMENT_OPTION_List.buf[ANGLE_MOVEMENT_OPTION_List.In_index&LIST_LENGTH_MASK];
          pM->dir=dir;
          pM->value=angle;
          ANGLE_MOVEMENT_OPTION_List.In_index+=1;
          //MOD_BUS_REG_FreshFlag=1;
          return_code=return_OK; 
        }
        else return_code=illegal_data;
      }
      else 
      {
        return_code=illegal_data;
      }
      
    }
    break;    
  case 0x38:
    {//�����ٶȿ���
      if(reg_num==4)
      {
        u16 L_dir,R_dir,L_Speed,R_Speed;
        L_dir=(((u16)pData[0])<<8)|(((u16)pData[1])<<0);
        L_Speed=(((u16)pData[2])<<8)|(((u16)pData[3])<<0);
        R_dir=(((u16)pData[4])<<8)|(((u16)pData[5])<<0);
        R_Speed=(((u16)pData[6])<<8)|(((u16)pData[7])<<0);
        if((L_dir<=1)&&(R_dir<=1)&&(L_Speed<=MAX_SPEED_IN_MMS)&&(R_Speed<=MAX_SPEED_IN_MMS))
        {
          U_M_CONTROL_Op.M_CONTROL_OPTION.M_FreshFlag=1;
          if(L_dir)
            U_M_CONTROL_Op.M_CONTROL_OPTION.LeftSpeed=-L_Speed;
          else
            U_M_CONTROL_Op.M_CONTROL_OPTION.LeftSpeed=L_Speed;
          if(R_dir)
            U_M_CONTROL_Op.M_CONTROL_OPTION.RightSpeed=-R_Speed;
          else        
            U_M_CONTROL_Op.M_CONTROL_OPTION.RightSpeed=R_Speed;
        }
        else return_code=illegal_data;
      }
      else return_code=illegal_data;
    }
    break;
  case 0x3C:
    {//����-����λ������
      if(reg_num==4)
      {
        u32 L,R;
        L=(((u32)pData[0])<<24)|(((u32)pData[1])<<16)|(((u32)pData[2])<<8)|(((u32)pData[3])<<0);
        R=(((u32)pData[4])<<24)|(((u32)pData[5])<<16)|(((u32)pData[6])<<8)|(((u32)pData[7])<<0);
        RoadLength[0]=(float)(*(s32*)(&L));
        RoadLength[1]=(float)(*(s32*)(&R));
      }
      else return_code=illegal_data;    
    }
    break;
  default:
    return_code=illegal_register;
  }
  
  //�ظ��û�
  Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
  Send_Data_A8_array[index++]=CMD_ModBus_WriteMore;
  Send_Data_A8_array[index++]=return_code;
  cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
  Send_Data_A8_array[index++]=cal_crc&0xff;
  Send_Data_A8_array[index++]=cal_crc>>8;
  FillUartTxBufN(Send_Data_A8_array,index,U_TX_INDEX);
  return 0;      
}

void MOD_BUS_REG_Backup(void)
{
  memcopy((void*)&MOD_BUS_Reg,(void*)&MOD_BUS_Reg_Backup,sizeof(MOD_BUS_REG));  
}

#define MIN_FLASH_RECOVER_TIMER 1000  
u8 RecoverFlashFlag=0;
u16 RecoverFlash_Timeout=0;
void MOD_BUS_REG_MODIFY_Check(void)
{
  if(MOD_BUS_REG_FreshFlag==1)
  {
    MOD_BUS_REG_FreshFlag=0;
    if(memcompare((void*)&MOD_BUS_Reg,(void*)&MOD_BUS_Reg_Backup,sizeof(MOD_BUS_REG))
       !=sizeof(MOD_BUS_REG))
    {
      //�޸Ĳ�����
      if(MOD_BUS_Reg.COMM_BD!=MOD_BUS_Reg_Backup.COMM_BD)
      {
        //����MODBUS ������
        if(MOD_BUS_Reg.COMM_BD<MOD_BUS_BD_LIST_LENGTH)
        {
          Usart2_Init_op(MOD_BUS_BD_LIST[MOD_BUS_Reg.COMM_BD]);
        }
      }
      //��������Ӧ�ĺ�����ȥ��ȡ���ı�
      //���±���
      MOD_BUS_REG_Backup();
      RecoverFlash_Timeout=MIN_FLASH_RECOVER_TIMER;
      RecoverFlashFlag=1;
    }
  }
  if(RecoverFlashFlag)
  {
    if(RecoverFlash_Timeout==0)
    {
      RecoverFlashFlag=0;
#if(MODBUS_PARA_REFLUSH_FLASH_ENABLE)      
      SaveFlashModBusData((void*)&MOD_BUS_Reg);
#if (MODBUS_USER_INTERFACE_PRINTF_DEBUG)
      printf("RECOVER MOD BUS Flash!\r\n");
#endif
#else
#if (MODBUS_USER_INTERFACE_PRINTF_DEBUG)      
      printf("DEBUG_NO_REFRESH!\r\n");
#endif
#endif      
    }
  }
}