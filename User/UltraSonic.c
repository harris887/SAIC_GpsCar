#include "user_inc.h"
#include <string.h>


u16 UltraSonicCheckTimer=DEFAULT_ULTRA_SONIC_CHECK_TIME_IN_MS;
MODBUS_SAMPLE UltraSonic_Modbus[2]={
  {.MachineState=0},
  {.MachineState=0}
};
const u8 MODBUS_READ_ULTRA_SONIC[8]={0x01 ,0x04 ,0x00 ,0x05 ,0x00 ,0x04 ,0xE1 ,0xC8};
u16 UltraSonicDistance[ULTRA_SONIC_SENSOR_NUM]=
{
  DEFAULT_ULTRA_SONIC_DISTANCE,DEFAULT_ULTRA_SONIC_DISTANCE,DEFAULT_ULTRA_SONIC_DISTANCE,
  DEFAULT_ULTRA_SONIC_DISTANCE,DEFAULT_ULTRA_SONIC_DISTANCE,DEFAULT_ULTRA_SONIC_DISTANCE,
};

u16 US_0_Timeout=DEFAULT_ULTRA_SONIC_NO_DATA_TIME_OUT;
u16 US_1_Timeout=DEFAULT_ULTRA_SONIC_NO_DATA_TIME_OUT;

void Analysis_Receive_From_UltraSonic(u8 data,MODBUS_SAMPLE* pMODBUS,u16* pUltraSonicDistance)
{
    switch(pMODBUS->MachineState)//初始化 默认 为 00;
    {
    case 0:
      {
        if(data == DEFAULT_MODE_BUS_ULTRA_SONIC_ADDR)//从机地址
        {
          pMODBUS->MachineState = 0x01;
          pMODBUS->BufIndex = 0;
          pMODBUS->DataBuf[pMODBUS->BufIndex++] = data;
        }
        else
        {
          pMODBUS->MachineState = 0x0B;//缓冲数据区域清零要处理，中间数据为01，误认为是要从机地址。
          pMODBUS->BufIndex = 0;
        }  
      }
      break;
      case 0x01:
      {	 
        pMODBUS->DataBuf[pMODBUS->BufIndex++] = data;
        if(data == CMD_ModBus_Read) //执行读取单个或多个寄存器命令  0x04 
        {
          pMODBUS->MachineState = 0x02; 
          pMODBUS->ModBus_CMD = data;
          pMODBUS->read_receive_timer = 0;
        }
        else if((data == CMD_ModBus_Write) || (data == CMD_ModBus_WriteMore))
        {
          pMODBUS->MachineState = 0x04; 
          pMODBUS->ModBus_CMD = data;
          pMODBUS->read_receive_timer = 0;
        }
        else
        { 
          pMODBUS->MachineState = 0x0B;
          pMODBUS->BufIndex = 0;
        }
      }
      break;
      case 0x02: //read part 00
      {    
        //接收到读功能的字节数
        pMODBUS->DataBuf[pMODBUS->BufIndex++] = data;
        pMODBUS->read_receive_timer++;
        if(pMODBUS->read_receive_timer == 2 )
        {
          pMODBUS->Read_Register_Num 
                  = pMODBUS->DataBuf[pMODBUS->BufIndex-2]*256 \
                    + pMODBUS->DataBuf[pMODBUS->BufIndex-1];
          if(pMODBUS->Read_Register_Num <= 16)//长度限定
          {
            pMODBUS->MachineState = 0x03;
          }
          else
          {
            pMODBUS->MachineState = 0x00;
          }
          pMODBUS->read_receive_timer = 0;
        } 
      }
      break;
      case 0x03: //read part 01
      {   
        pMODBUS->DataBuf[pMODBUS->BufIndex++] = data;
        pMODBUS->read_receive_timer++;
        if(pMODBUS->read_receive_timer >= (pMODBUS->Read_Register_Num+2))
        {
          u16 cal_crc;
          cal_crc=ModBus_CRC16_Calculate(pMODBUS->DataBuf,pMODBUS->Read_Register_Num+4);
              
          pMODBUS->receive_CRC_L = pMODBUS->DataBuf[pMODBUS->BufIndex-2];
          pMODBUS->receive_CRC_H = pMODBUS->DataBuf[pMODBUS->BufIndex-1];
          if(((cal_crc>>8) == pMODBUS->receive_CRC_H) 
                  && ((cal_crc&0xFF) == pMODBUS->receive_CRC_L))
          {
            pMODBUS->err_state = 0x00;//CRC校验正确 
            //正确读到数据
            //AckReadCmdToMaster(&Receive_Data_From_UltraSonic[4],
            //                         UltraSonic_Modbus.Read_Register_Num);
            pUltraSonicDistance[0]=(pMODBUS->DataBuf[6]<<8)|pMODBUS->DataBuf[7];
            pUltraSonicDistance[1]=(pMODBUS->DataBuf[8]<<8)|pMODBUS->DataBuf[9];
            pUltraSonicDistance[2]=(pMODBUS->DataBuf[10]<<8)|pMODBUS->DataBuf[11];
            
            if(pMODBUS==&UltraSonic_Modbus[0])
            {
              US_0_Timeout=DEFAULT_ULTRA_SONIC_NO_DATA_TIME_OUT;  
            }
            else
            {
              US_1_Timeout=DEFAULT_ULTRA_SONIC_NO_DATA_TIME_OUT;
            }
          }    
          else	  
          {
             //AckModBusCrcError(UltraSonic_Modbus.ModBus_CMD);
             pMODBUS->err_state = 0x04;
          }   
           pMODBUS->BufIndex = 0;  
           pMODBUS->read_receive_timer = 0;  
           pMODBUS->MachineState = 0x00;                
          }
        }
        break;
        case 0x04: //write
          {
            pMODBUS->DataBuf[pMODBUS->BufIndex++] = data;
            pMODBUS->read_receive_timer++;
            if( pMODBUS->read_receive_timer == 3 )
            {
              u16 cal_crc;
              cal_crc=ModBus_CRC16_Calculate(pMODBUS->DataBuf,3);
              
              pMODBUS->receive_CRC_L = pMODBUS->DataBuf[pMODBUS->BufIndex-2];
              pMODBUS->receive_CRC_H = pMODBUS->DataBuf[pMODBUS->BufIndex-1];
              if(((cal_crc>>8) == pMODBUS->receive_CRC_H) 
                  && ((cal_crc&0xFF) == pMODBUS->receive_CRC_L))
              {
                //AckWriteCmdToMaster(UltraSonic_Modbus.ModBus_CMD,
                //                    Receive_Data_From_UltraSonic[2]);
              }    
              else	  
              {
                //AckModBusCrcError(UltraSonic_Modbus.ModBus_CMD);
                pMODBUS->err_state = 0x04;
              }   
              pMODBUS->BufIndex = 0;  
              pMODBUS->read_receive_timer = 0;  
              pMODBUS->MachineState = 0;   
            }
          }
        break;
      case 0xb:break;  
      default:
        {
          pMODBUS->MachineState=0;
        }
    }
}



void Check_UltraSonic_TASK(void)
{
  if(UltraSonicCheckTimer==0)
  {
    UltraSonicCheckTimer = DEFAULT_ULTRA_SONIC_CHECK_TIME_IN_MS;
    FillUartTxBufN((u8*)MODBUS_READ_ULTRA_SONIC,sizeof(MODBUS_READ_ULTRA_SONIC),2);
    //FillUartTxBufN((u8*)MODBUS_READ_ULTRA_SONIC,sizeof(MODBUS_READ_ULTRA_SONIC),5);
    FillUartTxBufN((u8*)MODBUS_READ_ULTRA_SONIC,sizeof(MODBUS_READ_ULTRA_SONIC),4);
  
    if(US_0_Timeout==0)
    {
      UltraSonicDistance[0]=DEFAULT_ULTRA_SONIC_DISTANCE;   
      UltraSonicDistance[1]=DEFAULT_ULTRA_SONIC_DISTANCE;   
      UltraSonicDistance[2]=DEFAULT_ULTRA_SONIC_DISTANCE;   
    }
    if(US_1_Timeout==0)
    {
      UltraSonicDistance[3]=DEFAULT_ULTRA_SONIC_DISTANCE;   
      UltraSonicDistance[4]=DEFAULT_ULTRA_SONIC_DISTANCE;   
      UltraSonicDistance[5]=DEFAULT_ULTRA_SONIC_DISTANCE;     
    }
    
    if(1)
    {
      u8 i;
      for(i=0;i<ULTRA_SONIC_SENSOR_NUM;i++)
      {
        if(UltraSonicDistance[i]<=
           ((i<3)?MOD_BUS_Reg.US_FORWARD_Thres:MOD_BUS_Reg.US_BACKWARD_Thres))
        {
          LIGHT_SENSOR_Flag |= (1<<(i+2));
          M_LightSensorStatus[i+2]=1;
        }
        else
        {
          LIGHT_SENSOR_Flag &= ~(1<<(i+2));
          M_LightSensorStatus[i+2]=0;
        }
      }
    }
  }
}

