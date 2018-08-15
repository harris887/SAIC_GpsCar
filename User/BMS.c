#include "user_inc.h"
#include "string.h"

#define BMS_PRINTF_DEBUG      0
#define BMS_UART_ENUM         CH_BMS
#define U16_ED_SWAP(value)  ((value >> 8) | (value << 8))
#define U32_ED_SWAP(value)  ((value >> 24) | (value << 24) | ((value >> 8) & 0xFF00) | ((value << 8) & 0xFF0000))
u16 BMS_TimeOutCounter = DEFAULT_BMS_READ_START_DELAY;

const u8 BMS_READ_STATUS_ALL[8] = {0x01 ,0x03 ,0x13 ,0x88 ,0x00 ,0x21, 0x01, 0x7C}; //{0x01 ,0x03 ,0x13 ,0x88 ,0x00 ,0x14, 0xC1, 0x6B};
MODBUS_SAMPLE MODBUS_Bms = {
  .MachineState=0,
  .read_success_num=0,
};

BMS_STATUS BMS_St = {
  .RefreshFlag = 0,
  .Valid = 0,
};

u8 BMS_RX_Bytes[128] = {0};

void Analysis_Receive_From_BMS(u8 data,MODBUS_SAMPLE* pMODBUS, void* st)
{
    switch(pMODBUS->MachineState)//初始化 默认 为 00;
    {
    case 0:
      {
        if(data == 0x1)//从机地址
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
        if(data == CMD_ModBus_ReadEx) 
        {
          pMODBUS->MachineState = 0x02; 
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
        if(pMODBUS->read_receive_timer == 1 )
        {
          pMODBUS->Read_Register_Num = pMODBUS->DataBuf[pMODBUS->BufIndex-1];
          if(pMODBUS->Read_Register_Num <= 68)//长度限定
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
          cal_crc=ModBus_CRC16_Calculate(pMODBUS->DataBuf,pMODBUS->Read_Register_Num+3);
              
          pMODBUS->receive_CRC_L = pMODBUS->DataBuf[pMODBUS->BufIndex-2];
          pMODBUS->receive_CRC_H = pMODBUS->DataBuf[pMODBUS->BufIndex-1];
          //if(((cal_crc>>8) == pMODBUS->receive_CRC_H) && ((cal_crc&0xFF) == pMODBUS->receive_CRC_L))
          if(1)
          {
            pMODBUS->err_state = 0x00;//CRC校验正确 
            pMODBUS->read_success_num += 1;
            if(1)
            {
              u8 i;
              u8* bs = BMS_RX_Bytes; 
              BMS_STATUS* bms = (BMS_STATUS*)st;
              memcpy(BMS_RX_Bytes, pMODBUS->DataBuf + 3, 66);
              
              if(1)
              {
                for(i = 0; i < 16; i++) bms->VCELL_MV[i] = Get_BD_U16(&bs);
                bms->BAT_MV = Get_BD_U32(&bs);
                bms->BAT_MA = Get_BD_U32(&bs);
                for(i = 0; i < 3; i++)  bms->BAT_TEMP[i] = Get_BD_U16(&bs);;
                bms->FCC = Get_BD_U32(&bs);;
                bms->RC = Get_BD_U32(&bs);;
                bms->RSOC = Get_BD_U16(&bs);
                bms->CycleCount = Get_BD_U16(&bs);
                bms->PackStatus = Get_BD_U16(&bs);
                bms->BatStatus = Get_BD_U16(&bs);
                bms->PackConfig = Get_BD_U16(&bs);
                bms->ManufactureAccess = Get_BD_U16(&bs);              
              }
              bms->RefreshFlag = 1;
            }
          }    
          else	  
          {
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
            if( pMODBUS->read_receive_timer == 6 )
            {
              u16 cal_crc;
              cal_crc=ModBus_CRC16_Calculate(pMODBUS->DataBuf,6);
              
              pMODBUS->receive_CRC_L = pMODBUS->DataBuf[pMODBUS->BufIndex-2];
              pMODBUS->receive_CRC_H = pMODBUS->DataBuf[pMODBUS->BufIndex-1];
              if(((cal_crc>>8) == pMODBUS->receive_CRC_H) && ((cal_crc&0xFF) == pMODBUS->receive_CRC_L))
              {

              }    
              else	  
              {
                pMODBUS->err_state = 0x04;
              }   
              pMODBUS->BufIndex = 0;  
              pMODBUS->read_receive_timer = 0;  
              pMODBUS->MachineState = 0;   
            }
          }
        break;
      case 0xb:
        {
        
        }
        break;  
      default:
        {
          pMODBUS->MachineState=0;
        }
    }
}

void Check_BMS_Task(void)
{
  static u8 bms_trans_pro=0;
  
  switch(bms_trans_pro)
  {
  case 0:
    {
      if(BMS_TimeOutCounter==0)
      {
        if(BMS_St.RefreshFlag != 0) BMS_St.Valid = 1;
        else BMS_St.Valid = 0;
        BMS_St.RefreshFlag = 0;
        BMS_RS485_TX_ACTIVE();
        BMS_TimeOutCounter = 2;
        bms_trans_pro++;
      }
    }
    break;
  case 1:
    {
      if(BMS_TimeOutCounter==0)
      {
        FillUartTxBuf_NEx((u8*)BMS_READ_STATUS_ALL, sizeof(BMS_READ_STATUS_ALL), BMS_UART_ENUM);
        BMS_TimeOutCounter = 9;
        bms_trans_pro++;
      }
    }
    break;
  case 2:
    {
      if(BMS_TimeOutCounter==0)
      {
        BMS_RS485_RX_ACTIVE();
        BMS_TimeOutCounter = DEFAULT_BMS_READ_CYCLE - 11;
        bms_trans_pro = 0;
      }
    }
    break;
  default: bms_trans_pro = 0;
  }    
}

u16 Get_BD_U16(u8** beam) 
{
  u16 temp;
  temp = (*beam[0] << 8) | (*beam[1] << 0);
  *beam += 2;
  return temp;
}

u32 Get_BD_U32(u8** beam) 
{
  u32 temp;
  temp = ((u32)*beam[0] << 24) | ((u32)*beam[1] << 16) | ((u32)*beam[2] << 8) | ((u32)*beam[3] << 0);
  *beam += 4;
  return temp;
}