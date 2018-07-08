#include "user_inc.h"
#include "string.h"

#define BMS_PRINTF_DEBUG      0
u16 BMS_TimeOutCounter = DEFAULT_BMS_READ_START_DELAY;
const u8* READ_PROTECT_PARAM = ":000100000E09~";
const u8 BMS_READ_STATUS_ALL[6] = {0x06 ,0x01 ,0x10 ,0x00 ,0x00 ,0x17};
#define BMS_TX_BUF_SIZE   32
#define BMS_RX_BUF_SIZE   32
u8 BMS_TX_Buf[BMS_TX_BUF_SIZE];
u8 BMS_RX_Buf[BMS_RX_BUF_SIZE];
u8 BMS_RX_BufIndex = 0;
BMS_STATUS BMS_St = {0,0,0,0,0,0,0,0,0,0};

void Byte2HexAscii(u8 value,char* str)
{
  const char* hex="0123456789ABCDEF";
  str[0] = hex[value>>4];
  str[1] = hex[value&15];
}

u8 BMS_Crc(u8* data, u16 len)
{
  u16 i;
  u8 tmp=0;
  for(i=1;i<(len-3);i++)
  {
    tmp += data[i];
  }
  return tmp^0xFF;
}

u8 BMS_CheckSum(u8* data, u8 len)
{
  u8 tmp=0,i;
  for(i=0; i<len; i++)
  {
    tmp += data[i];
  }
  return tmp;
}

u8 NewBMSFrame(u8 cmd, u8* buf)
{
  u16 len;
  u8 crc;
  BMS_FRAME* pF = (BMS_FRAME*)buf;
  pF->SOI = BMS_SOI;
  Byte2HexAscii(DEFAULT_BMS_ADDR, pF->Addr);
  Byte2HexAscii(cmd, pF->Cmd);
  Byte2HexAscii(DEFAULT_BMS_VAR, pF->Ver);
  len = sizeof(BMS_FRAME);
  
  switch(cmd)
  {
  case CMD_GET_PROTECT_PARAM:
    {
      buf[len-1] = BMS_EOI;
      Byte2HexAscii(len>>8, pF->Len);
      Byte2HexAscii(len&0xFF, pF->Len+2);     
      crc = BMS_Crc(buf, len);
      Byte2HexAscii(crc, buf+len-3); 
    }
    break;
  }
  return len;
}

u8 BMS_ReadStatus(void)
{
  u8 len = sizeof(BMS_READ_STATUS_ALL);
  FillUartTxBuf_NEx((u8*)BMS_READ_STATUS_ALL, len, CH_BMS);
  BMS_St.tx_num += 1;
  return len;
}

void Handle_BmsRx(u8* data, u8 len)
{
  if((len+BMS_RX_BufIndex)<=BMS_RX_BUF_SIZE)
  {
    memcpy(BMS_RX_Buf+BMS_RX_BufIndex, data, len);
    BMS_RX_BufIndex += len;
    if(BMS_RX_BufIndex == sizeof(BMS_STATUS_FRAME))
    {
      u8 valid=0;
      BMS_STATUS_FRAME* pBMS_St = (BMS_STATUS_FRAME*)BMS_RX_Buf;
      if(pBMS_St->check_sum == BMS_CheckSum(BMS_RX_Buf, BMS_RX_BufIndex-1))
        valid += 1;
      if(pBMS_St->func_code == BMS_FUNC_CODE_STATUS)
        valid += 1;
      if(valid == 2)
      {
        BMS_St.ack_num += 1;
        BMS_St.voltage_mv = (pBMS_St->voltage_h<<8) | pBMS_St->voltage_l;
        BMS_St.curr_ma = (pBMS_St->curr_h<<8) | pBMS_St->curr_l;
        BMS_St.out_ele_num = (pBMS_St->out_ele_num_h<<8) | pBMS_St->out_ele_num_l;
        BMS_St.in_ele_num = (pBMS_St->in_ele_num_h<<8) | pBMS_St->in_ele_num_l;
        BMS_St.status = (pBMS_St->status_h<<8) | pBMS_St->status_l;
        BMS_St.temprature = pBMS_St->temprature;
        BMS_St.capacity = pBMS_St->capacity;
      }
      BMS_RX_BufIndex -= sizeof(BMS_STATUS_FRAME);
    }
  }
}

u16 Byte2HexStr(u8* data, u8 len, u8* str)
{
  u8 i;
  const char* hex_str = "0123456789ABCDEF";
  for(i=0; i<len; i++)
  {
    *str++ = hex_str[data[i]>>4];
    *str++ = hex_str[data[i]&15];
  }
  *str++ = '\n'; //'\0'
  return len*2+1;
}

void Flush_BmsRx(void)
{
#if (BMS_PRINTF_DEBUG)
  static u8 buf[256];  
  if(USART_BYTE == 'M')
  {
    if(BMS_RX_BufIndex != 0)
    {
      u16 strlen = Byte2HexStr(BMS_RX_Buf, BMS_RX_BufIndex, buf);
      FillUartTxBufN(buf, strlen, 1);
    }
  }
#endif
  BMS_RX_BufIndex = 0;
}

u8 BMS_ResetCheck(void)
{
  if(BMS_St.tx_num >= (BMS_St.ack_num+DEFAULT_BMS_RESET_LOSE_NUM))
  {
    BMS_St.tx_num = BMS_St.ack_num;
    BMS_St.reset_num += 1;
    return 1;
  }
  return 0;
}