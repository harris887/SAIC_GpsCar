#include "user_inc.h"

#define DEFAULT_WK2124_INIT_DELAY 1000
#define DEFAULT_RD_WK2124_CYCLE   4
u16 WK2124_Timeout = DEFAULT_WK2124_INIT_DELAY;

u32 WK2124_BD_List[WK_CH_NUM] = {
  38400 , // CH_DIDO
  38400 , // CH_DIDO
  9600  , // 
  9600  , // CH_BMS
};

void SPI2_MasterInit(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = WK2124_CS_PIN ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(WK2124_CS_PORT, &GPIO_InitStructure);  
  
  /* 配置引脚: SCK 和 MOSI，复用推挽输出 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13  | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* 配置引脚:  MISO--浮空输入  */ 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
 
  /* SPI寄存器配置---------------------- */
    /* SPI2 --------  配置如下:
        - 双向双线全双工通信 
        - SPI主设备模式
        - 8位帧结构
        - 时钟悬空高
        - 数据捕获于第二时钟沿
        - 内部NSS信号有SSI位控制
        - 波特率预分频值为8
        - 数据传输从MSB位开始
        - CRC多项式为7
  */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;   
  SPI_InitStructure.SPI_CPHA =SPI_CPHA_1Edge;  
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;//SPI_BaudRatePrescaler_128;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);

  /*----使能 SPI2 ----------- */
  SPI_Cmd(SPI2, ENABLE);
  
  WK2124_CS_HIGH();
}


u8 Spi2_SendReceiveByte(u8 byte)
{
  /* 等待上一个数据发送完*/
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

  /* 发送数据 */
  SPI_I2S_SendData(SPI2, byte);

  /* 等待接收数据完成 */
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

  /* 返回接收的数据*/
  return SPI_I2S_ReceiveData(SPI2);
}


//==============================================================================

#define WK2XXX_PAGE1        1
#define WK2XXX_PAGE0        0

#define WK2XXX_STATUS_PE    1
#define WK2XXX_STATUS_FE    2
#define WK2XXX_STATUS_BRK   4
#define WK2XXX_STATUS_OE    8

u8 wk2xxx_read_reg(u8 port,u8 reg,u8 *dat,u8 len)
{
  WK2124_CS_LOW();
  Spi2_SendReceiveByte(0x40|(port<<4)|reg);   
  while(len--)
  {
    *dat++ = Spi2_SendReceiveByte(0);
  }
  WK2124_CS_HIGH();
  return 0;
}

u8 wk2xxx_write_reg(u8 port,u8 reg,u8 *dat,u8 len)
{
  WK2124_CS_LOW();
  Spi2_SendReceiveByte((port<<4)|reg);   
  while(len--)
  {
    Spi2_SendReceiveByte(*dat++);
  }
  WK2124_CS_HIGH();
  return 0;
}

u8 wk2xxx_read_fifo(u8 port,u8 *dat,u8 len)
{
  WK2124_CS_LOW();
  Spi2_SendReceiveByte(0xC0|(port<<4));   
  while(len--)
  {
    *dat++ = Spi2_SendReceiveByte(0);
  }
  WK2124_CS_HIGH();
  return 0;
}

u8 wk2xxx_write_fifo(u8 port,u8 *dat,u8 len)
{
  WK2124_CS_LOW();
  Spi2_SendReceiveByte(0x80|(port<<4));   
  while(len--)
  {
    Spi2_SendReceiveByte(*dat++);
  }
  WK2124_CS_HIGH();
  return 0;
}

void WK2124_Init(void)
{
  u8 data[16],i;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin = RS485_7_DIR_PIN_RE;		  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(RS485_7_DIR_R_PORT, &GPIO_InitStructure); 
  
  GPIO_InitStructure.GPIO_Pin = RS485_7_DIR_PIN_DE;		  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(RS485_7_DIR_D_PORT, &GPIO_InitStructure);   
  RS485_7_RX_Active(); 
  
  //配置硬件
  SPI2_MasterInit();
  //配置寄存器
  data[0]=0x3F;//使能4个串口
  data[1]=0x00;//软件复位4个串口
  wk2xxx_write_reg(WK2XXX_GPORT,WK2XXX_GENA,data,2);
  data[0]=0x00;//禁止4个串口的中断
  wk2xxx_write_reg(WK2XXX_GPORT,WK2XXX_GIER,data,1);
  
  Delay_ms(100);
  //分别配置4个串口
  for(i=0;i<4;i++)
  {
    //--------- SPAGE0 ---------
    data[0]=0;
    wk2xxx_write_reg(i,WK2XXX_SPAGE,data,1);
    
    data[0]=0x03;//RX_EN,TX_EN,SLEEP_DISABLE
    data[1]=0x00;//STOP_1BIT,0校验,无校验位，普通模式，正常输出
    data[2]=0x0C;//0xC0;//接收不复位,发送不复位，使能接收FIFO,使能发送FIFO,
    data[3]=0x00;//禁止所有中断
    wk2xxx_write_reg(i,WK2XXX_SCR,data,4);
    //--------- SPAGE1 ---------
    data[0]=1;
    wk2xxx_write_reg(i,WK2XXX_SPAGE,data,1);    
    //11.0592Mhz晶振，115200bps-0x0005 9600bps-71
    if(WK2124_BD_List[i] == 115200)
    {
      data[0] = 0;  data[1] = 5;  data[2] = 0;
    }
    else if(WK2124_BD_List[i] == 57600)
    {
      data[0] = 0;  data[1] = 11;  data[2] = 0;
    }
    else if(WK2124_BD_List[i] == 38400)
    {
      data[0] = 0;  data[1] = 17;  data[2] = 0;
    }    
    else if(WK2124_BD_List[i] == 19200)
    {
      data[0] = 0;  data[1] = 35;  data[2] = 0;
    }    
    else if(WK2124_BD_List[i] == 9600)
    {
      data[0] = 0;  data[1] = 71;  data[2] = 0;
    }     
    else
    {
      data[0] = 0;  data[1] = 71;  data[2] = 0;
    }
    wk2xxx_write_reg(i,WK2XXX_BAUD1,data,3);
    
    //--------- SPAGE0 ---------
    data[0]=0;
    wk2xxx_write_reg(i,WK2XXX_SPAGE,data,1);    
  }
}

void FillUartTxBuf_NEx(u8* pData,u8 num,u8 U_0_3)
{
  wk2xxx_write_fifo(U_0_3,pData,num);  
}

u32 wk2124_rx_bytes[WK_CH_NUM] = {0, 0, 0, 0};
void WK2124_TransTask(void)
{
  static u8 temp_buf[256];
  u8 data[8],i;
  static u8 rx_time[WK_CH_NUM] = {0, 0, 0, 0};
  
  //---- RX ----
  if(WK2124_Timeout == 0)
  {
    WK2124_Timeout = DEFAULT_RD_WK2124_CYCLE;
    for(i=0;i<4;i++)
    {
       wk2xxx_read_reg(i,WK2XXX_RFCNT,data,1);
       if(rx_time[i] > 0) rx_time[i] -= 1;
       if(data[0]!=0)
       {
          wk2xxx_read_fifo(i,temp_buf,data[0]);
          
          switch(i)
          {
          case CH_DAM0808:
            {
              wk2124_rx_bytes[CH_DAM0808] += data[0];
              rx_time[CH_DAM0808] = 5;
              for(i = 0; i < data[0]; i++)
              {              
                Analysis_Receive_From_Dido(temp_buf[i], &MODBUS_Dido_0, &DIDO_D_INPUT_Status);
              }
            }
            break;
          case CH_DAM0404:
            {
              wk2124_rx_bytes[CH_DAM0404] += data[0];
              rx_time[CH_DAM0404] = 5;
              for(i = 0; i < data[0]; i++)
              {              
                Analysis_Receive_From_Dido(temp_buf[i], &MODBUS_Dido_1, &DIDO_A_INPUT_Status);
              }
            }
            break;
          case CH_BMS:
            {
              rx_time[CH_BMS] = 5;
              for(i = 0; i < data[0]; i++)
              {
                Analysis_Receive_From_BMS(temp_buf[i], &MODBUS_Bms, &BMS_St);
              }
            }
            break;
          }
       }
    }
    
    if(rx_time[CH_BMS] == 0)
    {
      if(MODBUS_Bms.MachineState)
      {
        MODBUS_Bms.MachineState = 0;
      }
    }  
    
    if(rx_time[CH_DAM0808] == 0)
    {
      if(MODBUS_Dido_0.MachineState)
      {
        MODBUS_Dido_0.MachineState = 0;
      }
    } 

    if(rx_time[CH_DAM0404] == 0)
    {
      if(MODBUS_Dido_1.MachineState)
      {
        MODBUS_Dido_1.MachineState = 0;
      }
    }     
  }
}

#if(0)
void WK2124_Test(void)
{
  u8 td[8];
  u8 data[32],i;
  static u32 NumOfSysTickIntBk;
  static u8 pro=0;
#if 0
  wk2xxx_read_reg(WK2XXX_GPORT,WK2XXX_GENA,data,2);
  wk2xxx_read_reg(WK2XXX_GPORT,WK2XXX_GIER,data+2,2);
  
  //--------- SPAGE0 ---------
  td[0]=0;
  wk2xxx_write_reg(0,WK2XXX_SPAGE,td,1);     
  wk2xxx_read_reg(0,WK2XXX_SPAGE,data+4,11);
  
  //--------- SPAGE1 ---------
  td[0]=1;
  wk2xxx_write_reg(0,WK2XXX_SPAGE,td,1);  
  wk2xxx_read_reg(0,WK2XXX_BAUD1,data+15,5);
   
  for(i=0;i<20;i++)
  {
    printf("0x%x ",data[i]);
  }
  printf("\n");
#else
  
  FillUartTxBuf_NEx("1234567890  ",12,0);
  wk2xxx_read_reg(0,WK2XXX_TFCNT,data,2);
  
  printf("T=%d,R=%d\n",data[0],data[1]);
  

#endif
}
#endif