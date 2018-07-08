#include "user_inc.h"

#define DEFAULT_WK2124_INIT_DELAY 2000
#define DEFAULT_RD_WK2124_CYCLE   20
u16 WK2124_Timeout = DEFAULT_WK2124_INIT_DELAY;

void SPI2_MasterInit(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = WK2124_CS_PIN ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(WK2124_CS_PORT, &GPIO_InitStructure);  
  
  /* ��������: SCK �� MOSI������������� */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13  | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* ��������:  MISO--��������  */ 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
 
  /* SPI�Ĵ�������---------------------- */
    /* SPI2 --------  ��������:
        - ˫��˫��ȫ˫��ͨ�� 
        - SPI���豸ģʽ
        - 8λ֡�ṹ
        - ʱ�����ո�
        - ���ݲ����ڵڶ�ʱ����
        - �ڲ�NSS�ź���SSIλ����
        - ������Ԥ��ƵֵΪ8
        - ���ݴ����MSBλ��ʼ
        - CRC����ʽΪ7
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

  /*----ʹ�� SPI2 ----------- */
  SPI_Cmd(SPI2, ENABLE);
  
  WK2124_CS_HIGH();
}


u8 Spi2_SendReceiveByte(u8 byte)
{
  /* �ȴ���һ�����ݷ�����*/
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

  /* �������� */
  SPI_I2S_SendData(SPI2, byte);

  /* �ȴ������������ */
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

  /* ���ؽ��յ�����*/
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
  
  //����Ӳ��
  SPI2_MasterInit();
  //���üĴ���
  data[0]=0x3F;//ʹ��4������
  data[1]=0x00;//�����λ4������
  wk2xxx_write_reg(WK2XXX_GPORT,WK2XXX_GENA,data,2);
  data[0]=0x00;//��ֹ4�����ڵ��ж�
  wk2xxx_write_reg(WK2XXX_GPORT,WK2XXX_GIER,data,1);
  
  Delay_ms(100);
  //�ֱ�����4������
  for(i=0;i<4;i++)
  {
    //--------- SPAGE0 ---------
    data[0]=0;
    wk2xxx_write_reg(i,WK2XXX_SPAGE,data,1);
    
    data[0]=0x03;//RX_EN,TX_EN,SLEEP_DISABLE
    data[1]=0x00;//STOP_1BIT,0У��,��У��λ����ͨģʽ���������
    data[2]=0x0C;//0xC0;//���ղ���λ,���Ͳ���λ��ʹ�ܽ���FIFO,ʹ�ܷ���FIFO,
    data[3]=0x00;//��ֹ�����ж�
    wk2xxx_write_reg(i,WK2XXX_SCR,data,4);
    //--------- SPAGE1 ---------
    data[0]=1;
    wk2xxx_write_reg(i,WK2XXX_SPAGE,data,1);    
    //11.0592Mhz����115200bps-0x0005 9600bps-71
    data[0]=0x00;//
    data[1]=71;//0x05
    data[2]=0x00;
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


void WK2124_TransTask(void)
{
  static u8 temp_buf[256];
  u8 data[8],i;
  static u8 bms_trans_pro=0;
  
  switch(bms_trans_pro)
  {
  case 0:
    {
      if(BMS_TimeOutCounter==0)
      {
        Flush_BmsRx();
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
        u8 tx_len = BMS_ReadStatus();
        BMS_TimeOutCounter = tx_len*2+2;
        bms_trans_pro++;
      }
    }
    break;
  case 2:
    {
      if(BMS_TimeOutCounter==0)
      {
        BMS_RS485_RX_ACTIVE();
        BMS_TimeOutCounter = DEFAULT_BMS_READ_CYCLE;
        if(BMS_ResetCheck()) BMS_TimeOutCounter = DEFAULT_BMS_RESET_TIME;
        bms_trans_pro = 0;
      }
    }
    break;
  default: bms_trans_pro = 0;
  }  
  
  //---- RX ----
  if(WK2124_Timeout == 0)
  {
    WK2124_Timeout = DEFAULT_RD_WK2124_CYCLE;
    for(i=0;i<4;i++)
    {
       wk2xxx_read_reg(i,WK2XXX_RFCNT,data,1);
       if(data[0]!=0)
       {
          wk2xxx_read_fifo(i,temp_buf,data[0]);
          
          switch(i)
          {
          case CH_BMS:
            {
              Handle_BmsRx(temp_buf, data[0]);
            }
            break;
          }
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