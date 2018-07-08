#include "Usart.h"
#include "user_inc.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h> 

#define DEBUG_USART USART1 // USART1 UART5

#define RS485_IDEL      0
#define RS485_RX_ENABLE 1
#define RS485_TX_INIT   2
#define RS485_TX_ENABLE 3
u8 MOTO_RS485_RX_TX_Timeout = 0;
u8 MOTO_RS485_RX_TX_STAUTS = RS485_IDEL;
//u8 debug_char = 'B';
/*��һ�����ܻ��߷��͵��ַ�*/
u8 USART_BYTE = 0;//'C'
u8 USART_UpInforEnable = 0;
u16 Uart1RxTime;
u16 Uart2RxTime;
u16 Uart3RxTime;
u16 Uart4RxTime;
u16 Uart5RxTime;
UART_OPTION UART1_Optx={false,0,0};
UART_OPTION UART1_Oprx={true,0,0,};
UART_OPTION UART3_Optx={false,0,0};
UART_OPTION UART3_Oprx={true,0,0,};
UART_OPTION UART4_Optx={false,0,0};
UART_OPTION UART4_Oprx={true,0,0,};
UART_OPTION UART2_Optx={false,0,0};
UART_OPTION UART2_Oprx={true,0,0,};
UART_OPTION UART5_Optx={false,0,0};
UART_OPTION UART5_Oprx={true,0,0,};


/*���ڱ��ش��Ľ��ճ�ʱ(ms)*/
#define B_115200_SILENCE_TIME 2 
#define B_57600_SILENCE_TIME  2
#define B_38400_SILENCE_TIME  2   //5BYTES
#define B_19200_SILENCE_TIME  3   //5BYTES
#define B_9600_SILENCE_TIME   6 //5   //5BYTES
#define B_4800_SILENCE_TIME   10  //5bytes
#define B_2400_SILENCE_TIME   20  //5bytes
#define B_1200_SILENCE_TIME   40  //5bytes

/*����RS485,����2���յ�ת����ʱ*/
#define RS485_SLAVE_TX_2_RX_DELAY_115200_TIME 2
#define RS485_SLAVE_TX_2_RX_DELAY_57600_TIME  2
#define RS485_SLAVE_TX_2_RX_DELAY_38400_TIME  2
#define RS485_SLAVE_TX_2_RX_DELAY_19200_TIME  (1+2)
#define RS485_SLAVE_TX_2_RX_DELAY_9600_TIME   (1+2)
#define RS485_SLAVE_TX_2_RX_DELAY_4800_TIME   (3+2)
#define RS485_SLAVE_TX_2_RX_DELAY_2400_TIME   (7+2)
#define RS485_SLAVE_TX_2_RX_DELAY_1200_TIME   (15+2)

const u16 RS485_SLAVE_TX_2_RX_DELAY_List[9]=
{
  RS485_SLAVE_TX_2_RX_DELAY_115200_TIME,//default
  RS485_SLAVE_TX_2_RX_DELAY_1200_TIME,
  RS485_SLAVE_TX_2_RX_DELAY_2400_TIME,
  RS485_SLAVE_TX_2_RX_DELAY_4800_TIME,
  RS485_SLAVE_TX_2_RX_DELAY_9600_TIME,
  RS485_SLAVE_TX_2_RX_DELAY_19200_TIME,
  RS485_SLAVE_TX_2_RX_DELAY_38400_TIME,
  RS485_SLAVE_TX_2_RX_DELAY_57600_TIME,
  RS485_SLAVE_TX_2_RX_DELAY_115200_TIME
};

u16 Uart1RxTimeReload=B_9600_SILENCE_TIME;
u16 Uart2RxTimeReload=B_9600_SILENCE_TIME;
u16 Uart3RxTimeReload=B_9600_SILENCE_TIME;
u16 Uart4RxTimeReload=B_9600_SILENCE_TIME;
u16 Uart5RxTimeReload=B_19200_SILENCE_TIME;

u16 RS485_SLAVE_TX_2_RX_Delay=RS485_SLAVE_TX_2_RX_DELAY_115200_TIME;

void Usart1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;  
  
  /* ���� USART1 Tx (PA.09) Ϊ����������� */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* ���� USART1 Rx (PA.10) Ϊ�������롣���򲻽Ӵ��ڳ����������ھ�˾�� */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

   /* ���ṹ������ĳ������ */
  USART_Init(USART1, &USART_InitStructure);
  
   /*��������ж� */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
  
   /*����1ʹ��*/
  USART_Cmd(USART1, ENABLE);
}

void Usart1_Init_op(u32 bd)
{
  USART_InitTypeDef USART_InitStructure;  
  
  USART_InitStructure.USART_BaudRate = bd;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

   /* ���ṹ������ĳ������ */
  USART_Init(USART1, &USART_InitStructure);
  
   /*��������ж� */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  
   /*����1ʹ��*/
  USART_Cmd(USART1, ENABLE);
  
}


void Usart2_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;  
  
  GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
  
  /* ���� USART2 Tx (PD.05) Ϊ����������� */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* ���� USART2 Rx (PD.06) Ϊ�������롣���򲻽Ӵ��ڳ����������ھ�˾�� */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPU
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

   /* ���ṹ������ĳ������ */
  USART_Init(USART2, &USART_InitStructure);
  
   /*��������ж� */
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  USART_ITConfig(USART2, USART_IT_TXE, DISABLE);  
  
   /*����1ʹ��*/
  USART_Cmd(USART2, ENABLE);
}

void Usart2_Init_op(u32 bd)
{
  USART_InitTypeDef USART_InitStructure;  
  
  USART_InitStructure.USART_BaudRate = bd;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

   /* ���ṹ������ĳ������ */
  USART_Init(USART2, &USART_InitStructure);
  
   /*��������ж� */
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  USART_ITConfig(USART2, USART_IT_TXE, DISABLE);  
  
   /*����2ʹ��*/
  USART_Cmd(USART2, ENABLE);
  
}

void Usart3_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;  
  
  /* ���� USART3 Tx (PB.10) Ϊ����������� */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* ���� USART3 Rx (PB.11) Ϊ�������롣���򲻽Ӵ��ڳ����������ھ�˾�� */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  USART_InitStructure.USART_BaudRate = 38400;//19200
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

   /* ���ṹ������ĳ������ */
  USART_Init(USART3, &USART_InitStructure);
  
   /*��������ж� */
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
  
   /*����1ʹ��*/
  USART_Cmd(USART3, ENABLE);
}

void Usart4_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure; 
  
  //RS485,RX/TX�л�
  GPIO_InitStructure.GPIO_Pin = RS485_1_DIR_PIN_RE;		  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(RS485_1_DIR_R_PORT, &GPIO_InitStructure); 
  
  GPIO_InitStructure.GPIO_Pin = RS485_1_DIR_PIN_DE;		  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(RS485_1_DIR_D_PORT, &GPIO_InitStructure);   
  RS485_1_RX_Active();  
  
  /* ���� USART4 Tx (PC.10) Ϊ����������� */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* ���� USART4 Rx (PC.11) Ϊ�������롣���򲻽Ӵ��ڳ����������ھ�˾�� */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  USART_InitStructure.USART_BaudRate = 115200;//115200
  USART_InitStructure.USART_WordLength = USART_WordLength_9b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_Even;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;   

   /* ���ṹ������ĳ������ */
  USART_Init(UART4, &USART_InitStructure);
  
   /*��������ж� */
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
  USART_ITConfig(UART4, USART_IT_TXE, DISABLE);
  
   /*����1ʹ��*/
  USART_Cmd(UART4, ENABLE);
}



void Usart5_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure; 
  
  GPIO_InitStructure.GPIO_Pin = RS485_2_DIR_PIN_RE;		  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(RS485_2_DIR_R_PORT, &GPIO_InitStructure); 
  
  GPIO_InitStructure.GPIO_Pin = RS485_2_DIR_PIN_DE;		  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(RS485_2_DIR_D_PORT, &GPIO_InitStructure);   
  RS485_2_RX_Active();    
  
  /* ���� USART5 Tx (PC.12) Ϊ����������� */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* ���� USART5 Rx (PD.2) Ϊ�������롣���򲻽Ӵ��ڳ����������ھ�˾�� */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  USART_InitStructure.USART_BaudRate = 115200;//115200 19200
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
   /* ���ṹ������ĳ������ */
  USART_Init(UART5, &USART_InitStructure);
  
   /*��ֹ�����ж� */
  USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
  USART_ITConfig(UART5, USART_IT_TXE, DISABLE);
  
   /*����1ʹ��*/
  USART_Cmd(UART5, ENABLE);
}

void UART1_ISR(void)
{
  if(USART1->SR&USART_FLAG_RXNE)
  {
    USART_BYTE = USART1->DR;
    UART1_Oprx.Buf[UART1_Oprx.InIndex++] = USART_BYTE;
    //UART1_Oprx.Buf[UART1_Oprx.InIndex++]=USART1->DR;//fill can clear flag
    Uart1RxTime=Uart1RxTimeReload;//20ms
  }
  if((USART1->SR&USART_FLAG_TXE)&&(UART1_Optx.Intrrupt==true))
  {
    USART1->DR=UART1_Optx.Buf[UART1_Optx.OutIndex++];////fill can clear flag
    if(UART1_Optx.OutIndex==UART1_Optx.InIndex)
    {
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
      UART1_Optx.Intrrupt=false;
    }
  }
}

u32 u2_rx_num = 0;
void UART2_ISR(void)
{
  if(USART2->SR&USART_FLAG_RXNE)
  {
    UART2_Oprx.Buf[UART2_Oprx.InIndex++]=USART2->DR;//fill can clear flag
    Uart2RxTime=Uart2RxTimeReload;
    u2_rx_num += 1;
  }
  if((USART2->SR&USART_FLAG_TXE)&&(UART2_Optx.Intrrupt==true))
  {
    USART2->DR=UART2_Optx.Buf[UART2_Optx.OutIndex++];////fill can clear flag
    if(UART2_Optx.OutIndex==UART2_Optx.InIndex)
    {
      USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
      UART2_Optx.Intrrupt=false;
    }
  }
}

void UART3_ISR(void)
{
  if(USART3->SR&USART_FLAG_RXNE)
  {
    UART3_Oprx.Buf[UART3_Oprx.InIndex++]=USART3->DR;//fill can clear flag
    Uart3RxTime=Uart3RxTimeReload;
  }
  if((USART3->SR&USART_FLAG_TXE)&&(UART3_Optx.Intrrupt==true))
  {
    USART3->DR=UART3_Optx.Buf[UART3_Optx.OutIndex++];////fill can clear flag
    if(UART3_Optx.OutIndex==UART3_Optx.InIndex)
    {
      USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
      UART3_Optx.Intrrupt=false;
    }
  }
}

void UART4_ISR(void)
{
  if(UART4->SR&USART_FLAG_RXNE)
  {
    UART4_Oprx.Buf[UART4_Oprx.InIndex++]=UART4->DR;//fill can clear flag
    Uart4RxTime=Uart4RxTimeReload;
  }
  
  if((UART4->SR&USART_FLAG_TXE)&&(UART4_Optx.Intrrupt==true))
  {
    UART4->DR=UART4_Optx.Buf[UART4_Optx.OutIndex++];////fill can clear flag
    if(UART4_Optx.OutIndex==UART4_Optx.InIndex)
    {
      USART_ITConfig(UART4, USART_IT_TXE, DISABLE);
      UART4_Optx.Intrrupt=false;
      
      Modebus_read_cmd_tx_finish=1;
      PGV_TimeOutCounter = 5;//6,5,4      
    }
  }
}
//u32 rx5 = 0;
void UART5_ISR(void)
{
  if(UART5->SR&USART_FLAG_RXNE)
  {
    UART5_Oprx.Buf[UART5_Oprx.InIndex++]=UART5->DR;
    Uart5RxTime=Uart5RxTimeReload;
    //rx5 += 1;
  }
  
  if((UART5->SR&USART_FLAG_TXE)&&(UART5_Optx.Intrrupt==true))
  {
    UART5->DR=UART5_Optx.Buf[UART5_Optx.OutIndex++];
    if(UART5_Optx.OutIndex==UART5_Optx.InIndex)
    {
      USART_ITConfig(UART5, USART_IT_TXE, DISABLE);
      UART5_Optx.Intrrupt=false; 
    }
  }
}

//printf�ĵײ�ʵ��
int putchar(int ch)
{ 
    DEBUG_USART->DR = (u8) ch;  
    while(USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET); 
    return ch; 
}

void FillUartTxBufN(u8* pData,u8 num,u8 U1_2_3)
{
  UART_OPTION* pUART;
  if(U1_2_3==1)     pUART= &UART1_Optx;
  else if(U1_2_3==2)    
  {
    Delay_ms(30);
    pUART= &UART2_Optx;
    //pUART->Buf[pUART->InIndex++]=0xaa;
  }
  else if(U1_2_3==3)    pUART= &UART3_Optx;
  else if(U1_2_3==4)    pUART= &UART4_Optx;
  else    pUART= &UART5_Optx;
  for(u8 i=0;i<num;i++)    pUART->Buf[pUART->InIndex++]=pData[i];
}

void UART_Task(void)
{  
  if(UART1_Optx.Intrrupt==false)
  {
    if(UART1_Optx.OutIndex!=UART1_Optx.InIndex)
    {
      UART1_Optx.Intrrupt=true;
      USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    }
  }

  if(UART2_Optx.Intrrupt==false)
  {
    if(UART2_Optx.OutIndex!=UART2_Optx.InIndex)
    {
      UART2_Optx.Intrrupt=true;
      USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
    }
  }
  
  if(UART3_Optx.Intrrupt==false)
  {
    if(UART3_Optx.OutIndex!=UART3_Optx.InIndex)
    {
      UART3_Optx.Intrrupt=true;
      USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
    }
  }  
  
  if(UART4_Optx.Intrrupt==false)
  {
    if(UART4_Optx.OutIndex!=UART4_Optx.InIndex)
    {
      UART4_Optx.Intrrupt=true;
      USART_ITConfig(UART4, USART_IT_TXE, ENABLE);
    }
  }

  if(UART5_Optx.Intrrupt==false)
  {
    if((UART5_Optx.OutIndex!=UART5_Optx.InIndex)
      &&(MOTO_RS485_RX_TX_STAUTS==RS485_TX_ENABLE))
    {
      UART5_Optx.Intrrupt=true;
      USART_ITConfig(UART5, USART_IT_TXE, ENABLE);
    }
  }   
  
  //-------------- RX -------------------------
  if(UART1_Oprx.InIndex!=UART1_Oprx.OutIndex)
  {
    
  }
  
  if(UART2_Oprx.InIndex!=UART2_Oprx.OutIndex)
  {
    Analysis_Receive_From_A8(UART2_Oprx.Buf[UART2_Oprx.OutIndex++]);
  } 
  
  if(UART3_Oprx.InIndex!=UART3_Oprx.OutIndex)
  {
    Analysis_Receive_From_Dido(UART3_Oprx.Buf[UART3_Oprx.OutIndex++], &MODBUS_Dido, &DIDO_INPUT_Status);
  }
  
  if(UART4_Oprx.InIndex!=UART4_Oprx.OutIndex)
  {
    Receive_From_PGV(0, UART4_Oprx.Buf[UART4_Oprx.OutIndex++]);
  }  
  
  if(UART5_Oprx.InIndex!=UART5_Oprx.OutIndex)
  {
    Analysis_Receive_From_Monitor(UART5_Oprx.Buf[UART5_Oprx.OutIndex++], &MODBUS_Monitor, MONITOR_St);
  }     
  
  if(Uart1RxTime==0)
  {

  }
  
  if(Uart2RxTime==0)
  {
    if(machine_state)
    {
      machine_state=0;
    }
  }   
  
  if(Uart3RxTime==0)
  {
    if(MODBUS_Dido.MachineState)
    {
      MODBUS_Dido.MachineState = 0;
    }
  }
  
  if(Uart4RxTime==0)
  {
    Receive_From_PGV(1,NULL);
  }
 
  if(Uart5RxTime==0)
  {
    if(MODBUS_Monitor.MachineState)
    {
      MODBUS_Monitor.MachineState = 0;
    }
  }  
  
  //����ӿڵ�485���ͽ��յ�ת��
  switch(MOTO_RS485_RX_TX_STAUTS)
  {
  case RS485_IDEL:
    if(MOTO_RS485_RX_TX_Timeout==0)
    {
      RS485_2_RX_Active();//
      MOTO_RS485_RX_TX_STAUTS = RS485_RX_ENABLE;
    }
    break;
  case RS485_RX_ENABLE:
    if(UART5_Optx.OutIndex != UART5_Optx.InIndex)
    {
      RS485_2_TX_Active();
      MOTO_RS485_RX_TX_Timeout = 2;
      MOTO_RS485_RX_TX_STAUTS = RS485_TX_INIT;
    }
    break;
  case RS485_TX_INIT:
    if(MOTO_RS485_RX_TX_Timeout==0)
    {
      MOTO_RS485_RX_TX_STAUTS = RS485_TX_ENABLE;
    }
    break;
  case RS485_TX_ENABLE:  
    if(UART5_Optx.Intrrupt==false)
    {
      MOTO_RS485_RX_TX_Timeout = RS485_SLAVE_TX_2_RX_Delay;
      MOTO_RS485_RX_TX_STAUTS = RS485_IDEL;
    }
    break;
  default: MOTO_RS485_RX_TX_STAUTS = RS485_IDEL;
  }  
  
}