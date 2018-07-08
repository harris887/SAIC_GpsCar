/****************************************************************
Copyright(C),2007-2010,AlexCybot (Beijing) Tech.Co.,Ltd.
All Rights Reserved
�� �� ���� Peer_Usart.h
ģ�������� ������������
��    �ߣ� HARRIS
��    ���� V1.2    
�������ڣ�         
ȫ�ֱ�����        
����ģ�飺        
��    ����         
�޸���ʷ��
 1���޸����ڣ�
    �� �� �ˣ� HARRIS
    ����������
****************************************************************/

#ifndef _Usart_h
#define _Usart_h

/******************************************************************************/
/*Ԥ������*/
#include "stm32f10x_lib.h"
#include "Protocol.h"
/*ʹ��Printf*/
#define  PRINTF_EN

#ifdef   PRINTF_EN
  #include "stdio.h"
#endif 

/******************************************************************************/
/*--------------------�궨��--------------------*/

/*���ڷ��ͻ������ĳ���*/
#define USART_TX_BUF_LEN   40
/*���ڽ��ջ������ĳ���*/
#define USART_RE_BUF_LEN   10

/******************************************************************************/
/*--------------------ȫ�ֱ��������--------------------*/
/*���ͻ���������*/
extern u8  Usart_Txbuffer[USART_TX_BUF_LEN];     
/*���ͻ�����������*/
extern u32 Usart_Tx_num;                   
/*���ͻ���������*/
extern u32 Usart_Tx_index;                 

/*�������ݻ�����*/
extern u8  Usart_Rebuffer[USART_RE_BUF_LEN];       
/*���ջ�����������*/
extern u32 Usart_Re_num;                     
/*���ջ���������*/
extern u32 Usart_Re_index;                   

/*��һ�����ܻ��߷��͵��ַ�*/
extern u8 USART_BYTE;

extern u8 CurrentUart;
extern u8 RS485_Timeout;
/******************************************************************************/
/*�ӿں�������*/

/*****************************************************************
�� �� ����    Usart1_Init
����������    оƬ����1�ĳ�ʼ��
��    �ߣ�    HARRIS
�������ڣ�    2009-9-29     
�� �� ��    ��
�� �� ֵ��    ��              
ȫ�ֱ�����    ��
ģ��֧�֣�    GPIO�̼�
*****************************************************************/
extern void Usart1_Init(void);

/*****************************************************************
�� �� ����    Usart3_Init
����������    оƬ����3�ĳ�ʼ��
��    �ߣ�    HARRIS
�������ڣ�    2009-9-29     
�� �� ��    ��
�� �� ֵ��    ��              
ȫ�ֱ�����    ��
ģ��֧�֣�    GPIO�̼�
��    ����
*****************************************************************/
extern void Usart3_Init(void);

/*****************************************************************
�� �� ����    Usart_SendData_AskMode
����������    ���ڷ��͵����ַ�
��    �ߣ�    HARRIS
�������ڣ�    2009-9-29     
�� �� ��    ��������-Ŀǰֻ���� USART1��USART3
�� �� ֵ��    ��              
ȫ�ֱ�����    ��
ģ��֧�֣�    GPIO�̼�
��    ����
*****************************************************************/
extern void Usart_SendData_AskMode(USART_TypeDef* USARTx ,u8 data);

/*****************************************************************
�� �� ����    putchar
����������    ʹ��printf�����ľ���ʵ��
��    �ߣ�    HARRIS
�������ڣ�    2009-9-29     
�� �� ��    ����Ҫ���͵��ַ�
�� �� ֵ��    ��              
ȫ�ֱ�����    ��
ģ��֧�֣�    GPIO�̼�
��    ����
�޸���ʷ��        
 1���޸����ڣ�
    �� �� �ˣ�
    ����������
*****************************************************************/
extern int putchar(int ch);

extern void USART_TX_PROT_TASK(void);
extern void USART_RX_PROT(void);
void SetBaterate(u32 bate);
extern void ShowRollStatus(void);
typedef struct
{
  u8 Intrrupt;
  u8 InIndex;
  u8 OutIndex;
  u8 Buf[256];
}UART_OPTION;
extern UART_OPTION UART_Optx;
extern UART_OPTION UART_Oprx;
extern u8 USART_UpInforEnable;
void UART1_ISR(void);
void UART_Task(void);
void FillUartTxBuf(u8 data);
void Usart2_Init(void);
void UART2_ISR(void);
int printf_U2(u16 len );
extern char printf_U2_buf[256];
#define false 0
#define true 1
extern u16 Uart1RxTime;
extern u16 Uart2RxTime;
extern u16 Uart3RxTime;
extern u16 Uart4RxTime;
extern u16 Uart5RxTime;

extern UART_OPTION UART1_Optx;
extern UART_OPTION UART1_Oprx;
extern UART_OPTION UART2_Optx;
extern UART_OPTION UART2_Oprx;
extern u16 RS485_SLAVE_TX_2_RX_Delay;
extern u8 MOTO_RS485_RX_TX_Timeout;
extern const u16 RS485_SLAVE_TX_2_RX_DELAY_List[9];

u8 Deal_UART_Infor(u8* pINFOR,u8 Length);
u8 CheckSum(u8 *ptr,u8 length);
void FillUartTxBufN(u8* pData,u8 num,u8 U1_2);
void Set_zero(u8* data,u16 length);
u8 Deal_ModBus_Infor(u8* pINFOR,u8 Length);
void Usart2_Init_op(u32 bd);
void DEBUG_COM_Protocol(void);
void UART3_ISR(void);
void Usart3_Init(void);
void Usart4_Init(void);
void Usart5_Init(void);
void UART4_ISR(void);
void UART5_ISR(void);
void Usart4_Init_op(u32 bd);
void Usart1_Init_op(u32 bd);
void Usart2_Init_op(u32 bd);
/**/

#define RS485_1_DIR_PIN_RE       GPIO_Pin_14
#define RS485_1_DIR_R_PORT       GPIOA
#define RS485_1_DIR_PIN_DE       GPIO_Pin_15
#define RS485_1_DIR_D_PORT       GPIOA
#define RS485_1_RX_Active()      RS485_1_DIR_R_PORT->BRR=RS485_1_DIR_PIN_RE;RS485_1_DIR_D_PORT->BRR=RS485_1_DIR_PIN_DE
#define RS485_1_TX_Active()      RS485_1_DIR_R_PORT->BSRR=RS485_1_DIR_PIN_RE;RS485_1_DIR_D_PORT->BSRR=RS485_1_DIR_PIN_DE


#define RS485_2_DIR_PIN_RE       GPIO_Pin_3
#define RS485_2_DIR_R_PORT       GPIOB
#define RS485_2_DIR_PIN_DE       GPIO_Pin_7
#define RS485_2_DIR_D_PORT       GPIOD
#define RS485_2_RX_Active()      RS485_2_DIR_R_PORT->BRR=RS485_2_DIR_PIN_RE;RS485_2_DIR_D_PORT->BRR=RS485_2_DIR_PIN_DE
#define RS485_2_TX_Active()      RS485_2_DIR_R_PORT->BSRR=RS485_2_DIR_PIN_RE;RS485_2_DIR_D_PORT->BSRR=RS485_2_DIR_PIN_DE

extern u8 debug_char;
extern u32 rx5;
#endif

