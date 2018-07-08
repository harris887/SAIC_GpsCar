#include "user_inc.h"

#if (0)
u16 ReadData[2] = {0,0};

u8 Send_Data_To_Master_ADS8341[64];//by johnson	  20140407 移植时重定义。


extern float t;
extern float h;
extern u8 Heat;


u16 AD_Data_8341[4] = {0};
u32 Temp_8341 = 0;//SHT 温度
u32 Humi_8341 = 0;//SHT 湿度
u32 SHT10_Counter_8341 = 0;


extern union Float_TO_U8 Temp_main;//发送给PC口的数据
extern union Float_TO_U8 Humi_main;//发送给PC口的数据
extern union Float_TO_U8 AD_8341_PC[4];//发送给PC口的数据




union Temperature SHT_T_8341;
union Humidity SHT_H_8341;

union Temperature
{
    float T_f;
    u8 array[4];
}SHT_T_8341;

union Humidity
{
    float H_f;
    u8 array[4];
}SHT_H_8341;


void wait(void)
{
    u8 i=0;
    for(i=0;i<8;i++)
    __nop();
}

void WriteToReg(u8 ByteData) // write ByteData to the register
{
	u8 temp;
	u8 i;
    GPIO_SetBits(GPIOB,GPIO_Pin_13);//拉高时钟	
    GPIO_ResetBits(GPIOC,GPIO_Pin_12);//CS=0;
	temp=0x80;
	for(i=0;i<8;i++)
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_13);//SCLOCK=0;
		if((temp & ByteData)==0)
		{
			GPIO_ResetBits(GPIOB,GPIO_Pin_15);//DIN=0;
		}
		else
		{
			GPIO_SetBits(GPIOB,GPIO_Pin_15);;//DIN=1;
		}
        wait();
        GPIO_SetBits(GPIOB,GPIO_Pin_13);//	SCLOCK=1;
        wait();
		temp=temp>>1;
	}
//	GPIO_SetBits(GPIOB,GPIO_Pin_12);//CS=1;
        GPIO_ResetBits(GPIOB,GPIO_Pin_13);//	SCLOCK=0;
        wait();
        GPIO_SetBits(GPIOB,GPIO_Pin_13);//	SCLOCK=1;
        wait(); wait();
}

void ReadFromReg(u8 nByte) // nByte is the number of bytes which need to be read
{
	int i,j;
	u8 temp,rec_bit_uchar;
//    GPIO_ResetBits(GPIOB,GPIO_Pin_12);//CS=0;

    GPIO_SetBits(GPIOB,GPIO_Pin_13);//拉高时钟	
    wait();
	temp=0;
	for(i=0; i<nByte; i++)
	{
		for(j=0; j<8; j++)
		{
		    GPIO_ResetBits(GPIOB,GPIO_Pin_13);//SCLOCK=0;
//			rec_bit_uchar=BitRdPortI(PBDR,SDI_PB);
            rec_bit_uchar=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14);
			rec_bit_uchar&=0x01;
			if(rec_bit_uchar==0)
			{
				temp=temp<<1;
			}
    		else
			{
				temp=temp<<1;
				temp=temp+0x01;
			}
            wait();
            GPIO_SetBits(GPIOB,GPIO_Pin_13);//SCLOCK=1;
			wait();
		}
		ReadData[i]=temp;
		temp=0;
	}
	GPIO_SetBits(GPIOC,GPIO_Pin_12);//CS=1;
}

/*************************************************************
读取AD数据   by johnson  20150402
**************************************************************/
void AD_F(void)
{
    //u8 i = 0;
    //float temp_11 = 0.0;
    

    WriteToReg(0xE7);//CH3    传感器4
    ReadFromReg(2);    
    AD_Data_8341[3] = ReadData[0]*256+ReadData[1];

    WriteToReg(0xA7);//CH2    传感器3
    ReadFromReg(2);    
    AD_Data_8341[2] = ReadData[0]*256+ReadData[1];

    WriteToReg(0xD7);//CH1    传感器2
    ReadFromReg(2);    
    AD_Data_8341[1] = ReadData[0]*256+ReadData[1];

    WriteToReg(0x97);//CH0    传感器1
    ReadFromReg(2);    
    AD_Data_8341[0] = ReadData[0]*256+ReadData[1];

#if 0  
    temp_11 = (float)AD_Data_8341[0];
    AD_8341_PC[0].f = temp_11;//by johnson @20150421 for send to pc
    temp_11 = (float)AD_Data_8341[1];
    AD_8341_PC[1].f = temp_11;//by johnson @20150421 for send to pc
    temp_11 = (float)AD_Data_8341[2];
    AD_8341_PC[2].f = temp_11;//by johnson @20150421 for send to pc
    temp_11 = (float)AD_Data_8341[3];
    AD_8341_PC[3].f = temp_11;//by johnson @20150421 for send to pc
#endif
    
    AD_8341_PC[0].f = (float)AD_Data_8341[0];//by johnson @20150421 for send to pc

    AD_8341_PC[1].f = (float)AD_Data_8341[1];//by johnson @20150421 for send to pc

    AD_8341_PC[2].f = (float)AD_Data_8341[2];//by johnson @20150421 for send to pc

    AD_8341_PC[3].f = (float)AD_Data_8341[3];//by johnson @20150421 for send to pc
    
    
#if 0
		for(i=0;i<4;i++)
		{
			printf(" %d  =%d\t\n",i,AD_Data_8341[i]);
		}
#endif 

#if 0
		for(i=0;i<4;i++)
		{
			printf(" %d  =%f\t\n",i,AD_8341_PC[i].f);
		}
#endif   



#if 0
    if(SHT10_Counter_8341%8==0)
    {
        if(SHT10_Counter_8341%8000==0)//每2000秒SHT10复位一次
        {
            SHT10_reset();
        }
        Temp_8341=SHT10_Measure(0x03,230);//进行温度测量
        t=SHT_T_8341.T_f = SHT10_Convert_Tempeture14bit(Temp_8341);//转换为浮点数

        Humi_8341=SHT10_Measure(0x05,65);//进行湿度测量
        h=SHT_H_8341.H_f = SHT10_Convert_Humidity12bit(Humi_8341,SHT_T_8341.T_f);//转换为浮点数
    }
    SHT10_Counter_8341++;
#endif
    
 // 如果打印信息，时间不够。                        
#if 0   
        printf("Temp_8341 =%f\t\n",t);  
        printf("Humi_8341 =%f\t\n",h); 
#endif


#if 0
    for(i=0;i<4;i++)
    {
        printf(" %d  =%d\t\n",i,AD_Data_8341[i]);
    }
#endif    
        
}
#endif