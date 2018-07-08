#ifndef __ADS8431_H_
#define __ADS8431_H_



#if (0)


extern u16 AD_Data_8341[4];
extern u32 Temp_8341;//SHT ÎÂ¶È
extern u32 Humi_8341;//SHT Êª¶È
extern u32 SHT10_Counter_8341;


extern union Temperature SHT_T_8341;
extern union Humidity SHT_H_8341;



extern u16 ReadData[2];//={0,0};
extern u8 K_AD;
void Get_Results_F(void);
void wait(void);
void WriteToReg(u8 ByteData);
void ReadFromReg(u8 n);
#endif

#endif
