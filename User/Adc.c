#include "user_inc.h"

#define DEFAULT_AD_CHECK_TIME                 32    //100ms��ȡһ�ε�ص�ѹ
#define DEFAULT_BATTERY_VOLT_FILTER_LENGTH    768   //�˲�������
#define DEFAULT_TEMP_AD_FILTER_LENGTH         256   //64
#define DEFAULT_E_CURRENT_AD_FILTER_LENGTH     32
#define DEFAULT_BATTERY_VOLT_STATUS_HOLD_TIME 128 //״̬���ĵĳ�������128��12.8s

#define BATTERY_00_PERCENT_AD_THRESHOLD 1646  //1694  //22.74V
#define BATTERY_10_PERCENT_AD_THRESHOLD 1698  //1735  //23.3V
#define BATTERY_30_PERCENT_AD_THRESHOLD 1780  //1817  //24.4V 
#define BATTERY_50_PERCENT_AD_THRESHOLD 1936  //1980  //26.6v
#define BATTERY_100_PERCENT_AD_THRESHOLD 2130 //2145 //29V-2160 28.8v-2145
/******************************************************************************/
#define CHARGE_MIN_CURRENT_AD_THRESHOLD  1900   //750
#define CHARGE_MAX_CURRENT_AD_THRESHOLD  3016   //1145 2950 

/******************************************************************************/
u16 adc_data[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

u16 AD_CHECK_TimeOut=0;
u16 AD_CHECK_Counter[AD_CHECK_CHANAL_NUM] = {0,0,0};

u16 BatteryVolt_StatusTimer = 0;
u8 BatteryVolt_LowFlag = 0;     //1-��ص�ѹ�ͣ�0-��ص�ѹ����
u16 ChargeCurrentFixFlag = 0;
s16 Temperature_100times = 0;   //�¶�ֵ��100��


const u16 AD_FILTER_LENGTH_List[AD_CHECK_CHANAL_NUM]=
{
  DEFAULT_BATTERY_VOLT_FILTER_LENGTH ,
  DEFAULT_E_CURRENT_AD_FILTER_LENGTH ,
  DEFAULT_TEMP_AD_FILTER_LENGTH ,
};
u32 AD_INTEGRAL_List[AD_CHECK_CHANAL_NUM]={0,0,0};
u16 AD_FILTERED_List[AD_CHECK_CHANAL_NUM]={0,0,0};
u16* AD_SOURCE_List[AD_CHECK_CHANAL_NUM]=
{
  &AD_Batt, &AD_An, &AD_Temp,
};

void DMA1_Channel1_ForAdc1Init(void) 
{ 
    DMA_InitTypeDef DMA_InitStructure; 
    /*��λDMA,��ֹ��������Ĵ���*/
    DMA_DeInit(DMA1_Channel1); 
   
     /*--------DMA�������£�-------------
     ���� ��ַ��ADC1�����ݼĴ���
     �ڴ�ĵ�ַ���û��Զ��������
     �ڴ���ΪĿ�ĵ�ַ
     �������Ĵ�С��2�����֣�������Ҫɨ���ͨ����ȷ����
     �����ַ��������ֹ
     �ڴ��ַ������ʹ�ܣ���Ҫ��
     ����һ�����ݴ�С�����֣�16λ��
     �ڴ�һ�����ݴ�С�����֣�16λ��
     ѭ��ģʽ������Bufferд�����Զ��ص���ʼ��ַ��ʼ���� ����Ҫ��
     ���ȼ����ߣ��Ժ�Ҫ�������ã�
     �ڴ浽�ڴ洫�䣺��ֹ
    -----------------------------------*/
    
    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address; 
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&(adc_data[0]); 
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; 
    /*BufferSize=4����ΪADCת������4��ͨ�� */
    DMA_InitStructure.DMA_BufferSize = 4; 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; 
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; 
    /*ѭ��ģʽ������Bufferд�����Զ��ص���ʼ��ַ��ʼ���� */
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 
    DMA_InitStructure.DMA_Priority = DMA_Priority_High; 
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 
    DMA_Init(DMA1_Channel1, &DMA_InitStructure); 

    /*ʹ��DMA��������ж�*/
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
    
    /*������ɺ�����DMAͨ�� */
    DMA_Cmd(DMA1_Channel1, ENABLE); 
}



/*****************************************************************
�� �� ����    ADC1_ScanModeWithDmaInit
ʵ��������    ADC1��ʼ������Ҫ��;���¶ȼ��ͻҶȼ��
��    �ߣ�    HARRIS
�������ڣ�    2009-9-29     
�� �� ��    ��
�� �� ֵ��    ��              
ȫ�ֱ�����    ��
ģ��֧�֣�    
��    ����
�޸���ʷ��        
 1���޸����ڣ�
    �� �� �ˣ�
    ����������
*****************************************************************/
void Adc_init(void) 
{ 
    ADC_InitTypeDef ADC_InitStructure; 
    GPIO_InitTypeDef GPIO_InitStructure;
    /*��ʼ��DMA*/
    DMA1_Channel1_ForAdc1Init();
    

    GPIO_InitStructure.GPIO_Pin = TEMP_SENSOR_PIN|BATT_PIN|CHARGE_PIN|AN_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(BATT_PORT, &GPIO_InitStructure);
     
    /*-------- ADC1��������-----------
    ADCģʽ������ģʽ
    ɨ��ģʽ��ʹ��
    ����ת��ģʽ����ֹ������ģʽʹ��
    ADC����ģʽ���������
    ���ݶ��뷽ʽ���Ҷ���
    ת�����г��ȣ�2��������Ҫ�趨��
    ----------------------------------*/
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; 
    ADC_InitStructure.ADC_ScanConvMode = ENABLE; 
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;  
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; 
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 
    //����ת�����г���Ϊ
    ADC_InitStructure.ADC_NbrOfChannel = 4;      
    ADC_Init(ADC1, &ADC_InitStructure); 
    
    /*�����û�����Ҫ�����Ҫ��ͨ�� */
     
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_13Cycles5); 
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_13Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_13Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4, ADC_SampleTime_13Cycles5);
    
    /*����ADC��DMA֧�� */
    ADC_DMACmd(ADC1, ENABLE);   
    
    /*ʹ��ADC1 */
    ADC_Cmd(ADC1, ENABLE); 
    
    /*������ADC�Զ�У׼����������ִ��һ�Σ���֤���� */
    ADC_ResetCalibration(ADC1); 
    while(ADC_GetResetCalibrationStatus(ADC1));       
} 


void AD_DMA_IrqHandler(void)
{
  //do nothing
  DMA_ClearITPendingBit(DMA1_IT_TC1);
}



void CheckBatteryVolt_TASK(void)
{
  if(AD_CHECK_TimeOut==0)
  {
    u16 i,temp;
    AD_CHECK_TimeOut = DEFAULT_AD_CHECK_TIME;
    
    //-------- AD �˲� ------------------------------//
    for(i = 0; i < AD_CHECK_CHANAL_NUM; i++)
    {
      temp = *(AD_SOURCE_List[i]);
      // ADֵ����
      if(i == BATTERY_AD_INDEX) temp += AD_BATT_OFFSET;
      
      if(AD_CHECK_Counter[i] < AD_FILTER_LENGTH_List[i])
      {
        AD_FILTERED_List[i] = temp;
        AD_INTEGRAL_List[i] += temp;
      }
      else
      {
         AD_INTEGRAL_List[i] -= (AD_INTEGRAL_List[i] / AD_FILTER_LENGTH_List[i]);
         AD_INTEGRAL_List[i] += temp;
         AD_FILTERED_List[i] = ((AD_INTEGRAL_List[i] + AD_FILTER_LENGTH_List[i] - 1) / AD_FILTER_LENGTH_List[i]);
      }
    }
    //-------------------------------------------------//
    //���� �¶�
    if(1)
    {
      Temperature_100times = (s16)((float)AD_FILTERED_List[TEMP_AD_INDEX] * COFF_AD_2_TEMPRATURE_100TIMES);
      Temperature_100times += TEMPERATURE_OFFSET_100TIMES;
    }
    
    //���������״̬
    if(1) 
    {  
      if((AD_FILTERED_List[E_CURRENT_AD_INDEX] >= CHARGE_MIN_CURRENT_AD_THRESHOLD)
          &&(AD_FILTERED_List[E_CURRENT_AD_INDEX] <= CHARGE_MAX_CURRENT_AD_THRESHOLD))
      {
        ChargeCurrentFixFlag=1;
      }
      else
      {
        ChargeCurrentFixFlag=0;
      }
    }
      
    //��������İٷֱ�
    if(1)
    {
        u32 diff,range;
        if(BatteryVolt_Filterd < BATTERY_00_PERCENT_AD_THRESHOLD)
        {
          //diff=0;
          M_BAT_Precent=0;
        }
        else if(BatteryVolt_Filterd < BATTERY_10_PERCENT_AD_THRESHOLD)
        {
          diff = BatteryVolt_Filterd - BATTERY_00_PERCENT_AD_THRESHOLD;
          range = BATTERY_10_PERCENT_AD_THRESHOLD - BATTERY_00_PERCENT_AD_THRESHOLD;
          M_BAT_Precent = 0 + (diff * 10) / range;
        }   
        else if(BatteryVolt_Filterd < BATTERY_30_PERCENT_AD_THRESHOLD)
        {
          diff = BatteryVolt_Filterd - BATTERY_10_PERCENT_AD_THRESHOLD;
          range = BATTERY_30_PERCENT_AD_THRESHOLD - BATTERY_10_PERCENT_AD_THRESHOLD;
          M_BAT_Precent = 10 + (diff * 20) / range;
        }        
        else if(BatteryVolt_Filterd < BATTERY_50_PERCENT_AD_THRESHOLD)
        {
          diff = BatteryVolt_Filterd - BATTERY_30_PERCENT_AD_THRESHOLD;
          range = BATTERY_50_PERCENT_AD_THRESHOLD - BATTERY_30_PERCENT_AD_THRESHOLD;
          M_BAT_Precent = 30 + (diff * 20) / range;
        }
        else if(BatteryVolt_Filterd<BATTERY_100_PERCENT_AD_THRESHOLD)
        {
          diff = BatteryVolt_Filterd - BATTERY_50_PERCENT_AD_THRESHOLD;
          range = BATTERY_100_PERCENT_AD_THRESHOLD - BATTERY_50_PERCENT_AD_THRESHOLD;
          M_BAT_Precent = 50 + (diff * 50) / range;
        }
        else
        {
          //diff=BATTERY_100_PERCENT_AD_THRESHOLD-BATTERY_00_PERCENT_AD_THRESHOLD;
          //range=BATTERY_100_PERCENT_AD_THRESHOLD-BATTERY_00_PERCENT_AD_THRESHOLD;
          M_BAT_Precent=100;
        }
        
      switch(BatteryVolt_LowFlag)
      {
      case 0://30%������
        {
          if(BatteryVolt_Filterd<BATTERY_30_PERCENT_AD_THRESHOLD)
          {
            BatteryVolt_StatusTimer+=1;
            if(BatteryVolt_StatusTimer>=DEFAULT_BATTERY_VOLT_STATUS_HOLD_TIME)
            {
              BatteryVolt_StatusTimer=0;
              BatteryVolt_LowFlag=1;
            }
          }
          else
          {
            BatteryVolt_StatusTimer=0;
          }      
        }
        break;
      case 1://10%������,30%����
        {
          if(BatteryVolt_Filterd>=BATTERY_30_PERCENT_AD_THRESHOLD)
          {
            BatteryVolt_StatusTimer+=1;
            if(BatteryVolt_StatusTimer>=DEFAULT_BATTERY_VOLT_STATUS_HOLD_TIME)
            {
              BatteryVolt_StatusTimer=0;
              BatteryVolt_LowFlag=0;
            }
          }
          else if(BatteryVolt_Filterd<BATTERY_10_PERCENT_AD_THRESHOLD)
          {
            BatteryVolt_StatusTimer+=1;
            if(BatteryVolt_StatusTimer>=DEFAULT_BATTERY_VOLT_STATUS_HOLD_TIME)
            {
              BatteryVolt_StatusTimer=0;
              BatteryVolt_LowFlag=2;
            }
          }
          else
          {
            BatteryVolt_StatusTimer=0;
          }           
        }
        break;
      case 2://10%����
        {
          if(BatteryVolt_Filterd>=BATTERY_10_PERCENT_AD_THRESHOLD)
          {
            BatteryVolt_StatusTimer+=1;
            if(BatteryVolt_StatusTimer>=DEFAULT_BATTERY_VOLT_STATUS_HOLD_TIME)
            {
              BatteryVolt_StatusTimer=0;
              BatteryVolt_LowFlag=1;
            }
          }
          else
          {
            BatteryVolt_StatusTimer=0;
          }      
        }
        break;
      default: 
        {
          BatteryVolt_StatusTimer=0;
          BatteryVolt_LowFlag=0;
        }
      }            
    }
  }
}