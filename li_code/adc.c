#include "stm32f10x.h"
#include <stdio.h>
#include "all.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"		   // 0x40012400+4C
#define ADC1_DR_Address    ((uint32_t)0x4001244C)  //ADC1 DR�Ĵ�������ַ

USART_InitTypeDef USART_InitStructure;  //���ڳ�ʼ���ṹ������
ADC_InitTypeDef ADC_InitStructure;      //ADC��ʼ���ṹ������
DMA_InitTypeDef DMA_InitStructure;      //DMA��ʼ���ṹ������
// ע��ADCΪ12λģ��ת������ֻ��ADCConvertedValue�ĵ�12λ��Ч
__IO uint16_t ADCConvertedValue[5];
//static u16 ADCConvertedValueLocal, Precent = 0, Vp = 0,Vn = 0,VT = 0,Vin = 0,Vi = 0,num=0;

void ADC_GPIO_Configuration(void);


/*static void Delay_ARMJISHU(__IO uint32_t nCount)
{
  for (; nCount != 0; nCount--);
}
*/
void adc_int(void)
{
   
  
  ADC_GPIO_Configuration();
  
  /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
 
  /* DMA1 channel1 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;    //DMA��Ӧ���������ַ
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADCConvertedValue;   //�ڴ�洢����ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	//DMA��ת��ģʽΪSRCģʽ����������Ƶ��ڴ�
  DMA_InitStructure.DMA_BufferSize = 5;		   //DMA�����С��1��,��λΪDMA_MemoryDataSize
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//����һ�����ݺ��豸��ַ��ֹ����
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//����һ�����ݺ�Ŀ���ڴ��ַ����
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //�����������ݿ��Ϊ16λ
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;  //DMA�����ݳߴ磬HalfWord����Ϊ16λ
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;   //ת��ģʽ��ѭ������ģʽ��
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;	//DMA���ȼ���
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;		  //M2Mģʽ����
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);          
  
  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);

  /* Enable ADC1 and GPIOC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
  
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  //������ת��ģʽ
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;		  //����ɨ��ģʽ
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;   //��������ת��ģʽ
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ADC�ⲿ���أ��ر�״̬
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;   //���뷽ʽ,ADCΪ12λ�У��Ҷ��뷽ʽ
  ADC_InitStructure.ADC_NbrOfChannel = 5;	 //����ͨ������1��
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel10 configuration ADCͨ���飬 ��10��ͨ�� ����˳��1��ת��ʱ�� */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_55Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 4, ADC_SampleTime_55Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 5, ADC_SampleTime_55Cycles5);
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);	  //ADC���ʹ��
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);  //����ADC1
  
  /* Enable ADC1 reset calibaration register */   
  ADC_ResetCalibration(ADC1);	  //����У׼
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));  //�ȴ�����У׼���
  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);		//��ʼУ׼
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));	   //�ȴ�У׼���
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);	//����ת����ʼ��ADCͨ��DMA��ʽ���ϵĸ���RAM����

}




void ADC_GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;                       
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}
/*
void adc_get_value(void)
{

    //ADCConvertedValueLocal = ADCConvertedValue[0];
     Vp = ADCConvertedValue[0];
     Vn = ADCConvertedValue[1];	
     VT = ADCConvertedValue[2];
    Vin = ADCConvertedValue[3];	
     Vi = ADCConvertedValue[4];				
	//Precent = (ADCConvertedValueLocal*100/4096);	//����ٷֱ�
    //Voltage = Precent*33;						  // 3.3V�ĵ�ƽ�������Ч��ƽ
	
	switch (num)
	{
		case 0:
			  sprintf(packet_tbuf2, "V+:%5.0d", Vp);
			  packet_sendsize2 = 8;
			  packet_tct2 = 0;
			  packet_T_OK2 = RESET;
			  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);// ʹ�ܷ����ж�  
			break;
		case 1:
			  sprintf(packet_tbuf2, "V-:%5.0d", Vn);
			  packet_sendsize2 = 8;
			  packet_tct2 = 0;
			  packet_T_OK2 = RESET;
			  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);// ʹ�ܷ�����
			break;
		case 2:
			  sprintf(packet_tbuf2, "VT:%5.0d", VT);
			  packet_sendsize2 = 8;
			  packet_tct2 = 0;
			  packet_T_OK2 = RESET;
			  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);// ʹ�ܷ�����
			break;
		case 3:
			  sprintf(packet_tbuf2, "Vin:%5.0d", Vin);
			  packet_sendsize2 = 10;
			  packet_tct2 = 0;
			  packet_T_OK2 = RESET;
			  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);// ʹ�ܷ�����
			break;
		case 4:
			  sprintf(packet_tbuf2, "VI:%5.0d", Vi);
			  packet_sendsize2 = 8;
			  packet_tct2 = 0;
			  packet_T_OK2 = RESET;
			  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);// ʹ�ܷ�����
			break;
		default:
			break;
	}
   if(num<5)num++;
   else num=0;
   

}
 */