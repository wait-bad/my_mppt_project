#include "adc.h"

ADC_InitType ADC_InitStructure;
DMA_InitType DMA_InitStructure;

 __IO  uint16_t ADCConvertedValue[5];
 

void ADC_RCC_IO_Init(void)  
{
	GPIO_InitType GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	
	/* ADCCLK = PCLK2/6 */
	RCC_ADCCLKConfig(RCC_APB2CLK_Div6);
	
	/* Enable peripheral clocks ------------------------------------------------*/
	/* Enable DMA1 clocks */
	RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_DMA1, ENABLE);

	/* Enable ADC1 and GPIOA clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_ADC1 , ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOA , ENABLE);

  
  /* Configure ADC pin as analog input */
	GPIO_InitStructure.GPIO_Pins = GPIO_Pins_0 | GPIO_Pins_1 |GPIO_Pins_4 | GPIO_Pins_5 | GPIO_Pins_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

 
void   ADC_DMAConfig(void)  
{
		/* DMA1 channel1 configuration ----------------------------------------------*/
		DMA_Reset(DMA1_Channel1);
		DMA_DefaultInitParaConfig(&DMA_InitStructure);
		DMA_InitStructure.DMA_PeripheralBaseAddr    = (uint32_t)&ADC1->RDOR;
		DMA_InitStructure.DMA_MemoryBaseAddr        = (uint32_t)ADCConvertedValue;
		DMA_InitStructure.DMA_Direction             = DMA_DIR_PERIPHERALSRC;
		DMA_InitStructure.DMA_BufferSize            = 5;
		DMA_InitStructure.DMA_PeripheralInc         = DMA_PERIPHERALINC_DISABLE;
		DMA_InitStructure.DMA_MemoryInc             = DMA_MEMORYINC_ENABLE;
		DMA_InitStructure.DMA_PeripheralDataWidth   = DMA_PERIPHERALDATAWIDTH_HALFWORD;
		DMA_InitStructure.DMA_MemoryDataWidth       = DMA_MEMORYDATAWIDTH_HALFWORD;
		DMA_InitStructure.DMA_Mode                  = DMA_MODE_CIRCULAR;
		DMA_InitStructure.DMA_Priority              = DMA_PRIORITY_HIGH;
		DMA_InitStructure.DMA_MTOM                  = DMA_MEMTOMEM_DISABLE;
		DMA_Init(DMA1_Channel1, &DMA_InitStructure);
		/* Enable DMA1 channel1 */
		DMA_ChannelEnable(DMA1_Channel1, ENABLE);
}


void   ADC_Config(void)
{ 
 /* ADC1 configuration ------------------------------------------------------*/
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Mode              = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanMode          = ENABLE;
	ADC_InitStructure.ADC_ContinuousMode    = ENABLE;
	ADC_InitStructure.ADC_ExternalTrig      = ADC_ExternalTrig_None;
	ADC_InitStructure.ADC_DataAlign         = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NumOfChannel      = 5;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/* ADC1 regular channels configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28_5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_28_5); 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_28_5); 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 4, ADC_SampleTime_28_5); 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 5, ADC_SampleTime_28_5); 
	ADC_ExternalTrigConvCtrl(ADC1, ENABLE);
	/* Enable ADC1 DMA */
	ADC_DMACtrl(ADC1, ENABLE);
	
	/* Enable ADC1 */
	ADC_Ctrl(ADC1, ENABLE);

	/* Enable ADC1 reset calibration register */   
	ADC_RstCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));

	/* Start ADC1 calibration */
	ADC_StartCalibration(ADC1);
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));

	/* Start ADC1 Software Conversion */ 
	ADC_SoftwareStartConvCtrl(ADC1, ENABLE);
}

void   UserADC_init(void)
{
	ADC_RCC_IO_Init();
	ADC_DMAConfig();
	ADC_Config();
}

uint16_t   GetADCVal(uint8_t adcindex)
{
		return  ADCConvertedValue[adcindex];
}