#include "adc.h"
#include "timer.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "math.h"
#include "string.h"
#include "walk.h"

//TOTALLASRNUM=4
static int16_t ADC_Data[10][TOTALLASERNUM];
static int16_t Laser[TOTALLASERNUM];
//pa4 pa5
//PA4  P9 ADC12_IN4     PA5  P3 ADC12_IN5
void ADC1mixed_DMA_Config(void)
{
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef       DMA_InitStructure;
	GPIO_InitTypeDef      GPIO_InitStructure;
	
	/* Enable ADC, DMA2 and GPIO clocks ****************************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	/************************************************************************/
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;     
	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)&(ADC1->DR); 			// peripheral address, = & ADC1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ADC_Data;					// memory address to save DMA data
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;               	// data dirction: peripheral to memory, ie receive maggage from peripheral
	DMA_InitStructure.DMA_BufferSize = 10*TOTALLASERNUM;                          	//the buffer size, in data unit
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//16 bit data
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;        //16 bit data  32??MCU?1?half-word?16 bits
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0, ENABLE);

	/* Configure ADCx Pin (ADC Channel) as analog input -------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_2;   //Track
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5; //底盘激光
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* ADC Common Init **********************************************************/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);
	
		/* ADC1 Init ****************************************************************/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;	//12位精度
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;			//多通道,使用扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;		//连续转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//转换由软件触发
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//数据右对齐
	ADC_InitStructure.ADC_NbrOfConversion = TOTALLASERNUM;				//顺序进行规则转换的ADC通道数目
	ADC_Init(ADC1, &ADC_InitStructure);

	
	/* ADC1 regular channel13 configuration *************************************/
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_144Cycles);	//1,为规则采样顺序值,为ADC的各通道的采样顺序,只使用了一个通道,所以设为1
	#ifdef BLUE_FIELD
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 2, ADC_SampleTime_144Cycles);	//2,为规则采样顺序值,为ADC的各通道的采样顺序,只使用了一个通道,所以设为1
	#elif defined RED_FIELD
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2,  2, ADC_SampleTime_144Cycles);	//2,进柱子时扫墙面的激光，换成蓝厂时改成pc5，通道15
	#endif
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7,  3, ADC_SampleTime_144Cycles);
	/* Enable DMA request after last transfer (Single-ADC mode) */
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	/* Enable ADCx DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADCx */
	ADC_Cmd(ADC1, ENABLE);
	
	ADC_SoftwareStartConv(ADC1);
}

static int  firstFlag=0;
void AverageValue(int PosY)
{	
	uint8_t   i=0,j=0;
	int       post_i=0,post_j=0;
	static int  mutationCount=0;
	static int  ADdataSum[TOTALLASERNUM];
	static uint16_t  tempData[10];
	static uint16_t  prePostLaserValue;
	static int  prePosY;
  
	for(i=0;i<10;i++)
	{
		tempData[i]=ADC_Data[i][1];
	}
	for(j=0;j<TOTALLASERNUM;j++)
	{
		if(j==1)//POST
		{
			 for(post_i=0;post_i<10;post_i++)
			 {
				 for(post_j=0;post_j<10-post_i;post_j++)
				 {
					 if(tempData[post_j]>tempData[post_j+1])
						 swap(&tempData[post_j],&tempData[post_j+1]);
				 }
			 }
			 Laser[j]=(tempData[4]+tempData[5])/2;
		}
		else
		{
			for(i=0;i<10;i++)
			{
				ADdataSum[j]+=ADC_Data[i][j];
			}
			Laser[j]=ADdataSum[j]/10; //2816
			ADdataSum[j]=0;
		}

	}
	
	/* 0号舵机追踪的激光 */
	/* 1号底盘左（右）侧激光   */
	/* 2号靠进柱子时抱柱子用的激光*/
	
	//滤掉突变的激光值
	if(PosY>12700)
	{
			if(mutationCount>=3)
			{
				mutationCount=0;
				prePostLaserValue=Laser[POST];
				prePosY=PosY;
			}
			else
			{
					if(firstFlag==0)
					{
						 SetFirstFlag();
						 prePostLaserValue=Laser[POST];
						 prePosY=PosY;
					}
					else
					{ 
						 if(fabs(fabs(PosY-prePosY)-fabs(prePostLaserValue-Laser[POST]))>100)//突变激光值
						 {
								Laser[POST]=prePostLaserValue;
								mutationCount++;					 
						 }
						 else
						 {
								prePostLaserValue=Laser[POST];
								prePosY=PosY;
						 }
					}
			}
		
	}

}

void SetFirstFlag(void)
{
	firstFlag=1;
}

void ClearFirstFlag(void)
{
	firstFlag=0;
}

uint16_t GetLaserValue(u8 num)
{
	return Laser[num];
}

void swap(uint16_t *a,uint16_t *b)
{
	int temp;
	temp=*a;
	*a=*b;
	*b=temp;
}






