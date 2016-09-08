/**
  ******************************************************************************
  * @file    gpio.c
  * @author  Calcus Lee
  * @version V1.0.1
  * @date    9-August-2013
  * @brief   functions of gpio
  ******************************************************************************
**/
#include "gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
/**
  * @brief  set the pins of a specific GPIO group to be input or output driver pin.
  * @param  GPIOx: where x can be A-I.
  * @param  GPIO_Pin: The specific pins you want to select in group GPIOX.
			This parameter can be combination of GPIO_Pin_x where x can be (0..15) @ref GPIO_pins_define
  * @param  GPIO_Mode. the value can be one of the following value
		    GPIO_Mode_IN   
		    GPIO_Mode_OUT 
		    GPIO_Mode_AF  
		    GPIO_Mode_AN
  * @retval None
  * @author Calcus Lee
**/
            
void GPIO_Init_Pins(GPIO_TypeDef * GPIOx,
					uint16_t GPIO_Pin,
					GPIOMode_TypeDef GPIO_Mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	  /* Enable GPIOx, clock */  
  switch((uint32_t)GPIOx)
  {
    case GPIOA_BASE: 
    {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    break;
    }
	case GPIOB_BASE: 
    {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    break;
    }
	case GPIOC_BASE: 
    {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    break;
    }
	case GPIOD_BASE: 
    {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    break;
    }
	case GPIOE_BASE: 
    {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    break;
    }
	case GPIOF_BASE: 
    {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    break;
    }
	case GPIOG_BASE: 
    {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    break;
    }
	case GPIOH_BASE: 
    {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
    break;
    }
	case GPIOI_BASE: 
    {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
    break;
    }	
	default: break;
  }
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode;

	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		
	GPIO_Init(GPIOx,&GPIO_InitStructure);	
}

void BeepInit(void)
{ 
  GPIO_Init_Pins(GPIOE,GPIO_Pin_2,GPIO_Mode_OUT);
}
void LIGHT_GRABFAN_R_INIT(void)
{
	GPIO_Init_Pins(GPIOB,GPIO_Pin_0,GPIO_Mode_IN);                //Unlash_Fan_LightSwitch
}
void LIGHT_GRABPOST_INIT(void)
{
	 GPIO_Init_Pins(GPIOE,GPIO_Pin_9,GPIO_Mode_IN);                //Post_LightSwitch
}
void LIGHT_GRABFAN_L_INIT(void)
{
	 GPIO_Init_Pins(GPIOB,GPIO_Pin_1,GPIO_Mode_IN);                //Grab_Fan_LightSwitch
}
void LIGHT_HIGH_INIT(void)
{
  GPIO_Init_Pins(GPIOE,GPIO_Pin_11,GPIO_Mode_IN); 
}
void SWITCH_END_0_INIT(void)
{
	 GPIO_Init_Pins(GPIOE,GPIO_Pin_0,GPIO_Mode_IN);//行程开关
}
void SWITCH_END_1_INIT(void)
{
	GPIO_Init_Pins(GPIOE,GPIO_Pin_1,GPIO_Mode_IN);
}
void LIGHT_LOWSPEED_INIT(void)
{
	GPIO_Init_Pins(GPIOE,GPIO_Pin_8,GPIO_Mode_IN);   
}
void LED_Init(void)
{    	 
	GPIO_Init_Pins(GPIOA,GPIO_Pin_8,GPIO_Mode_OUT);
}


/*柱子重启按钮初始化*/
void RESTART_POST_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		
	GPIO_Init(GPIOE,&GPIO_InitStructure);	
}

/*出发区重启按钮初始化*/
void RESTART_DEPART_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		
	GPIO_Init(GPIOB,&GPIO_InitStructure);	
}
