#ifndef __GPIO_H

#define __GPIO_H

#include "stm32f4xx_gpio.h"

#define LIGHT_GRABPOST    GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_9)

#define LIGHT_GRABFAN_R   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)
#define LIGHT_GRABFAN_L   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)

#define LIGHT_LOWSPEED    GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_8)
#define LIGHT_HIGH        GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_11)

#define SWITCH_END_0      GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0)
#define SWITCH_END_1      GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_1)

#define LED_ON           	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
#define LED_OFF          	GPIO_SetBits(GPIOA,GPIO_Pin_8);
#define BEEP_ON         	GPIO_SetBits(GPIOE,GPIO_Pin_2)
#define BEEP_OFF        	GPIO_ResetBits(GPIOE,GPIO_Pin_2)

#define RESTART_POST      GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_15)
#define RESTART_DEPART    GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11)//上面那个行程开关


void LED_Init(void);
void LIGHT_GRABFAN_R_INIT(void);
void LIGHT_GRABPOST_INIT(void);
void LIGHT_GRABFAN_L_INIT(void);
void LIGHT_HIGH_INIT(void);
void SWITCH_END_0_INIT(void);
void SWITCH_END_1_INIT(void);
void LIGHT_LOWSPEED_INIT(void);
void RESTART_POST_INIT(void);
void RESTART_DEPART_INIT(void);
void GPIO_Init_Pins(GPIO_TypeDef * GPIOx,
					uint16_t GPIO_Pin,
					GPIOMode_TypeDef GPIO_Mode);

void BeepInit(void);
	
#endif
