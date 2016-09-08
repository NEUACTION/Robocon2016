#ifndef __ADC_H
#define __ADC_H

#include "stm32f4xx.h"


#define BASIC              0
#define POST               1
#define TRACK              2

#define TOTALLASERNUM      3

#define LEFTGET  (GetLaserValue(LEFT)<2000&&GetLaserValue(LEFT)>70)
#define LEFTLOSE (GetLaserValue(LEFT)<70||GetLaserValue(LEFT)>2000)

#define RIGHTGET  (GetLaserValue(RIGHT)<=2000&&GetLaserValue(RIGHT)>70)
#define RIGHTLOSE (GetLaserValue(RIGHT)<70||GetLaserValue(RIGHT)>2000)

void ADC1mixed_DMA_Config(void);
void AverageValue(int PosY);
uint16_t GetLaserValue(u8 num);
void swap(uint16_t *a,uint16_t *b);
void SetFirstFlag(void);
void ClearFirstFlag(void);

#endif
