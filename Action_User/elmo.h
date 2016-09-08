#ifndef __ELMO_H
#define __ELMO_H
#include "stm32f4xx.h"
#define   POS_REL  1    //相对位置方式POS_REL = 1 绝对位置方式POS_ABS = 0
#define   POS_ABS  0 


union can_message
{
	uint8_t data8[4];
	int data32_t;
	float dataf;
};

void elmo_Init(void);
void elmo_Enable(uint8_t ElmoNum);
void elmo_Disable(uint8_t ElmoNum);

void Vel_cfg(uint8_t ElmoNum,uint32_t acc,uint32_t dec) ;
void Pos_cfg(uint8_t ElmoNum,uint32_t acc,uint32_t dec,uint32_t vel) ;
void VelCrl(uint8_t ElmoNum,int vel);
void PosCrl(uint8_t ElmoNum,uint8_t rel_abs,int pos);

void ReadActualPos(uint8_t ElmoNum);

void ReadActualCurrent(uint8_t ElmoNum);
void ReadActualVel(uint8_t ElmoNum);
#endif 


