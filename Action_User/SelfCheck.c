#include "SelfCheck.h"
#include "ELmo.h"

void anticlockwise(void)
{
	VelCrl(1,80000);
	VelCrl(2,80000);
	VelCrl(3,80000);
}	
void clockwise(void)
{
	VelCrl(1,-80000);
	VelCrl(2,-80000);
	VelCrl(3,-80000);
}
void stop(void)
{
	VelCrl(1,0);
	VelCrl(2,0);
	VelCrl(3,0);
}

void beepfan(void)
{
	GPIOE->ODR^=0x04;
}

