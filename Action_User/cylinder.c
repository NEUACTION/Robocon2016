#include "cylinder.h"
#include "stm32f4xx_can.h"
#include "string.h"

//3号板
#define Cylinder_ID 100

//0|01|000|11|       依次是空闲位，板号位（1-2），接口位（0-7），开关位
#define Cylinder_0_on  0x63
#define Cylinder_0_off 0x60

#define Cylinder_1_on  0x67
#define Cylinder_1_off 0x64

#define Cylinder_2_on  0x6B
#define Cylinder_2_off 0x68

#define Cylinder_3_on  0x6F
#define Cylinder_3_off 0x6C

#define Cylinder_4_on  0x73
#define Cylinder_4_off 0x70

#define Cylinder_5_on  0x77
#define Cylinder_5_off 0x74

#define Cylinder_6_on  0x7B
#define Cylinder_6_off 0x78

#define Cylinder_7_on  0x7F
#define Cylinder_7_off 0x7C

char array[8] = {0};

void Cylinder_Trans(char *CylinderMovement)
{
	uint8_t mbox_cylinder;
	CanTxMsg TxMessage_cylinder;
	
	TxMessage_cylinder.StdId=Cylinder_ID;					 // standard identifier=0
	TxMessage_cylinder.ExtId=Cylinder_ID;					 // extended identifier=StdId
	TxMessage_cylinder.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage_cylinder.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage_cylinder.DLC=8;						 // 发送两帧信息
	
	TxMessage_cylinder.Data[0] = CylinderMovement[0];		   //气缸初始状态
	TxMessage_cylinder.Data[1] = CylinderMovement[1];		   //气缸初始状态
	TxMessage_cylinder.Data[2] = CylinderMovement[2];		   //气缸初始状态
	TxMessage_cylinder.Data[3] = CylinderMovement[3];		   //气缸初始状态
	TxMessage_cylinder.Data[4] = CylinderMovement[4];		   //气缸初始状态
	TxMessage_cylinder.Data[5] = CylinderMovement[5];		   //气缸初始状态
	TxMessage_cylinder.Data[6] = CylinderMovement[6];		   //气缸初始状态
	TxMessage_cylinder.Data[7] = CylinderMovement[7];		   //气缸初始状态

       
	mbox_cylinder= CAN_Transmit(CAN1, &TxMessage_cylinder);   

	while((CAN_TransmitStatus(CAN1, mbox_cylinder)!= CAN_TxStatus_Ok));
}

void Grab_Fan(void)
{
		memset(array, 0, 8);
	  array[0] = Cylinder_0_on;
 		Cylinder_Trans(array);
}
void Unlash_Fan(void)
{
		memset(array, 0, 8);
	  array[0] = Cylinder_0_off;
 		Cylinder_Trans(array);
}
void Push_Hand(void)
{
	memset(array,0,8);
	array[0]=Cylinder_2_on;
	array[1]=Cylinder_1_off;
	Cylinder_Trans(array);
}
void Pull_Hand(void)
{
	memset(array,0,8);
	array[0]=Cylinder_2_off;
	array[1]=Cylinder_1_on;
	Cylinder_Trans(array);
}
void Pull_Hand_Disable(void)
{
	memset(array,0,8);
	array[0]=Cylinder_2_off;
	array[1]=Cylinder_1_off;
	Cylinder_Trans(array);
}
void Grab_HighPost(void)
{
	memset(array, 0, 8);
 	array[0] = Cylinder_5_on;
	array[1] = Cylinder_6_off;
 	Cylinder_Trans(array);
}
void Unlash_HighPost(void)
{
	memset(array, 0, 8);
 	array[0] = Cylinder_5_off;
	array[1] = Cylinder_6_on;
 	Cylinder_Trans(array);
}

void HighPost_Disable(void)
{
	memset(array, 0, 8);
 	array[0] = Cylinder_5_off;
	array[1] = Cylinder_6_off;
 	Cylinder_Trans(array);
}

void Unlash_LowPost(void)
{
	memset(array, 0, 8);
 	array[0] = Cylinder_3_on;
	array[1] = Cylinder_4_off;
 	Cylinder_Trans(array);                     
	
}
void Grab_LowPost(void)
{
	memset(array, 0, 8);
 	array[0] = Cylinder_3_off;
	array[1] = Cylinder_4_on;
 	Cylinder_Trans(array);
}
void LowPost_Disable(void)
{
	memset(array, 0, 8);
 	array[0] = Cylinder_3_off;
	array[1] = Cylinder_4_off;
 	Cylinder_Trans(array);
}
void PropUp(void)
{
	memset(array,0,8);
	array[0]=Cylinder_7_on;
	Cylinder_Trans(array);
}

void PropDown(void)
{
	memset(array,0,8);
	array[0]=Cylinder_7_off;
	Cylinder_Trans(array);
}
void test(void)
{
	memset(array, 0, 8);
 	array[0] = Cylinder_1_on;
	array[1] = Cylinder_2_on;
	array[2] = Cylinder_3_on;
	array[3] = Cylinder_4_on;
	array[4] = Cylinder_5_on;
	array[5] = Cylinder_6_on;
	array[6] = Cylinder_7_on;
	array[7] = Cylinder_0_on;
 	Cylinder_Trans(array);
}

