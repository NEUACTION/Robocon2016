/**
  ******************************************************************************
  * @file      
  * @author  2016Robcon-NEU Action团队
  * @version 
  * @date    
  * @brief   
  ******************************************************************************
  * @attention
  *
  *
  *
  * 
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include <math.h>
#include "walk.h"
#include "GET_SET.h"
#include "stdint.h"
#include "action_math.h"
#include "math.h"
#include "elmo.h"
#include "timer.h"
#include "robs.h"
#include "adc.h"
#include "usart.h"
#include "gpio.h"
#include "fuzzypid.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
/* Extern   variables ---------------------------------------------------------*/
extern float debug_Err[3];
extern int   debug_tagVel[6];
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/


/*************************************************************************/
/*----------------------------Walking Part-------------------------------*/
/*************************************************************************/

void LockWheel(void)
{
	VelCrl(1,0);
	VelCrl(2,0);
	VelCrl(3,0);	
}
/**
  * @brief  开环直线加上旋转
  * @param  Vel:车的和速度（正数 Vel>0）
  * @param  ward:车的行进方向
                   -180到+180
  * @param  Rotate:车自身的旋转速度（正数时右旋 俯视图）
  * @param  selfAngle:车自身的姿态角度
  * @retval none
  * @author lxy
  */
void BasicLine(int Vel,float ward,float Rotate)
{ 
	static  int tarV[3];
	static  int last_tarV[3]={0};
	        int V_sum;	
					int RotateVal;
					int maxV=0;
	        int i=0;
	        float reduceP;
	
	debug_tagVel[0]=last_tarV[0];
	debug_tagVel[1]=last_tarV[1];
	debug_tagVel[2]=last_tarV[2];
	
	/*计算自转脉冲数*/
	RotateVal=VelTransform(Rotate);
  /*计算前进总车速的脉冲数*/
	V_sum=VelTransform(Vel);
	
/*三轮速度计算*/
	#ifdef    BLUE_FIELD
	tarV[2]= (V_sum*Cos(ward-Get_Angle()+ 60))+RotateVal;
	tarV[1]=-(V_sum*Cos(ward-Get_Angle()+  0))+RotateVal;
	tarV[0]=-(V_sum*Cos(ward-Get_Angle()+120))+RotateVal;
	#elif	    defined	RED_FIELD
	tarV[0]=-(V_sum*Cos(ward-Get_Angle()+ 60))-RotateVal;//换蓝场时将tarV[0]各自都加个负号，并把0号与2号对调
	tarV[1]= (V_sum*Cos(ward-Get_Angle()+  0))-RotateVal;
	tarV[2]= (V_sum*Cos(ward-Get_Angle()+120))-RotateVal;
  #endif

	for(i=0;i<3;i++)
	{
		if(fabs(tarV[i])>maxV)
			  maxV=fabs(tarV[i]);
	}
//	USART_OUT(USART2,"maxV=%d\r\n",maxV/10);
	if(maxV>420000)
	{
		BEEP_ON;
		for(i=0;i<3;i++)
		{
			reduceP=(float)tarV[i]/(float)maxV;
			tarV[i]=420000*reduceP;
		}
	}

	/*三轮速度给定*/
	VelCrl(1,tarV[0]);
	VelCrl(2,tarV[1]);
	VelCrl(3,tarV[2]);
	
	last_tarV[0]=tarV[0];
	last_tarV[1]=tarV[1];
	last_tarV[2]=tarV[2];
}


void EndCloseLoopLine(void)
{
	 float	            errAngle            =0;
	 int                rotate              =0;
	 float              refAngle            =0;
	 #ifdef    BLUE_FIELD
	 int 								pAngle							=-1000;
	 #elif defined RED_FIELD
	 int                pAngle              =1000;
	 #endif 
   

	 errAngle=refAngle-Get_Angle();
     if(fabs(errAngle)<1)errAngle=0;
	 rotate = errAngle*pAngle;//-sumErrAngle*iAngle;
	 VelCrl(1,rotate);
	 VelCrl(3,rotate);

}
/**
  * @brief  闭环画弧,画完返回值1，没画完返回值0
  * @param  Vel:车的线速度（正数 Vel>0）
  * @param  WardInit：开始的方向
  * @param  WardEnd：画完的方向
  * @param  Radius：弧的半径（正数为顺时针，负数为逆时针）
  * @param  IsRotate：IsRotate==1,要自转，IsRotate==0，不自转

  * @retval none
  * @author tzy
  */
int8_t BasicCircle(int Vel,float WardInit,float WardEnd,float Refangle,float Radius,int8_t IsRotate,FPID_TypeDef *fpid)
{
	float ActRadius,Pos_Ox,Pos_Oy,angle,WardAdd;
	
	Pos_Ox=get_origin_x(Radius,WardInit);//坐标原点
	Pos_Oy=get_origin_y(Radius,WardInit);
	ActRadius=sqrt(pow((Get_POS_X()-Pos_Ox),2)+pow((Get_POS_Y()-Pos_Oy),2));//实际半径
	angle=((90-WardInit)-Acos(get_cos(Radius,WardInit)));//画圆的角度 

	WardAdd=WardInit+angle;	
	
	if(Radius>0)//顺时针
	{
	  if(IsRotate==1)FuzPidLine(Vel,WardAdd,WardAdd,Radius,ActRadius,1,fpid);
    if(IsRotate==0)FuzPidLine(Vel,WardAdd,Refangle,Radius,ActRadius,1,fpid);
	  if(WardAdd<=WardEnd)return CIRCLE_END;
	  if(WardAdd>WardEnd) return 0;		
	}
	else if(Radius<0)//逆时针
	{
		if(IsRotate==1) FuzPidLine(Vel,WardAdd,WardAdd,-Radius,ActRadius,-1,fpid);
    if(IsRotate==0)	FuzPidLine(Vel,WardAdd,Refangle,-Radius,ActRadius,-1,fpid); 
		if(WardAdd>=WardEnd)return CIRCLE_END;
		if(WardAdd<WardEnd) return 0;
	}
  return  2;	
}


/**
  * @brief  闭环画弧,画完返回值1，没画完返回值0
  * @param  Vel:车的线速度（正数 Vel>0）
  * @param  WardInit：开始的方向
  * @param  WardEnd：画完的方向
  * @param  Radius：弧的半径（正数为顺时针，负数为逆时针）
  * @param  IsRotate：IsRotate==1,要自转，IsRotate==0，不自转

  * @retval none
  * @author tzy
  */
int8_t HighCircle(int Vel,float WardInit,float WardEnd,float Refangle,float Radius,int8_t IsRotate,FPID_TypeDef *fpid)
{
	float ActRadius,Pos_Ox,Pos_Oy,angle,WardAdd;
	float p_highcircle=0.1;
	
	Pos_Ox=get_origin_x(Radius,WardInit);//坐标原点
	Pos_Oy=get_origin_y(Radius,WardInit);
	ActRadius=sqrt(pow((Get_POS_X()-Pos_Ox),2)+pow((Get_POS_Y()-Pos_Oy),2));//实际半径
	angle=((90-WardInit)-Acos(get_cos(Radius,WardInit)));//画圆的角度 
	
  if(GetLaserValue(BASIC)-REFLASER<70)
  WardAdd=WardInit+angle+(GetLaserValue(BASIC)-REFLASER)*p_highcircle;
	else
	WardAdd=WardInit+angle;	
	
	if(Radius>0)//顺时针
	{
	  if(IsRotate==1)FuzPidLine(Vel,WardAdd,WardAdd,Radius,ActRadius,1,fpid);
    if(IsRotate==0)FuzPidLine(Vel,WardAdd,Refangle,Radius,ActRadius,1,fpid);
	  if(WardAdd<=WardEnd)return CIRCLE_END;
	  if(WardAdd>WardEnd) return 0;		
	}
	if(Radius<0)//逆时针
	{
		if(IsRotate==1) FuzPidLine(Vel,WardAdd,WardAdd,-Radius,ActRadius,-1,fpid);
    if(IsRotate==0)	FuzPidLine(Vel,WardAdd,Refangle,-Radius,ActRadius,-1,fpid); 
		if(WardAdd>=WardEnd)return CIRCLE_END;
		if(WardAdd<WardEnd) return 0;
	}		
	return  2;	
}

/*************************************************************************/
/*----------------------------Adjustment Part-------------------------------*/
/*************************************************************************/

/**
  * @brief  把激光值转化为X坐标值
  * @param  LaserValue 激光值
  * @retval none
  * @author tzy
  */
int Trans_Laser_to_Xvalue(float LaserValue,float Angle)
{	
	#ifdef  BLUE_FIELD
	return (HIGH_WALL_XVALUE-(Trans_Laser_to_Dis(LaserValue)*Cos(Angle)-DIS_LASER_Y*Sin(Angle)));
	#elif   defined	RED_FIELD
	return (HIGH_WALL_XVALUE-(Trans_Laser_to_Dis(LaserValue)*Cos(Angle)+DIS_LASER_Y*Sin(Angle)));
	#endif
}
/**
  * @brief  高台处矫正坐标
  * @retval none
  * @author tzy
**/

void AdjPosX_A2(int AccurateX)
{
	Set_Offset_X(Get_POS_X()-AccurateX);
}

void AdjPosX(int LaserValue,int8_t flag)
{
   static int accurateX;
   static float offset_x;
   if(flag==0){
   accurateX=Trans_Laser_to_Xvalue(LaserValue,Get_Angle());
   offset_x=Get_POS_X()-accurateX;
	 }
   if(flag==1)Set_Offset_X(offset_x);
}
void AdjPosY(float PosY)
{
	 static float accuratey;
	 #ifdef  BLUE_FIELD
   accuratey=ACCURATEY+DIS_LASER_Y;
   #elif 	 defined  RED_FIELD
   accuratey=ACCURATEY-DIS_LASER_Y;
   #endif
	//if there is a accurate laser value,and then 
	//blue:accuratey= 7175+Trans_Laser_to_Dis(Laser)*sin(angle)+DIS_LASER_X*cos(angle)
	//red: accuratey=7175+Trans_Laser_to_Dis(Laser)*sin(angle)-DIS_LASER_X*cos(angle)
   Set_Offset_Y(PosY-accuratey);
}

int Trans_Dis_to_Laser(float distance)
{
	return (1.071f*(distance-DIS_LASER_X)-209.6f);
}

int Trans_Laser_to_Dis(float Laser)
{
	return  ((Laser+209.6f)/1.071f+DIS_LASER_X);
}







void  SetActualVel(void)
{
		int posX=0,posY=0;
		int vel_X=0,vel_Y=0;
		int i=0;
		static int prePosX[10],prePosY[10];	
	/*计算实际速度*/
		 posX=Get_POS_X();
		 posY=Get_POS_Y();
		
		 vel_X=(posX-prePosX[0])*10;
		 vel_Y=(posY-prePosY[0])*10;
		 		 
		 Set_ActVel_X(vel_X);
		 Set_ActVel_Y(vel_Y);	
     /*更新存储的坐标*/
		 for(i=0;i<9;i++)
		 {
			  prePosX[i]=prePosX[i+1];
			  prePosY[i]=prePosY[i+1];
		 }
		 prePosX[9]=posX;
		 prePosY[9]=posY;
}

/************************ (C) COPYRIGHT 2015 ACTION *****END OF FILE****/

