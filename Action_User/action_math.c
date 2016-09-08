/**
  ******************************************************************************
  * @file     
  * @author  tzy
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
#include "action_math.h"
#include "math.h"
#include "GET_SET.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
//static float Pos_x;
//static float Pos_y;
//static float Angle;
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
/**
  * @brief  
  * @param 
  * @param 
  * @retval 
  */
//三角函数转换
double Sin(double Angle)
{
	return sin(Angle/180*PI);
}
double Cos(double Angle)
{
	return cos(Angle/180*PI);
}
double Tan(double Angle)
{
	return tan(Angle/180*PI);
}

//安全反三角函数,返回角度值（不是弧度值）
double Acos(double Cos)
{
	if(Cos>1)Cos=1;
	if(Cos<-1)Cos=-1;
	return ((180/PI)*acos(Cos));
}

double Asin(double Sin)
{
	if(Sin>1)Sin=1;
	if(Sin<-1)Sin=-1;
	return ((180/PI)*asin(Sin));
}

//实际速度转脉冲速度 
int VelTransform(float vel)
{
	return ((vel*2000*Reduce_Rate)/(PI*2*R_Wheel));//2000线（2000脉转一圈），减速比，直径
}
float AngleTranform(float ROBS_Angle)
{
 return ((ROBS_Angle-2048)*0.087f/Reduce_Rate_ROBS);
}

//获得圆弧圆点
float get_origin_x(float Circle_Radius,float angle_init)
{
	return (Get_POS_Xtemp()-(Circle_Radius*Cos(angle_init)));
}
float get_origin_y(float Circle_Radius,float angle_init)
{
	return (Get_POS_Ytemp()+(Circle_Radius*Sin(angle_init)));
}

float get_cos(float Circle_Radius,float angle_init)
{
	return ((get_origin_y(Circle_Radius,angle_init)-Get_POS_Y())/Circle_Radius);
}
float get_sin(float Circle_Radius,float angle_init)
{
	return ((Get_POS_X()-get_origin_x(Circle_Radius,angle_init))/Circle_Radius);
}

/************************ (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
