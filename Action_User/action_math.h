
/**
  ******************************************************************************
  * @file    
  * @author  Tzy
  * @version 
  * @date   
  * @brief   This file contains the headers of action_math.c
  ******************************************************************************
  * @attention
  *
  *
  * 
  * 
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ACTION_MATH_H__
#define __ACTION_MATH_H__

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
double Sin(double Angle);
double Cos(double Angle);
double Tan(double Angle);
double Asin(double Sin);
double Acos(double Cos);
int VelTransform(float vel);
float get_origin_x(float Circle_Radius,float angle_init);
float get_origin_y(float Circle_Radius,float angle_init);
float get_cos(float Circle_Radius,float angle_init);
float get_sin(float Circle_Radius,float angle_init);

#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/

