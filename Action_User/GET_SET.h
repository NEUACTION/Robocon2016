/**
  ******************************************************************************
  * @file    
  * @author  Lxy Action
  * @version 
  * @date   
  * @brief   This file contains the headers of 
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
#ifndef __GET_SET_H
#define __GET_SET_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define PI 3.1415926
#define Reduce_Rate 299/14
#define Reduce_Rate_ROBS 52/30
#define R_Wheel 70
#define HOO     1960

#define OUT_OF_SIGHT   0

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void Set_EcoPosition(int16_t val);
int16_t Get_EcoPosition(void);
void    Reset_EcoPosition(void);

void 		 Set_FanFlag(int8_t val);
uint8_t  Get_FanFlag(void);
void Set_FanFlag_light(int8_t val);
uint8_t Get_FanFlag_light(void);


void 		 Set_Original_POS_X(float val);
void 		 Set_Original_POS_Y(float val);
void 		 Set_Original_Angle(float val);
float 	 Get_Original_POS_X(void);
float		 Get_Original_POS_Y(void);
float		 Get_Original_Angle(void);


void		 Set_POS_X(float val);
float		 Get_POS_X(void);
void		 Set_POS_Y(float val);
float		 Get_POS_Y(void);
void		 Set_Angle(float val);
float		 Get_Angle(void);

void 		 Set_AdjVel(int vel);
int 		 Get_AdjVel(void);

void		 Set_LaserValue_TEMP(float val);
void     xyTempUpdate(void);


void		 Set_Current(float val,int num);
void		 Set_Pos(int val,int num);
void		 Set_Vel(int vel,int num);
void     Set_BlueTooth_Flag(int8_t flag);
int8_t   Get_BlueTooth_Flag(void);

void		 Set_Offset_X(float val);
void		 Set_Offset_Y(float val);
void     Reset_Offset_X(void);
void 		 Reset_Offset_Y(void);
void		 Set_POS_Xtemp(float val);
void		 Set_POS_Ytemp(float val);
float		 Get_POS_Xtemp(void);
float		 Get_POS_Ytemp(void);

float		 Get_Current(int num);
int	   	 Get_Pos(int num);
int   	 Get_Vel(int num);
float		 Get_LaserValue_TEMP(void);

void		 Set_ROBS_Angle(int val);
int		   Get_ROBS_Angle(void);

void		 Set_ActVel_X(int val);
void		 Set_ActVel_Y(int val);
int			 Get_ActVel_X(void);
int			 Get_ActVel_Y(void);

void		 Calculate(void);

void		 Set_Camera_Angle(int16_t val);
int16_t  Get_Camera_Angle(void);


void		 updaterobs_data(uint8_t i,uint8_t data,uint8_t flag);
#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/

