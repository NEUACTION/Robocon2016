#ifndef __TRACK_H__
#define __TRACK_H__

/**
******************************************************************************
* @file
* @author  
* @version
* @date
* @brief   This file contains the headers of
******************************************************************************
* @attention
*
*
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
 
 #define LASER_RANGE         3500.0 
 #define P_FAN               0.00001
 #define L                   471.28
/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void UpdataHeight(float PosX,float PosY);
void UpdataAngle(float PosX,float PosY);
void UpdataWindSpeed(float PosX,float PosY);
void PosCrl_mm(int Dis);
void UpdataEcoCoor(float PosX,float PosY);

void UpdateAdjVel(int status);
void InitStaticVar(void);

#endif
/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/

