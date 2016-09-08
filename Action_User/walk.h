/**
  ******************************************************************************
  * @file    
  * @author  Tzy
  * @version 
  * @date   
  * @brief   This file contains the headers of walk.c
  ******************************************************************************
  * @attention
  *
  *
  * 
  * 
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef  __WALK_H__
#define  __WALK_H__

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

// #define  RED_FIELD
#define BLUE_FIELD	
 
//#define BLUE_TOOTH
 #define  ENABLE_MISS_PROTECT


 #define LASER_YVALUE      			  	110
 #define LOWSPEED_X             	  (4215)
 
	
/* Exported constants --------------------------------------------------------*/
 #define TAGVEL       							 2200//2600//2100
 
 #ifdef  BLUE_FIELD
 #define ADDVELL      							 1200 
 #define ADDVEL       							 (1200+500)
 #define HIGH_DEC 									 900
 
 #elif  defined  RED_FIELD
 #define ADDVELL      							 1200 
 #define ADDVEL       							 (1200+500)
 #define HIGH_DEC                    600
 #endif
 
 
 

 #define LED_ON_X               		 5215 
 #define START_ADJX_Y           		 6600
 #define LOWSPEED_DIS           		(975+POST_XVALUE-LOWSPEED_X)

//改动行车轨迹时需改动的值    975+5500-4265=2210
 #define LINE_OY                     6153.6f 
 
 #ifdef  BLUE_FIELD
 
 #define POS_LINE1_END_Y             2200
 #define LASER_LINE1_END_Y           2957
 #define POS_LINE2_END_Y             4000
 
 #elif   defined RED_FIELD
 
 #define POS_LINE1_END_Y             2050
 #define LASER_LINE1_END_Y           2821
 #define POS_LINE2_END_Y             3971
 
 #endif
 
 #ifdef  BLUE_FIELD
 #define START_CIRCLE_X              (1214-80)
 #elif   defined RED_FIELD
 #define START_CIRCLE_X              (1214-30)
 #endif
 
 #ifdef  BLUE_FIELD
 		#define HIGH_CIRCLE_START_Y         (4044+150)//4276  (4044+250)
 #elif   defined RED_FIELD
  	#define HIGH_CIRCLE_START_Y         (4044+80)
 #endif
 
 #define LAST_CIRCLE_START_Y         (9237)
 #define LAST_CIRCLE_END_X           (3800)
 
 #define RADIUS1       							  1400
 #define RADIUS2      							 -6000//-5000//
 #define RADIUS3    						  	 -3800
 #define REFLASER                     170
 #define ACCURATE_A2_X                0
 
 #ifdef  BLUE_FIELD      
 #define P_VEL_Y                      3
 #elif   defined RED_FIELD             
 #define P_VEL_Y                      4
 #endif 
 
 #define P_LASER                      1.2
 
 //坐标原点改动后需改动的值
 #define ACCURATEY       	  				7187
 #define HIGH_WALL_XVALUE 	 				735
 #define POST_XVALUE        				5500

 
 //激光位置改动后需改动的值
 #define DIS_LASER_X      	  			 139
 #define DIS_LASER_Y      	 		 		 102.7
 
 
 //以下宏定义一般不用改动
 #define WASHING_WHEEL_CASE  22
 #define SELF_CHECK_CASE     66
 #define RESTART_IN_ORIGIN   88
 #define RESTART_IN_A2       110

 #define ISROTATE            1   //要自转
 #define NOROTATE            0   //不自转
 
 #define PLEFT              -1   //与左边保持距离
 #define PRIGHT              1   //与右边保持距离
 
 #define CIRCLE_END          1   //画完圆
 
 #define XDIRECTION         90  //X方向
 #define YDIRECTION          0  //Y方向

 #define PID_TAN            ((Get_POS_Xtemp()-Get_POS_X())/(Get_POS_Y()-Get_POS_Ytemp()))


 typedef struct{
	 float pParam;
	 float iParam;
	 float dParam;
 }PID_TypeDef;
 
 typedef struct{
	 PID_TypeDef Position;
	 PID_TypeDef Angle;
	 PID_TypeDef Actvel;
 }PIDGather_TypeDef;
 
 
 typedef struct{
	 float pvalue_base;
	 float ivalue_base;
	 float dvalue_base;
	 
	 float pvalue_s;
	 float ivalue_s;
	 float dvalue_s;
	 
	 float pangle_base;
	 float iangle_base;
	 float dangle_base;
	 
	 float pangle_s;
	 float iangle_s;
	 float dangle_s;
	 
	 float pacvel_base;
	 float iacvel_base;
	 float dacvel_base;
	 
	 float pacvel_s;
	 float iacvel_s;
	 float dacvel_s;
	 
 }FPID_TypeDef;
 
/* Exported functions ------------------------------------------------------- */
void 			LockWheel(void);
void 			BasicLine(int Vel,float ward,float Rotate);
void		  EndCloseLoopLine(void);

int8_t	  BasicCircle(int Vel,float WardInit,float WardEnd,float Refangle,float Radius,int8_t IsRotate,FPID_TypeDef *fpid);
int8_t    HighCircle(int Vel,float WardInit,float WardEnd,float Refangle,float Radius,int8_t IsRotate,FPID_TypeDef *fpid);
 
void 			AdjPosX(int LaserValue,int8_t flag);
void 			AdjPosX_A2(int AccurateX);
void 			AdjPosY(float PosY);
int 			Trans_Laser_to_Xvalue(float LaserValue,float Angle);
int 			Trans_Dis_to_Laser(float distance);
int 			Trans_Laser_to_Dis(float Laser);

void 		  SetActualVel(void);

#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/

