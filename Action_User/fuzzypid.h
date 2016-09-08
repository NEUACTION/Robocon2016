
/**
  ******************************************************************************
  * @file    
  * @author  Tzaiyang
  * @version 
  * @date   
  * @brief   This file contains the headers of fuzzypid.c
  ******************************************************************************
  * @attention
  *
  *
  * 
  * 
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FUZZYPID_H
#define FUZZYPID_H


/* Includes ------------------------------------------------------------------*/  
#include "walk.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/


//差量限制
#define ERRVALUE_MAX    250
#define ERRVALUE_MIN   -250
#define ERRANGLE_MAX    15
#define ERRANGLE_MIN   -15
#define ERRACTVEL_MAX   600
#define ERRACTVEL_MIN  -600
//差量微分限制
#define ERRCVALUE_MAX   20
#define ERRCVALUE_MIN  -20
#define ERRCANGLE_MAX   0.8f
#define ERRCANGLE_MIN  -0.8f
#define ERRCACTVEL_MAX  60
#define ERRCACTVEL_MIN -60
//积分限制
#define ERRSUMVALUE_MAX  500
#define ERRSUMVALUE_MIN -500
#define ERRSUMANGLE_MAX  30
#define ERRSUMANGLE_MIN -30
#define ERRSUMACTVEL_MAX  1000
#define ERRSUMACTVEL_MIN -1000

//模糊语言定义
#define NB     -3
#define NM     -2
#define NS     -1
#define ZO      0
#define PS      1
#define PM      2
#define PB      3


#define VALUE_PID_SET   1
#define ANGLE_PID_SET   2
#define ACVEL_PID_SET   3


void FuzPidTable(int8_t Flag,int Err,int Errc,PIDGather_TypeDef *PIDGather,FPID_TypeDef *fpid);
void FuzPidLine(float Vel,float Ward,float Refangle,float Refvalue,float Curvalue,int8_t Flag,FPID_TypeDef *fpid);
void SlopLine(float Vel,float Ward,float Refangle,float Refvalue,float Curvalue,int8_t Flag,FPID_TypeDef *fpid);

void FPIDVal_Set(float *fpid_value,FPID_TypeDef *fpid);
											
void FPIDpos_Set(float a,float b,float c,float d,float e,float f,float *fpid_value);
void FPIDangle_Set(float a,float b,float c,float d,float e,float f,float *fpid_value);
void FPIDacvel_Set(float a,float b,float c,float d,float e,float f,float *fpid_value);
											
void PIDPos_Set(float pid_p,float pid_i,float pid_d,PIDGather_TypeDef *PIDGathe,FPID_TypeDef *fpid);
void PIDAngle_Set(float pid_p,float pid_i,float pid_d,PIDGather_TypeDef *PIDGathe,FPID_TypeDef *fpid);
void PIDActvel_Set(float pid_p,float pid_i,float pid_d,PIDGather_TypeDef *PIDGathe,FPID_TypeDef *fpid);
void ClearFirst(void);
void SetFirst(void);

#endif
    
