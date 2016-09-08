/**
  ******************************************************************************
  * @file    Fuzzy PID files   
  * @author  ACTION-Tzaiyang
  * @version 1.0
  * @date    March-1st-2016 
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
#include "fuzzypid.h"
#include "walk.h"
#include "GET_SET.h"
#include "usart.h"
#include "action_math.h"
#include "adc.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
static int8_t  isfirst=1;
/* Extern   variables ---------------------------------------------------------*/
extern float  debug_Err[3];
extern float  debug_Actvelx;
extern float  debug_change[3];
extern float  debug_pidPos[3];
extern float  debug_pidAng[3];
extern float  debug_pidAcv[3];
extern int8_t debug_errCla[3];
extern int8_t debug_errcCla[3];
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/


/**
  * @brief  基于模糊PID控制算法的直线闭环
  * @param  Vel:车的速度（正数 Vel>0）
  * @param  Ward：前进方向
  * @param  Refvalue：参考值
  * @param  Curvalue：实际值
  * @param  pPosition：位置闭环p参数
  * @param  Flag：调节标识位

  * @retval none
  */
void FuzPidLine(float Vel,float Ward,float Refangle,float Refvalue,float Curvalue,int8_t Flag,FPID_TypeDef *fpid)
{
					 float      Actvel;
					 float      adjVel;
					 float      newWard;
					 float      changeWard;
					 float      changeVel;
					 float      rotate;

					 float      errValue;
					 float	    errAngle;
					 float      errActvel;
	 static  float      sumErrAngle;
	 static  float      sumErrValue;
	 static  float      sumErrActvel;
	 static  float      last_errAngle;
	 static  float      last_errValue;
	 static  float      last_errActvel;
   static  float      errcValue;
   static  float      errcAngle;
   static  float      errcActvel;
	         int8_t     errValueCla;
					 int8_t     errAngleCla;
	         int8_t     errAcvelCla;
					 int8_t     errcValueCla;
					 int8_t     errcAngleCla;
	         int8_t     errcAcvelCla;
	 static  PIDGather_TypeDef  PIDGather; 
	      
	 Actvel=sqrt( pow(Get_ActVel_X(),2)+pow(Get_ActVel_Y(),2) );
	
	 errValue=(Refvalue-Curvalue)*Flag;/*位置误差*/
	 errAngle=Refangle-Get_Angle();/*角度误差*/
	 errActvel=Vel-Actvel;/*速度差值*/
	 
   errcValue=(errValue-last_errValue);
   errcAngle=(errAngle-last_errAngle);
   errcActvel=(errActvel-last_errActvel);
	 
	 #ifdef DEBUG
	 debug_Err[0]=errValue;
	 debug_Err[1]=errAngle;
	 debug_Err[2]=errActvel;
	 #endif
   
   //差量限制
   if(fabs(errValue)>ERRVALUE_MAX)   (errValue>0)?(errValue=ERRVALUE_MAX):(errValue=ERRVALUE_MIN);
   if(fabs(errAngle)>ERRANGLE_MAX)   (errAngle>0)?(errAngle=ERRANGLE_MAX):(errAngle=ERRANGLE_MIN);
   if(fabs(errActvel)>ERRACTVEL_MAX) (errActvel>0)?(errActvel=ERRACTVEL_MAX):(errActvel=ERRACTVEL_MIN);
   //差量微分限制
   if(fabs(errcValue)>ERRCVALUE_MAX)   (errcValue>0)?(errcValue=ERRCVALUE_MAX):(errcValue=ERRCVALUE_MIN);
   if(fabs(errcAngle)>ERRCANGLE_MAX)   (errcAngle>0)?(errcAngle=ERRCANGLE_MAX):(errcAngle=ERRCANGLE_MIN);
   if(fabs(errcActvel)>ERRCACTVEL_MAX) (errcActvel>0)?(errcActvel=ERRCACTVEL_MAX):(errcActvel=ERRCACTVEL_MIN);  
	 
	 if(isfirst){
		 sumErrAngle=0;
		 sumErrValue=0;
		 sumErrActvel=0; 
		 rotate =0;
		 changeWard=0;
		 adjVel=0; 
	 }
	 else      {
		 sumErrAngle+=errAngle;
		 sumErrValue+=errValue;
		 sumErrActvel+=errActvel;
		 
		 //积分调节限制
     if(fabs(sumErrAngle)>ERRSUMANGLE_MAX)  (sumErrAngle>0)?(sumErrAngle=ERRSUMANGLE_MAX):(sumErrAngle=ERRSUMANGLE_MIN);
     if(fabs(sumErrValue)>ERRSUMVALUE_MAX)  (sumErrValue>0)?(sumErrValue=ERRSUMVALUE_MAX):(sumErrValue=ERRSUMVALUE_MIN);
     if(fabs(sumErrActvel)>ERRSUMACTVEL_MAX)(sumErrActvel>0)?(sumErrActvel=ERRSUMACTVEL_MAX):(sumErrActvel=ERRSUMACTVEL_MIN);
		 
		 (errValue>0)?(errValueCla=(int8_t)(errValue*3/ERRVALUE_MAX+0.5f)):(errValueCla=(int8_t)(errValue*3/ERRVALUE_MAX-0.5f));
		 (errAngle>0)?(errAngleCla=(int8_t)(errAngle*3/ERRANGLE_MAX+0.5f)):(errAngleCla=(int8_t)(errAngle*3/ERRANGLE_MAX-0.5f));
		 (errActvel>0)?(errAcvelCla=(int8_t)(errActvel*3/ERRACTVEL_MAX+0.5f)):(errAcvelCla=(int8_t)(errActvel*3/ERRACTVEL_MAX-0.5f));
		 
		 (errcValue>0)?(errcValueCla=(int8_t)(errcValue*3/ERRCVALUE_MAX+0.5f)):(errcValueCla=(int8_t)(errcValue*3/ERRCVALUE_MAX-0.5f));
		 (errcAngle>0)?(errcAngleCla=(int8_t)(errcAngle*3/ERRCANGLE_MAX+0.5f)):(errcAngleCla=(int8_t)(errcAngle*3/ERRCANGLE_MAX-0.5f));
		 (errcActvel>0)?(errcAcvelCla=(int8_t)(errcActvel*3/ERRCACTVEL_MAX+0.5f)):(errcAcvelCla=(int8_t)(errcActvel*3/ERRCACTVEL_MAX-0.5f));
		 
		 //PID设定
		 FuzPidTable(VALUE_PID_SET,errValueCla,errcValueCla,&PIDGather,fpid);
		 FuzPidTable(ANGLE_PID_SET,errAngleCla,errcAngleCla,&PIDGather,fpid);
		 FuzPidTable(ACVEL_PID_SET,errAcvelCla,errcAcvelCla,&PIDGather,fpid);
     //PID控制输出
		 rotate = -errAngle*(PIDGather.Angle).pParam-sumErrAngle*(PIDGather.Angle).iParam-errcAngle*(PIDGather.Angle).dParam;
		 changeWard=errValue*(PIDGather.Position).pParam+sumErrValue*(PIDGather.Position).iParam+errcValue*(PIDGather.Position).dParam; /*需要调节的角度*/
	   adjVel=errActvel*(PIDGather.Actvel).pParam+sumErrActvel*(PIDGather.Actvel).iParam+(errcActvel)*PIDGather.Actvel.dParam; 
	 }
	 
	 newWard=Ward+changeWard;/*更新后角度*/
	 changeVel=(Vel+adjVel)/Cos(changeWard);/*更新后速度*/
	 
	 BasicLine(changeVel,newWard,rotate);
	 last_errAngle=errAngle;
	 last_errValue=errValue;
	 last_errActvel=errActvel;
	 ClearFirst();
   
	 #ifdef DEBUG
	 debug_Actvelx=Get_ActVel_X();
	 debug_change[0]=changeVel;
	 debug_change[1]=changeWard;
	 debug_change[2]=rotate;
	 debug_pidPos[0]=(PIDGather.Position).pParam;
	 debug_pidPos[1]=(PIDGather.Position).iParam;
	 debug_pidPos[2]=(PIDGather.Position).dParam;
	 debug_pidAng[0]=(PIDGather.Angle).pParam;
	 debug_pidAng[1]=(PIDGather.Angle).iParam;
	 debug_pidAng[2]=(PIDGather.Angle).dParam;
	 debug_pidAcv[0]=(PIDGather.Actvel).pParam;
	 debug_pidAcv[1]=(PIDGather.Actvel).iParam;
	 debug_pidAcv[2]=(PIDGather.Actvel).dParam;
	 debug_errCla[0]=errValueCla;
	 debug_errCla[1]=errAngleCla;
	 debug_errCla[2]=errAcvelCla;
	 debug_errcCla[0]=errcValueCla;
	 debug_errcCla[1]=errcAngleCla;
	 debug_errcCla[2]=errcAcvelCla;
	 #endif
}


/**
  * @brief  基于模糊PID控制算法的直线闭环
  * @param  Vel:车的速度（正数 Vel>0）
  * @param  Ward：前进方向
  * @param  Refvalue：参考值
  * @param  Curvalue：实际值
  * @param  pPosition：位置闭环p参数
  * @param  Flag：调节标识位

  * @retval none
  */
void SlopLine(float Vel,float Ward,float Refangle,float Refvalue,float Curvalue,int8_t Flag,FPID_TypeDef *fpid)
{
					 float      Actvel;
					 float      adjVel;
					 float      newWard;
					 float      changeWard;
					 float      changeVel;
					 float      rotate;

					 float      errValue;
					 float	    errAngle;
					 float      errActvel;
	 static  float      sumErrAngle;
	 static  float      sumErrValue;
	 static  float      sumErrActvel;
	 static  float      last_errAngle;
	 static  float      last_errValue;
	 static  float      last_errActvel;
   static  float      errcValue;
   static  float      errcAngle;
   static  float      errcActvel;
	         int8_t     errValueCla;
					 int8_t     errAngleCla;
	         int8_t     errAcvelCla;
					 int8_t     errcValueCla;
					 int8_t     errcAngleCla;
	         int8_t     errcAcvelCla;
	 static  PIDGather_TypeDef  PIDGather; 
	      
	 Actvel=sqrt( pow(Get_ActVel_X(),2)+pow(Get_ActVel_Y(),2) );
	
	 errValue=(Refvalue-Curvalue)*Flag;/*位置误差*/
	 errAngle=Refangle-Get_Angle();/*角度误差*/
	 errActvel=Vel-Actvel;/*速度差值*/
	 
   errcValue=(errValue-last_errValue);
   errcAngle=(errAngle-last_errAngle);
   errcActvel=(errActvel-last_errActvel);
	 
	 #ifdef DEBUG
	 debug_Err[0]=errValue;
	 debug_Err[1]=errAngle;
	 debug_Err[2]=errActvel;
	 #endif
   
   //差量限制
   if(fabs(errValue) >ERRVALUE_MAX)   (errValue>0)?(errValue=ERRVALUE_MAX):(errValue=ERRVALUE_MIN);
   if(fabs(errAngle) >ERRANGLE_MAX)   (errAngle>0)?(errAngle=ERRANGLE_MAX):(errAngle=ERRANGLE_MIN);
   if(fabs(errActvel)>ERRACTVEL_MAX) (errActvel>0)?(errActvel=ERRACTVEL_MAX):(errActvel=ERRACTVEL_MIN);
   //差量微分限制
   if(fabs(errcValue) >ERRCVALUE_MAX)   (errcValue>0)?(errcValue=ERRCVALUE_MAX):(errcValue=ERRCVALUE_MIN);
   if(fabs(errcAngle) >ERRCANGLE_MAX)   (errcAngle>0)?(errcAngle=ERRCANGLE_MAX):(errcAngle=ERRCANGLE_MIN);
   if(fabs(errcActvel)>ERRCACTVEL_MAX) (errcActvel>0)?(errcActvel=ERRCACTVEL_MAX):(errcActvel=ERRCACTVEL_MIN);  
	 
	 if(isfirst){
		 sumErrAngle=0;
		 sumErrValue=0;
		 sumErrActvel=0; 
		 rotate =0;
		 changeWard=0;
		 adjVel=0; 
	 }
	 else      {
		 sumErrAngle+=errAngle;
		 sumErrValue+=errValue;
		 sumErrActvel+=errActvel;
		 
		 //积分调节限制
     if(fabs(sumErrAngle)>ERRSUMANGLE_MAX)  (sumErrAngle>0)?(sumErrAngle=ERRSUMANGLE_MAX):(sumErrAngle=ERRSUMANGLE_MIN);
     if(fabs(sumErrValue)>ERRSUMVALUE_MAX)  (sumErrValue>0)?(sumErrValue=ERRSUMVALUE_MAX):(sumErrValue=ERRSUMVALUE_MIN);
     if(fabs(sumErrActvel)>ERRSUMACTVEL_MAX)(sumErrActvel>0)?(sumErrActvel=ERRSUMACTVEL_MAX):(sumErrActvel=ERRSUMACTVEL_MIN);
		 
		 (errValue>0)?(errValueCla=(int8_t)(errValue*3/ERRVALUE_MAX+0.5f)):(errValueCla=(int8_t)(errValue*3/ERRVALUE_MAX-0.5f));
		 (errAngle>0)?(errAngleCla=(int8_t)(errAngle*3/ERRANGLE_MAX+0.5f)):(errAngleCla=(int8_t)(errAngle*3/ERRANGLE_MAX-0.5f));
		 (errActvel>0)?(errAcvelCla=(int8_t)(errActvel*3/ERRACTVEL_MAX+0.5f)):(errAcvelCla=(int8_t)(errActvel*3/ERRACTVEL_MAX-0.5f));
		 
		 (errcValue>0)?(errcValueCla=(int8_t)(errcValue*3/ERRCVALUE_MAX+0.5f)):(errcValueCla=(int8_t)(errcValue*3/ERRCVALUE_MAX-0.5f));
		 (errcAngle>0)?(errcAngleCla=(int8_t)(errcAngle*3/ERRCANGLE_MAX+0.5f)):(errcAngleCla=(int8_t)(errcAngle*3/ERRCANGLE_MAX-0.5f));
		 (errcActvel>0)?(errcAcvelCla=(int8_t)(errcActvel*3/ERRCACTVEL_MAX+0.5f)):(errcAcvelCla=(int8_t)(errcActvel*3/ERRCACTVEL_MAX-0.5f));
		 
		 //PID设定
		 FuzPidTable(VALUE_PID_SET,errValueCla,errcValueCla,&PIDGather,fpid);
		 FuzPidTable(ANGLE_PID_SET,errAngleCla,errcAngleCla,&PIDGather,fpid);
		 FuzPidTable(ACVEL_PID_SET,errAcvelCla,errcAcvelCla,&PIDGather,fpid);
     //PID控制输出
		 rotate = -errAngle*(PIDGather.Angle).pParam-sumErrAngle*(PIDGather.Angle).iParam-errcAngle*(PIDGather.Angle).dParam;
		 changeWard=errValue*(PIDGather.Position).pParam+sumErrValue*(PIDGather.Position).iParam+errcValue*(PIDGather.Position).dParam; /*需要调节的角度*/
	   adjVel=errActvel*(PIDGather.Actvel).pParam+sumErrActvel*(PIDGather.Actvel).iParam+(errcActvel)*PIDGather.Actvel.dParam; 
	 }
	 
	 if(GetLaserValue(BASIC)<130)newWard=Ward+changeWard-GetLaserValue(BASIC)*0.1;
	 else
	 newWard=Ward+changeWard;/*更新后角度*/
	 changeVel=(Vel+adjVel)/Cos(changeWard);/*更新后速度*/
	 
	 BasicLine(changeVel,newWard,rotate);
	 last_errAngle=errAngle;
	 last_errValue=errValue;
	 last_errActvel=errActvel;
	 ClearFirst();
   
	 #ifdef DEBUG
	 debug_Actvelx=Get_ActVel_X();
	 debug_change[0]=changeVel;
	 debug_change[1]=changeWard;
	 debug_change[2]=rotate;
	 debug_pidPos[0]=(PIDGather.Position).pParam;
	 debug_pidPos[1]=(PIDGather.Position).iParam;
	 debug_pidPos[2]=(PIDGather.Position).dParam;
	 debug_pidAng[0]=(PIDGather.Angle).pParam;
	 debug_pidAng[1]=(PIDGather.Angle).iParam;
	 debug_pidAng[2]=(PIDGather.Angle).dParam;
	 debug_pidAcv[0]=(PIDGather.Actvel).pParam;
	 debug_pidAcv[1]=(PIDGather.Actvel).iParam;
	 debug_pidAcv[2]=(PIDGather.Actvel).dParam;
	 debug_errCla[0]=errValueCla;
	 debug_errCla[1]=errAngleCla;
	 debug_errCla[2]=errAcvelCla;
	 debug_errcCla[0]=errcValueCla;
	 debug_errcCla[1]=errcAngleCla;
	 debug_errcCla[2]=errcAcvelCla;
	 #endif
}
/**
  * @brief  模糊PID控制规则表匹配
  * @param  PIDGather:PID参数
  * @param  Err:差量
  * @param  Errc:差量变化率

  * @retval none
  */

void FuzPidTable(int8_t Flag,int Err,int Errc,PIDGather_TypeDef *PIDGather,FPID_TypeDef *fpid)
{
  if(Err==NB)//First Column
	{
		if(Errc==NB){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( PB,NB,PS,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( PB,NB,PS,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( PB,NB,PS,PIDGather,fpid);
		}
		if(Errc==NM){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( PB,NB,NS,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( PB,NB,NS,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( PB,NB,NS,PIDGather,fpid);
		}
		if(Errc==NS){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( PM,NB,NB,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( PM,NB,NB,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( PM,NB,NB,PIDGather,fpid);
		}
		if(Errc==ZO){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( PM,NB,NB,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( PM,NB,NB,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( PM,NB,NB,PIDGather,fpid);
		}
		if(Errc==PS){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( PS,NM,NB,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( PS,NM,NB,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( PS,NM,NB,PIDGather,fpid);
		}
		if(Errc==PM){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( ZO,ZO,NM,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( ZO,ZO,NM,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( ZO,ZO,NM,PIDGather,fpid);
		}
		if(Errc==PB){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( ZO,ZO,PS,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( ZO,ZO,PS,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( ZO,ZO,PS,PIDGather,fpid);
		}
	}
	if(Err==NM)//Second Column
	{
		if(Errc==NB){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( PB,NB,PS,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( PB,NB,PS,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( PB,NB,PS,PIDGather,fpid);
		}
		if(Errc==NM){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( PB,NB,NS,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( PB,NB,NS,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( PB,NB,NS,PIDGather,fpid);
		}
		if(Errc==NS){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( PM,NB,NB,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( PM,NB,NB,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( PM,NB,NB,PIDGather,fpid);
		}
		if(Errc==ZO){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( PS,NB,NM,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( PS,NB,NM,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( PS,NB,NM,PIDGather,fpid);
		}
		if(Errc==PS){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( PS,NM,NM,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( PS,NM,NM,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( PS,NM,NM,PIDGather,fpid);
		}
		if(Errc==PM){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( ZO,ZO,NS,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( ZO,ZO,NS,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( ZO,ZO,NS,PIDGather,fpid);
		}
		if(Errc==PB){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( NS,ZO,ZO,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( NS,ZO,ZO,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( NS,ZO,ZO,PIDGather,fpid);
		}
	}
	if(Err==NS)//Third Column
	{
		if(Errc==NB){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( PM,NM,ZO,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( PM,NM,ZO,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( PM,NM,ZO,PIDGather,fpid);
		}
		if(Errc==NM){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( PM,NM,NS,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( PM,NM,NS,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( PM,NM,NS,PIDGather,fpid);
		}
		if(Errc==NS){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( PM,NM,NM,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( PM,NM,NM,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( PM,NM,NM,PIDGather,fpid);
		}
		if(Errc==ZO){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( PS,NM,NM,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( PS,NM,NM,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( PS,NM,NM,PIDGather,fpid);
		}
		if(Errc==PS){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( ZO,ZO,NS,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( ZO,ZO,NS,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( ZO,ZO,NS,PIDGather,fpid);
		}
		if(Errc==PM){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( NS,PS,NS,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( NS,PS,NS,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( NS,PS,NS,PIDGather,fpid);
		}
		if(Errc==PB){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( NS,PS,ZO,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( NS,PS,ZO,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( NS,PS,ZO,PIDGather,fpid);
		}
	}
	if(Err==ZO)//Forth Column
	{
		if(Errc==NB){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( PM,NM,ZO,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( PM,NM,ZO,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( PM,NM,ZO,PIDGather,fpid);
		}
		if(Errc==NM){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( PM,NM,NS,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( PM,NM,NS,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( PM,NM,NS,PIDGather,fpid);
		}
		if(Errc==NS){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( PS,NS,NS,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( PS,NS,NS,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( PS,NS,NS,PIDGather,fpid);
		}
		if(Errc==ZO){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( ZO,ZO,NS,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( ZO,ZO,NS,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( ZO,ZO,NS,PIDGather,fpid);
		}
		if(Errc==PS){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( NS,PS,NS,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( NS,PS,NS,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( NS,PS,NS,PIDGather,fpid);
		}
		if(Errc==PM){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( NM,PM,NS,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( NM,PM,NS,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( NM,PM,NS,PIDGather,fpid);
		}
		if(Errc==PB){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( NM,PM,ZO,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( NM,PM,ZO,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( NM,PM,ZO,PIDGather,fpid);
		}
	}
	if(Err==PS)//Fifth Column
	{
		if(Errc==NB){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( PS,NS,ZO,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( PS,NS,ZO,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( PS,NS,ZO,PIDGather,fpid);
		}
		if(Errc==NM){ 
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( PS,NS,ZO,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( PS,NS,ZO,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( PS,NS,ZO,PIDGather,fpid);
		}
		if(Errc==NS){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( ZO,ZO,ZO,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( ZO,ZO,ZO,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( ZO,ZO,ZO,PIDGather,fpid);
		}
		if(Errc==ZO){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( NS,PM,ZO,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( NS,PM,ZO,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( NS,PM,ZO,PIDGather,fpid);
		}
		if(Errc==PS){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( NS,PM,ZO,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( NS,PM,ZO,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( NS,PM,ZO,PIDGather,fpid);
		}
		if(Errc==PM){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( NM,PM,ZO,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( NM,PM,ZO,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( NM,PM,ZO,PIDGather,fpid);
		}
		if(Errc==PB){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( NM,PM,ZO,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( NM,PM,ZO,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( NM,PM,ZO,PIDGather,fpid);
		}
	}
	if(Err==PM)//Sixth Column
	{
		if(Errc==NB){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( PS,ZO,PB,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( PS,ZO,PB,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( PS,ZO,PB,PIDGather,fpid);
		}
		if(Errc==NM){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( ZO,ZO,NS,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( ZO,ZO,NS,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( ZO,ZO,NS,PIDGather,fpid);
		}
		if(Errc==NS){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( NS,PS,PS,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( NS,PS,PS,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( NS,PS,PS,PIDGather,fpid);
		}
		if(Errc==ZO){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( NM,PS,PS,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( NM,PS,PS,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( NM,PS,PS,PIDGather,fpid);
		}
		if(Errc==PS){ 
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( NM,PM,PS,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( NM,PM,PS,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( NM,PM,PS,PIDGather,fpid);
		}
		if(Errc==PM){ 
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( NM,PB,PS,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( NM,PB,PS,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( NM,PB,PS,PIDGather,fpid);
		}
		if(Errc==PB){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( NB,PB,PB,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( NB,PB,PB,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( NB,PB,PB,PIDGather,fpid);
		}
	}
	if(Err==PB)//Seventh Column
	{
		if(Errc==NB){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( ZO,ZO,PB,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( ZO,ZO,PB,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( ZO,ZO,PB,PIDGather,fpid);
		}
		if(Errc==NM){ 
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( ZO,ZO,PM,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( ZO,ZO,PM,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( ZO,ZO,PM,PIDGather,fpid);
		}
		if(Errc==NS){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( NM,PS,PM,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( NM,PS,PM,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( NM,PS,PM,PIDGather,fpid);
		}
		if(Errc==ZO){ 
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( NM,PM,PM,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( NM,PM,PM,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( NM,PM,PM,PIDGather,fpid);
		}
		if(Errc==PS){  
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( NM,PM,PS,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( NM,PM,PS,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( NM,PM,PS,PIDGather,fpid);
		}
		if(Errc==PM){
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( NB,PB,PS,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( NB,PB,PS,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( NB,PB,PS,PIDGather,fpid);
		}
		if(Errc==PB){ 
			if(Flag==VALUE_PID_SET)PIDPos_Set   ( NB,PB,PB,PIDGather,fpid);
			if(Flag==ANGLE_PID_SET)PIDAngle_Set ( NB,PB,PB,PIDGather,fpid);
			if(Flag==ACVEL_PID_SET)PIDActvel_Set( NB,PB,PB,PIDGather,fpid);
		}
	}
}

void PIDPos_Set(float pid_p,float pid_i,float pid_d,PIDGather_TypeDef *PIDGathe,FPID_TypeDef *fpid)
{
	PIDGathe->Position.pParam=fpid->pvalue_base+pid_p*fpid->pvalue_s;
	PIDGathe->Position.iParam=fpid->ivalue_base+pid_i*fpid->ivalue_s;
	PIDGathe->Position.dParam=fpid->dvalue_base+pid_d*fpid->dvalue_s;
}

void PIDAngle_Set(float pid_p,float pid_i,float pid_d,PIDGather_TypeDef *PIDGathe,FPID_TypeDef *fpid)
{
	PIDGathe->Angle.pParam=fpid->pangle_base+pid_p*fpid->pangle_s;
	PIDGathe->Angle.iParam=fpid->iangle_base+pid_i*fpid->iangle_s;
	PIDGathe->Angle.dParam=fpid->dangle_base+pid_d*fpid->dangle_s;
}

void PIDActvel_Set(float pid_p,float pid_i,float pid_d,PIDGather_TypeDef *PIDGathe,FPID_TypeDef *fpid)
{
	PIDGathe->Actvel.pParam=fpid->pacvel_base+pid_p*fpid->pacvel_s;
	PIDGathe->Actvel.iParam=fpid->iacvel_base+pid_i*fpid->iacvel_s;
	PIDGathe->Actvel.dParam=fpid->dacvel_base+pid_d*fpid->dacvel_s;
}

void SetFirst(void)
{
	isfirst=1;
}

void ClearFirst(void)
{
	isfirst=0;
}

void FPIDpos_Set(float a,float b,float c,float d,float e,float f,float *fpid_value)
{
	   *(fpid_value+0)=a;
	   *(fpid_value+1)=b;
	   *(fpid_value+2)=c;
	   *(fpid_value+3)=d;
	   *(fpid_value+4)=e;
	   *(fpid_value+5)=f;
}

void FPIDangle_Set(float a,float b,float c,float d,float e,float f,float *fpid_value)
{
	   *(fpid_value+ 6)=a;
	   *(fpid_value+ 7)=b;
	   *(fpid_value+ 8)=c;
	   *(fpid_value+ 9)=d;
	   *(fpid_value+10)=e;
	   *(fpid_value+11)=f;
}

void FPIDacvel_Set(float a,float b,float c,float d,float e,float f,float *fpid_value)
{
	   *(fpid_value+12)=a;
	   *(fpid_value+13)=b;
	   *(fpid_value+14)=c;
	   *(fpid_value+15)=d;
	   *(fpid_value+16)=e;
	   *(fpid_value+17)=f;
}

void FPIDVal_Set(float *fpid_value,FPID_TypeDef *fpid)
{
	 fpid->pvalue_base=*(fpid_value+ 0);
	 fpid->ivalue_base=*(fpid_value+ 1);
	 fpid->dvalue_base=*(fpid_value+ 2);
	
	 fpid->pvalue_s   =*(fpid_value+ 3);
	 fpid->ivalue_s   =*(fpid_value+ 4);
	 fpid->dvalue_s   =*(fpid_value+ 5);
	
	 fpid->pangle_base=*(fpid_value+ 6);
	 fpid->iangle_base=*(fpid_value+ 7);
	 fpid->dangle_base=*(fpid_value+ 8);
	
	 fpid->pangle_s   =*(fpid_value+ 9);
	 fpid->iangle_s   =*(fpid_value+10);
	 fpid->dangle_s   =*(fpid_value+11);
	
	 fpid->pacvel_base=*(fpid_value+12);
	 fpid->iacvel_base=*(fpid_value+13);
	 fpid->dacvel_base=*(fpid_value+14);
	
	 fpid->pacvel_s   =*(fpid_value+15);
	 fpid->iacvel_s   =*(fpid_value+16);
	 fpid->dacvel_s   =*(fpid_value+17);
}


