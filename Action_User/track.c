/*************************************************************************/
/*----------------------------Tracking Part-------------------------------*/
/*************************************************************************/

/**
******************************************************************************
* @file
* @author  
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
#include "walk.h"
#include "elmo.h"
#include "track.h"
#include "stdint.h"
#include "adc.h"
#include "usart.h"
#include "robs.h"
#include "timer.h"
#include "math.h"
#include "GET_SET.h"
#include "action_math.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
#ifdef  	BLUE_FIELD
static float 			angle_cmd				=-17.0;
#elif			defined RED_FIELD
static float 			angle_cmd				= 17.0;
#endif

static int 				tarHeight				=0;
static uint8_t 		missCount				=0;
static float 			dutyFactor			=0.05;
static uint8_t 		speedStatus			=0;	
/* Extern   variables ---------------------------------------------------------*/
extern float  debug_windspeed;
extern int8_t status;
extern int    debug_ecoPosY;
extern int    debug_ecoPosX;
extern int    debug_accurateLaser;
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


/**
  * @brief  控制升降台的高度
  * @retval none
  * @author lxy
  */
static int accuPreLaser;  //准确的上一个激光值
void UpdataHeight(float PosX,float PosY)
{
//	int8_t errPix=0;
	int maxHeight=-495,minHeight=0;
	
//  curHeight=Get_Pos(4)/450;
//	errPix=60-Get_Camera_Height();
	
//	if(fabs(errPix)>3&&errPix<0)
//	{
//		 tarHeight=Trans_Laser_to_Dis(accuPreLaser)*Tan(errPix*0.165)+curHeight ;	
//	}
	
	if(PosY>600.0f&&PosY<2000.0f)minHeight=-100;

	if(PosY>2000.0f&&PosY<4300.0f)minHeight=-350;

	if((PosY>4300.0f&&PosY<8000.0f)||(Get_FanFlag()==2&&PosY>4000))minHeight=-495;
	
	if(tarHeight<maxHeight)
		tarHeight=maxHeight;
	else if(tarHeight>minHeight)
		tarHeight=minHeight;

  if(PosY>8000)tarHeight=-160;
	
	PosCrl_mm(tarHeight);
}


/**
  * @brief  控制舵机角度
  * @retval none
  * @author ljc
**/

extern int8_t status;
static float  distance=0;	//小车大车之间的距离
static int curAngle;      //舵机当前的角度
void UpdataAngle(float PosX,float PosY)     //更新舵机角度
{
	 int      pos							=0;
	 float    err_pos					=0;
	 #ifdef   BLUE_FIELD
   float    angleMax				= 110;
	 float 		angleMin				=-30;
	 #elif    defined RED_FIELD
	 float 		angleMax				= 30;
	 float 		angleMin				=-110; 
	 #endif


	 curAngle=Get_ROBS_Angle();
	 pos = Get_Camera_Angle();	
	 	
	 
	 /*计算舵机转动角度*/
   if(pos) 
	 {
		  err_pos=pos-160;
		  if(fabs(err_pos)<3)err_pos=0;
			if(PosY>100&&PosY<2500){
				#ifdef BLUE_FIELD
				angle_cmd=err_pos*0.1875f+curAngle+15;
				#elif	 defined RED_FIELD
				angle_cmd=err_pos*0.1875f+curAngle-15;
				#endif
			}
			else if(PosY>2500&&PosY<5000){
				#ifdef BLUE_FIELD
				angle_cmd=err_pos*0.1875f+curAngle-6;
				#elif	 defined RED_FIELD
				angle_cmd=err_pos*0.1875f+curAngle+6;
				#endif
			}
			else if(PosY>5000&&PosY<7500){
				#ifdef BLUE_FIELD
				angle_cmd=err_pos*0.1875f+curAngle-8;
				#elif	 defined RED_FIELD
				angle_cmd=err_pos*0.1875f+curAngle+8;
				#endif
			}		
			else 		
				angle_cmd=err_pos*0.1875f+curAngle;
	 }
	 
#ifdef ENABLE_MISS_PROTECT	 
	 if(!pos&&PosY<7000&&PosY>1000&&(status>0&&status<8) )   missCount++;
	 else       missCount=0;	 
	 if(missCount>=30)status=21;
#endif
	 if(PosY>=6301&&PosY<7056) //高台
	 {	
				angleMin=-30;
				angleMax=30;			
	 }

	 if(PosY>8900)
	 {
		 angleMax=0;
		 angleMin=0;
	 }	
	 if(angle_cmd>angleMax)
			angle_cmd=angleMax;
	 else if(angle_cmd<angleMin)
			 angle_cmd=angleMin;
	 ROBS_PosCrl(angle_cmd,3000);
	 //ROBS_PosCrl(0,3000);
}

 /**
  * @brief  计算小车坐标
*/

void UpdataEcoCoor(float PosX,float PosY)
{

	 int  trackLaser=0;
	 static int preLaser;
	 static uint8_t getFlag=1;
   static int  getCount=0;

   static int eco_offsetX;	  //上电矫正小车坐标
	 static int eco_offsetY;
	 static int adjustCount=0;
	
	 static int ecoPosX,ecoPosY;
	 static float xita,alpha;

	 trackLaser  = GetLaserValue(TRACK);
	 if((trackLaser<LASER_RANGE&&trackLaser>250))
	 {       			
					/*激光和最近前一次准确距离差值小于200*/
					if(fabs(trackLaser-accuPreLaser)<200||accuPreLaser==0)  
					{
							distance = Trans_Laser_to_Dis(trackLaser);  
							accuPreLaser=trackLaser;
						  getFlag=1;
					}
					else
					{
						  getFlag=0;
						  if(getCount==3)
							{
								getFlag=1;
								getCount=0;
								accuPreLaser=trackLaser;								
							}
							
					}
						
		}
		
		if(fabs(trackLaser-preLaser)<200&&getFlag==0)
		{
			 getCount++;
		}
		else     //如果getFlag=1或者上一次激光和这一次激光差值小于200
		{
			getCount=0;
		}
		preLaser=trackLaser;
		/*计算小车坐标*/
		xita=30+curAngle;
		alpha=11.72-30-Get_Angle();
		ecoPosX=distance*Cos(xita)+ L * Cos(alpha) + PosX-eco_offsetX;
		ecoPosY=distance*Sin(xita)+ L * Sin(alpha) + PosY-eco_offsetY;
		if(adjustCount++==100)
		{
			 eco_offsetX=ecoPosX;
			 eco_offsetY=ecoPosY;
		}
    debug_ecoPosY  =  ecoPosY;
		debug_ecoPosX  =  ecoPosX;
	  debug_accurateLaser=accuPreLaser;
	
}
  /* @brief  控制舵机风速
  * @retval none
  * @author ljc
**/
void UpdataWindSpeed(float PosX, float PosY)
{
//	float adjDutyFactor=0.0f;
  static uint8_t count_light=0;
	static uint8_t count_blue=0;
	
//	adjDutyFactor=adjDutyFactor;//prevent warning
	
	switch(speedStatus)
	{
	 /*大风阶段一*/
	 case 0:
			if (PosX>100)
			{
				#ifdef BLUE_FIELD
			  dutyFactor=0.088f;
        #elif defined RED_FIELD
			  dutyFactor=0.087f;
        #endif	 
				speedStatus++;
			}
		 break;
	 /*大风阶段二*/
	 case 1:
		 if(PosY>1000)
		 {
			 speedStatus++;
		 }
		 break;
	 /*小风阶段*/
	 case 2:
//				adjDutyFactor=-(float)Get_AdjVel()/100000.0f;//(1500-accuPreLaser)*0.00001;
				#ifdef BLUE_FIELD
//			  dutyFactor=0.086f+adjDutyFactor;
//			  if(dutyFactor>=0.088f)
//					dutyFactor=0.088f;
//				else if(dutyFactor<=0.084f)
//					dutyFactor=0.084f;
	        dutyFactor=0.088f;

        #elif defined RED_FIELD
					dutyFactor=0.088f;
        #endif
			if(Get_BlueTooth_Flag())
			{
						if(Get_FanFlag()==2)count_blue++; 
						else                count_blue=0;
			}
			else
			{
						if(Get_FanFlag_light()==1)count_light++;
						else                count_light=0;	
			}
					 
			if((count_light>=6||count_blue>=5)&&PosY>2500)		
			{	
        #ifdef BLUE_FIELD
			  dutyFactor=0.072f;//0.078f;
        #elif defined RED_FIELD
			  dutyFactor=0.074f;
        #endif					
				speedStatus++;
				count_light=0;
				count_blue=0;
			}
		break;			
		/*河道大风阶段*/
		case 3:
			if(Get_BlueTooth_Flag())
			{
						if(Get_FanFlag()==1)count_blue++; 
						else                count_blue=0;
			}
			else
			{
						if(Get_FanFlag_light()==2)count_light++;
						else                count_light=0;	
			}
			
			if(count_light>=6||count_blue>=5)
			{
				#ifdef BLUE_FIELD
				#ifdef BLUE_TOOTH
				if(Get_EcoPosition()<1500)
				#else
				if(accuPreLaser<2800)
				#endif
				{
					  #ifdef BLUE_TOOTH
					  dutyFactor=0.078f+(float)Get_EcoPosition()*0.000011f; 
					  #else
						dutyFactor=0.079f+(float)accuPreLaser*0.000001f;
            #endif					
						if(dutyFactor>0.085f)                                                
								dutyFactor=0.085f;
				}
				else
					dutyFactor=0.072;

				#elif defined RED_FIELD
				#ifdef BLUE_TOOTH
				if(Get_EcoPosition()<1500)
				#else
				if(accuPreLaser<2800)
				#endif
				{
					  #ifdef BLUE_TOOTH
					  dutyFactor=0.078f+(float)Get_EcoPosition()*0.000011f; 
					  #else
						dutyFactor=0.079f+(float)accuPreLaser*0.000001f;
            #endif					  
						if(dutyFactor>0.083f)                                                
								dutyFactor=0.083f;
				}
				else
					dutyFactor=0.072f;
				#endif	
				                                               //0.015  0.000005		
				count_light=0;
				count_blue=0;
				speedStatus++;
			}
		break;
		/*停风阶段*/
		case 4:
		#ifdef 	BLUE_TOOTH	//用蓝牙作信号
		if(Get_BlueTooth_Flag())
			{
						if(Get_FanFlag()==0)count_blue++; 
						else                count_blue=0;
			}

			if(count_blue>=5){
				dutyFactor=0.05f;
				count_light=0;
				count_blue=0;
				speedStatus++;
			}
			else
			{
				if(Get_EcoPosition()<1500)
				{
					dutyFactor=0.078f+(float)Get_EcoPosition()*0.000011f;     
				if(dutyFactor>0.083f)                                               
				    dutyFactor=0.083f;
				}
				else
				{
				    dutyFactor=0.072;
				}
			}		
		#else  //用灯带作信号
			if(Get_FanFlag_light()==1)count_light++;
			else                			count_light=0;	

			if(count_light>=8){
				dutyFactor=0.05f;
				count_light=0;
				count_blue=0;
				speedStatus++;
			}
			else
			{
				if(accuPreLaser<2800)
				{
						dutyFactor=0.079f+(float)accuPreLaser*0.000001f;     
						if(dutyFactor>0.083f)                                                
								dutyFactor=0.083f;
				}
				else
					dutyFactor=0.072;
			}
		 #endif
		break;
		case 5:
			break;
	}
	debug_windspeed=dutyFactor;
	if(PosY>8800) dutyFactor=0.05f;
	TIM_SetCompare2(TIM3,dutyFactor*2000);
}

void UpdateAdjVel(int status)
{
	 int 								adjVel				=0;
	 int 								disY					=0;
	 static     int     preEcoPos     =0;
	 static     int     equalCount    =0;
	 static     int     adjEnableFlag =1;
	 /*保持大小车距离，调节车速*/
	 if(status==1)
	 {
		 adjVel=(accuPreLaser-1500)*0.5;
		 if(adjVel>400)
			 adjVel=400;
		 Set_AdjVel(adjVel);
	 }
	 if(status>1&&status<6)
	 {
			 if(preEcoPos==Get_EcoPosition())
			 {
					equalCount++;
					if(equalCount==5)
						Set_AdjVel(0);
			 }
			 else
			 {
				 equalCount=0;
				 #ifdef BLUE_TOOTH
    		 disY=0.8612*Get_EcoPosition()-1050;
				 adjVel=(disY-Get_POS_Y())*1;
				 if(adjVel<-2000)adjVel=-2000;
				 if(adjVel>600)  adjVel= 600;
				 #elseif
				 adjVel=(accuPreLaser-1700);
				 #endif
				 Set_AdjVel(adjVel);
				 preEcoPos=Get_EcoPosition();
			 }
	 }
	 #ifdef BLUE_TOOTH
	 else if((status==6||status==7)&&Get_EcoPosition()<3200)
	 {
		     if(preEcoPos==Get_EcoPosition())
				 {
					  equalCount++;
					  if(equalCount>=10)
							//Set_AdjVel(0);
						  status=21;
				 }
				 else
				 {
					equalCount=0;
					#ifdef BLUE_FIELD
					disY=1.019*Get_EcoPosition()+5322; 
					#elif defined RED_FIELD
//					disY=1.019*Get_EcoPosition()+5022;
					disY=(2.199e+04f)*sin(0.0005311f*Get_EcoPosition()+0.1405f)+
					     (1.429e+04f)*sin(0.0006822f*Get_EcoPosition()+2.978f);
					#endif	
					
					 adjVel=(disY-(int)Get_POS_Y())*1.5;
					 if(adjVel>600)
						 adjVel=600;
					 else if(adjVel<-2000)
						 adjVel=-2000;
					 Set_AdjVel(adjVel);
					 preEcoPos=Get_EcoPosition();
				 }
	 }
	 #else
	 /*防止犯规调节速度*/
	 else if(((status==6&&speedStatus==4)||status==7)&&adjEnableFlag==1)
	 {

				 disY=1.014*accuPreLaser+4127; 	
					
				 adjVel=(disY-(int)Get_POS_Y())*2.5;
				 if(adjVel>600)
					 adjVel=600;
				 else if(adjVel<-2000)
					 adjVel=-2000;
				 Set_AdjVel(adjVel);
				 if(Get_POS_Y()>8600||speedStatus==5)//如果Y坐标大于8600或者小车给出信号，认为小车出河道了
				 {
					   adjEnableFlag=0;
					   Set_AdjVel(0);
				 }	
	 }
   #endif
		 			
}


void PosCrl_mm(int Dis)
{
	 float pulse;     
	 //pulse=Dis*372.8;  RE35
	 pulse=Dis*450;
	 PosCrl(4,POS_ABS,(int)pulse); 
}

void InitStaticVar(void)
{
				tarHeight				=0;
				#ifdef  BLUE_FIELD
				angle_cmd				=-17.0;
        #elif   defined RED_FIELD
	      angle_cmd				= 17.0;
	      #endif
				missCount				=0;
				dutyFactor			=0.05;
				speedStatus			=0;	
}
/************************ (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
