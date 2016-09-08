/**
  ******************************************************************************
  * @file     
  * @author  lxy
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
#include "GET_SET.h"
#include "elmo.h"
#include "action_math.h"
#include "walk.h"

/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/

static float Original_Pos_x;
static float Original_Pos_y;
static float Original_Angle;


static float Pos_x;
static float Pos_y;
static float Angle;
static float offset_x=0;
static float offset_y=0;
static float Pos_xtemp;                      //temporary value of coordinate in X direction 
static float Pos_ytemp;                      //temporary value of coordinate in Y direction
static float LaserValue_TEMP;
static union can_message receive_current[6];
static int   Pos[7];
static int   Vel[7];

static int8_t btFlag=1; 

//static int		 Camera_Height;
static float 	 adjVel=0;
static uint8_t FanFlag=1;
static uint8_t FanFlag_light=1;
static int     camera_angle;
static int     actVelX,actVelY;
static uint16_t Ecoposition;

static 	uint8_t robs_data[4];
static	uint8_t robs_datagetting=0;
static  int		  ROBS_Angle;
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
	
		void Set_EcoPosition(int16_t val)
	{
		Ecoposition=val;
	}
	
	int16_t Get_EcoPosition(void)
	{
		return Ecoposition;
	}
	void Set_FanFlag(int8_t val)
	{
		FanFlag=val;
	}
	uint8_t Get_FanFlag(void)
	{
		return  FanFlag;
	}
	void Set_FanFlag_light(int8_t val)
	{
		FanFlag_light=val;
	}
	uint8_t Get_FanFlag_light(void)
	{
		return  FanFlag_light;
	}
	
	void Set_POS_X(float val)
	{
		Pos_x=val;
	}
	float Get_POS_X(void)
	{
	  return (Pos_x-offset_x);
	}
	void Set_POS_Y(float val)
	{
		Pos_y=val;
	}
	float Get_POS_Y(void)
	{
		return (Pos_y-offset_y);
	}
	 void Set_Angle(float val)
	{
		Angle=val;
	}
	float Get_Angle(void)
	{
		return Angle;
	}
	
	void Set_Original_POS_X(float val)
	{
		Original_Pos_x=val;
	}
	float	Get_Original_POS_X(void)
	{
		  return   Original_Pos_x;
	}
	
	void Set_Original_POS_Y(float val)
	{
		Original_Pos_y=val;
	}
	float Get_Original_POS_Y(void)
	{
		  return   Original_Pos_y; 
	}
	
		void Set_Original_Angle(float val)
	{
		Original_Angle=val;
	}

  float Get_Original_Angle(void)
	{
		  return   Original_Angle; 
	}
	
	void Set_POS_Xtemp(float val)
	{
		Pos_xtemp=val;
	}
	float Get_POS_Xtemp(void)
	{
		return Pos_xtemp;
	}
		void Set_POS_Ytemp(float val)
	{
		Pos_ytemp=val;
	}
	float Get_POS_Ytemp(void)
	{
		return Pos_ytemp;
	}
	
	
	void Set_Current(float val,int num)
	{
		receive_current[num].dataf=val;	
	}
  float Get_Current(int num)
	{
		return (receive_current[num].dataf);
	}
	
		
	void Set_Vel(int vel,int num)
	{
		Vel[num]=vel;
	}
	int Get_Vel(int num)
	{
		return Vel[num];
	}
	
	
	
	void Set_BlueTooth_Flag(int8_t flag)
	{
		btFlag=flag;
	}
	int8_t Get_BlueTooth_Flag(void)
	{
		return btFlag;
	}
	

	
	void Set_Pos(int val,int num)
	{
		Pos[num]=val;
	}
	int Get_Pos(int num)
	{
		return Pos[num];
	}
	
  	void Set_LaserValue_TEMP(float val)
	{
		  LaserValue_TEMP=val; 
	}

	float Get_LaserValue_TEMP()
	{
		return LaserValue_TEMP;
	}
	void Set_ActVel_X(int val)
{
	  actVelX=val;
}
int Get_ActVel_X(void)
{
	  return   actVelX;
}


void Set_ActVel_Y(int val)
{
	  actVelY=val;
}
int Get_ActVel_Y(void)
{
	  return   actVelY;
}

void Set_AdjVel(int vel)
{
		adjVel=vel;
}

int Get_AdjVel(void)
{
		return adjVel;
}

void Set_Camera_Angle(int16_t val)
{
	camera_angle=val;
}

int16_t Get_Camera_Angle(void)
{
	return camera_angle;
}

void Set_ROBS_Angle(int val)
{
	 ROBS_Angle=(val-2048)/19.72; 
}
int Get_ROBS_Angle()
{
 //ROBS_Angle=(val-2048)/19.72; 
	if(robs_datagetting)// 更新中 返回上一状态值
	{
			return (2048-ROBS_Angle)/19.72;
	}
	else//  更新完毕
	{
			if(robs_data[1]<0x30)
			{
				 ROBS_Angle=robs_data[0]-'0';
			}
			else if(robs_data[2]<0x30)			
			{
				ROBS_Angle=10*(robs_data[0]-'0')+(robs_data[1]-'0');
			}
			else if(robs_data[3]<0x30)	
			{
				ROBS_Angle=100*(robs_data[0]-'0')+10*(robs_data[1]-'0')+(robs_data[2]-'0');
			}
			else if(robs_data[3]>=0x30)
			{
				ROBS_Angle=1000*(robs_data[0]-'0')+100*(robs_data[1]-'0')+10*(robs_data[2]-'0')+robs_data[3]-'0';
			}
			return (2048-ROBS_Angle)/19.72;	
	}	
}





void updaterobs_data(uint8_t i,uint8_t data,uint8_t flag)
{
			 robs_data[i]=data;		
			 robs_datagetting = flag;
}
void xyTempUpdate(void)
{
		Set_POS_Xtemp(Get_POS_X());
		Set_POS_Ytemp(Get_POS_Y());
}

 void Set_Offset_X(float val)
	{
		offset_x+=val;
	}
	void Set_Offset_Y(float val)
	{
		offset_y+=val;
	}
	void Reset_Offset_X(void)
	{
      offset_x=0;
	}
	void Reset_Offset_Y(void)
	{
       offset_y=0;
	}
	void Reset_EcoPosition(void)
	{
		   Ecoposition=0;
	}
	
void Calculate(void)
{
	    float posX=0,posY=0;
	 		float act_x=Get_Original_POS_X();
			float act_y=Get_Original_POS_Y();
			float angle=Get_Original_Angle();
	
			 if(angle>180)
			 angle=angle-360;
		   if(angle<-180)
		   angle=angle+360;
			 Set_Angle(angle);
       #ifdef BLUE_FIELD
			 posX= (act_x*Cos(60.0)-act_y*Sin(60.0))/*坐标轴变换*/+147*(Cos(angle)-Cos(-30));
		   posY=-(act_x*Sin(60.0)+act_y*Cos(60.0))/*坐标轴变换*/-147*(Sin(angle)-Sin(-30));
			 #elif  defined RED_FIELD
			 posX= (act_x*Cos(120.0)-act_y*Sin(120.0))/*坐标轴变换*/+147*(Cos(angle)-Cos(-30));  //120为编码器坐标系x与世界坐标系x的夹角，-30为初始角度
		   posY= (act_x*Sin(120.0)+act_y*Cos(120.0))/*坐标轴变换*/-147*(Sin(angle)-Sin(-30));
			 #endif
			 Set_POS_X(posX);
			 Set_POS_Y(posY*1.0117f); 
}


/************************ (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
