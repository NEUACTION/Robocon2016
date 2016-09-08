
#include  <includes.h>
#include  <app_cfg.h>
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "walk.h"
#include "adc.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "elmo.h"
#include "cylinder.h"
#include "robs.h"
#include "action_math.h"
#include "GET_SET.h"
#include "fuzzypid.h"
#include "ucos_ii.h"
#include "track.h"
#include "debug.h"
#include "SelfCheck.h"

//////////////////Area of defining semaphore////////////////////////
OS_EVENT 		  			*PeriodSem;
OS_EVENT 						*CalculateSem;
OS_EVENT    				*DebugSem;
OS_EXT    INT8U     OSCPUUsage;


int8_t status						   = 0 ;
int    timeflag						 = 0 ;
int    timenumber					 = 0 ;
int8_t laser_flag          = 0 ;

#ifdef DEBUG
int8_t debug_start_flag		 = 1 ;
int8_t debug_char_flag     = 1 ;	
int8_t debug_flag          = 0 ;
int8_t debug_time_flag     = 0 ;

int    debug_tagVel[6]		 ={0};
int    debug_actVel[6]     ={0};
float  debug_Err[3]				 ={0};//0-errValue,1-errAngle,2-errActvel
float  debug_Actvelx			 = 0 ;
float  debug_Actvely       = 0 ;
float  debug_change[3]		 ={0};//0-changvel,1-changeward,2-rotate
float  debug_pidPos[3]		 ={0};   //0-P,1-I,2-D
float  debug_pidAng[3] 		 ={0};
float  debug_pidAcv[3]		 ={0};               
int8_t debug_errCla[3]	   ={0};
int8_t debug_errcCla[3]		 ={0};
int    debug_time[4] 			 ={0};
float  debug_current[6]		 ={0};

int8_t debug_data_number   = 0 ;
int8_t debug_graph_number  = 0 ;
int8_t debug_BLUE_time		 = 0 ;

float  debug_encoderl			 = 0 ;
float  debug_encoderr			 = 0 ;
float  debug_windspeed		 = 0 ;
int    debug_ecoPosY			 = 0 ;
int    debug_ecoPosX			 = 0 ;
int    debug_accurateLaser = 0 ;
int8_t debug_switch[2]     ={0};
int8_t debug_high_light    = 0 ;

#endif

   
void App_Task()
{
	CPU_INT08U  os_err;
	os_err = os_err; /* prevent warning... */
	
	/******************Create Semaphore***********************/
    PeriodSem				=	OSSemCreate(0);
    CalculateSem    = OSSemCreate(0);
	  DebugSem        = OSSemCreate(0);
  /******************Create Task**************************/	
	 os_err = OSTaskCreate(	(void (*)(void *)) ConfigTask,					//Initial Task
	                      	(void          * ) 0,							
													(OS_STK        * )&App_ConfigStk[Config_TASK_START_STK_SIZE-1],		
													(INT8U           ) Config_TASK_START_PRIO  );	
						
													
	 os_err = OSTaskCreate(	(void (*)(void *)) WalkTask,					
	                      	(void          * ) 0,							
													(OS_STK        * )&WalkTaskStk[Walk_TASK_STK_SIZE-1],		
													(INT8U           ) Walk_TASK_PRIO  );
	 os_err = OSTaskCreate(	(void (*)(void *)) CalTask,					
	                      	(void          * ) 0,							
													(OS_STK        * )&CalTaskStk[Cal_TASK_STK_SIZE-1],		
													(INT8U           ) Cal_TASK_PRIO  );
   os_err = OSTaskCreate(  (void (*)(void *)) DebugTask,
                          (void          * ) 0,
                          (OS_STK        * )&DebugTaskStk[Debug_TASK_STK_SIZE-1],
                          (INT8U           ) Debug_TASK_PRIO );													
}

void ConfigTask(void)
{	
	  CPU_INT08U  os_err;
    int cy_count    =0;	//cylinder count
	  os_err = os_err;
	  /*********************电机抱死***********************************/
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	  CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8, GPIO_Pin_9);  //1 1

		elmo_Init();
		elmo_Enable(1);//Driver 1
		elmo_Enable(2);//Driver 2
		elmo_Enable(3);//Driver 3
		elmo_Enable(4);//fan_drivers
		elmo_Enable(5);//POST Driver 5
		elmo_Enable(6);//POST Driver 6
	  /*********************Cylinder INIT*****************/

		Unlash_Fan();
		Grab_HighPost();
		for(cy_count=0;cy_count<5;cy_count++)
		{
			Pull_Hand();
			TIM_Delayms(TIM5,8);
			Pull_Hand_Disable();
			TIM_Delayms(TIM5,120);
		}
		Pull_Hand(); 
	  HighPost_Disable();
		Unlash_LowPost();
		PropDown();
		/*********************ROBS_USART INIT***********************/
	  TIM_Delayms(TIM5,1000);
	  ROBS_USART1_Init(1000000);  //0 0
    Enable_ROBS();

	  /*********************PWM Init************************************/
    TIM3_DuctedFan_Init(2000-1,840-1);
	  Extra_DuctedFan_Init(2000-1,840-1);
		
		
		/*********************LASER INIT******************************/
	  ADC1mixed_DMA_Config();
	  TIM_Delayms(TIM5,8000);

	 /*********************BEEP INIT******************************/
	  BeepInit();
		LED_Init();
		#ifdef BLUE_FIELD
		BEEP_ON;
		LED_OFF;
		#elif defined RED_FIELD
		LED_ON;
		BEEP_OFF;
		#endif
		
		
	 /**********LightSwitch Init*******************************/
	  LIGHT_GRABFAN_L_INIT();
		LIGHT_GRABFAN_R_INIT();
		LIGHT_GRABPOST_INIT();
		LIGHT_LOWSPEED_INIT();
		LIGHT_HIGH_INIT();
		SWITCH_END_0_INIT();
		SWITCH_END_1_INIT();
		RESTART_POST_INIT();
		RESTART_DEPART_INIT();

		/*********************DEBUG_USART Init*********/
		BLE_UART5_Init(115200);
		/*********************GYRO_USART Init*********/
	  GYRO_USART3_Init(115200);  //1 1
		
		/*********************MOTOR DRIVE INTI**************************/
		Vel_cfg(1,5000000,5000000);  //
		Vel_cfg(2,5000000,5000000);  //configuring Acceleration and Deceleration
		Vel_cfg(3,10000000,10000000);  //configuring Acceleration and Deceleration
		Pos_cfg(4,3000000, 3000000,300000);
		Vel_cfg(5,10000000,10000000); 
		Vel_cfg(6,10000000,10000000);

 		/*********************TIMER INIT*******************************/
	  TIM_Init(TIM2,999,839,3,3);//3 3
  	OSSemSet(PeriodSem,0,&os_err);	
		OSTaskSuspend(OS_PRIO_SELF);
}

void WalkTask(void)
{
	CPU_INT08U  os_err;
	
	int8_t 						startFlag           =0;
	int8_t 						adjFlag             =0;
	int8_t						gyroFlag            =0;    
	int    						time_count          =0;
    int         		        cy_count            =0;
	int             	        switchCount         =0;
	int               			wash_wheel_count    =0;
	int               			push_handCount      =0;
	int              			restartCount    		=0;
	int               			selfCheckCount      =0;
	int              		    adjVel              =0;
	int              		    decvel              =0;
    int                         addvel              =0;
	int                         elmo_num            =0;
	float                       fpid_value[18];	
	FPID_TypeDef                fpid;
	
	FPIDpos_Set(  0.15,	0.01,		0.15,  0.02,	0.002,	0.03,fpid_value);
	FPIDangle_Set(60,		1.5,	  120,   10.00,	0.2,		30,  fpid_value);
	FPIDacvel_Set(0.15,	0.010,	0.10,  0.04,	0.003,	0.03,fpid_value);
	FPIDVal_Set(fpid_value,&fpid);

  while(DEF_TRUE)
	{
	  OSSemPend(PeriodSem,0,&os_err);     //release once semaphore per 5 millisecond
		/*每隔10ms更新一次激光值*/
	  AverageValue(Get_POS_Y());         
		/* 获得ROBS位置 */
		ReadROBSAngle();
		/* 获得6个轮子速度 */
		for(elmo_num=1;elmo_num<=6;elmo_num++)
		ReadActualVel(elmo_num);
		#ifdef READ_CURRENT
		for(elmo_num=1;elmo_num<=6;elmo_num++)    
		ReadActualCurrent(elmo_num);
		#endif
		ReadActualPos(4);
		
		/*获得编码器实际速度*/
		for(elmo_num=1;elmo_num<=6;elmo_num++)
		debug_actVel[elmo_num-1]=Get_Vel(elmo_num);
		
		#ifdef  READ_ACTVEL_Y
		debug_Actvely=Get_ActVel_Y();
		#endif
    #ifdef READ_CURRENT
		for(elmo_num=1;elmo_num<=6;elmo_num++)
		debug_current[elmo_num-1]=Get_Current(elmo_num);
		#endif
		#ifdef READ_SWITCH        
		debug_switch[0]  =SWITCH_END_0;
		debug_switch[1]  =SWITCH_END_1;
		#endif
		#ifdef READ_HIGHT_LIGHT
		debug_high_light =LIGHT_HIGH;//LIGHT_GRABFAN_L;
		#endif

		if(status<8)
		{
			 UpdataHeight(Get_POS_X(),Get_POS_Y());
			 UpdataAngle(Get_POS_X(),Get_POS_Y());
			 UpdataWindSpeed(Get_POS_X(),Get_POS_Y());
			 UpdataEcoCoor(Get_POS_X(),Get_POS_Y());	
			 UpdateAdjVel(status);
		}
		adjVel  =Get_AdjVel();
		
	
		/*重启部分开始*/
		if(RESTART_DEPART==0&&RESTART_POST==1&&(status<SELF_CHECK_CASE||status>(SELF_CHECK_CASE+3)))
		{
				restartCount++;
				if(restartCount==10)
				{
						BEEP_ON;
						TIM_Delayms(TIM5,500);
						BEEP_OFF;
						restartCount=0;
//					  if((status>=9 &&status<=11)||
//							  status==(RESTART_IN_POST+1)||
//						    status==(RESTART_IN_POST+2)) {
//									status=RESTART_IN_HILL;   /*高台处重启*/
//									debug_flag=RESTART_IN_HILL;
//								}
						if(status>=12&&status<=15)  /*A2处重启*/
						{
							status=RESTART_IN_A2;  
							debug_flag=RESTART_IN_A2;
						}
						else                             {
							status=RESTART_IN_ORIGIN; /*出发区重启*/
							debug_flag=RESTART_IN_ORIGIN;
						}
				}
		}
		else
		{
			restartCount=0;
		}		
		/*重启部分结束*/
		
		switch(status)
		{
			case 0:/*出发区直线*/
			   /*洗轮子部分*/
				 if(1==SWITCH_END_0&&1==SWITCH_END_1)
				 {
					 wash_wheel_count++;
					 if(wash_wheel_count==20)
					 {
						    wash_wheel_count=0;
								status=WASHING_WHEEL_CASE;
					 }
				 }
				 else 
				 {
					  wash_wheel_count=0;
				 }
					
				 /*自检部分*/
				 if(RESTART_DEPART==0&&RESTART_POST==0)
				 {
					  selfCheckCount++;
					  if(selfCheckCount==20)
						{
							 selfCheckCount=0;
							 status=SELF_CHECK_CASE;
							 BEEP_ON;
							 TIM_Delayms(TIM5,1000);
							 BEEP_OFF;
						}
				 }
				 else
				 {
					 selfCheckCount=0;
				 }
				 
				 /*正常程序部分*/
				 if(LIGHT_HIGH==1||startFlag==1)
				 {
					   if(gyroFlag==0)
						 {
							  USART_OUT(USART3,"ACT0");
							  gyroFlag=1;
						 }
						 #ifdef BLUE_FILED
						 BEEP_OFF;
						 #elif defined RED_FIELD
						 LED_OFF;
						 #endif
					   OPEN_EXTRA_DUCTED;
						 debug_flag=0;	
					   debug_start_flag=2;
					   startFlag=1;
					   timeflag=1;
					   addvel+=50;
					   if(addvel>=TAGVEL)addvel=TAGVEL;
					   FuzPidLine(addvel,90-5.0f,-30,Get_POS_X()*0.0875f,Get_POS_Y(),PLEFT,&fpid);
						 if(Get_POS_X()>=START_CIRCLE_X)
						 {
							  addvel=0;
							  xyTempUpdate();
							  SetFirst();
							  //Configuring Acceleration and Deceleration
								Vel_cfg(1,10000000,10000000);  
								Vel_cfg(2,10000000,10000000);  
								Vel_cfg(3,10000000,10000000);
								//Setting PID value by fuzzy control
							  FPIDpos_Set(  0.25,	 0.01,	0.30,  0.03,	0.003,	0.05,fpid_value);
							  FPIDangle_Set(100,		1.0,	 250,  25.00,	0.3,		30,  fpid_value);
							  FPIDacvel_Set(0.15,	0.010,	0.10,  0.04,	0.003,	0.03,fpid_value);
							  FPIDVal_Set(fpid_value,&fpid);						 
							  debug_flag++;						 
								status++;
						 }                                  
				 }
			  break;
			case 1://第一个120度弧				
				if(BasicCircle(TAGVEL+adjVel,90-5.0,-28,-30,RADIUS1,NOROTATE,&fpid)==CIRCLE_END)
				{
					 xyTempUpdate();
					 Grab_HighPost();
					 SetFirst();
					 debug_flag++;					
					 status++;
//					 FPIDpos_Set(  0.21,	 0.01,	0.30,  0.03,	0.003,	0.05,fpid_value);
//					 FPIDangle_Set(80,		1.0,	 250,  25.00,	0.3,		30,  fpid_value);
//					 FPIDacvel_Set(0.15,	0.010,	0.10,  0.04,	0.003,	0.03,fpid_value);
//					 FPIDVal_Set(fpid_value,&fpid);	
				}
			  break;
			case 2://过渡直线1
			  FuzPidLine(TAGVEL+adjVel,-30,-30,(LINE_OY-Get_POS_Y())/1.732f,Get_POS_X(),PRIGHT,&fpid);  //Tan(fabs(angle)),-PID_TAN,PRIGHT
	          if(Get_POS_Y()>POS_LINE1_END_Y)
				{
					 xyTempUpdate();
					 SetFirst();
					 debug_flag++;					
					 status++;
				}
				break;
			case 3://-30度直线(第一个山岗，可以用激光调节)
				addvel+=ADDVELL/15;
			  if(addvel>=ADDVELL)addvel=ADDVELL;
				FuzPidLine(TAGVEL+addvel+adjVel,-30,-30,Trans_Dis_to_Laser(803),GetLaserValue(BASIC),PLEFT,&fpid);
				if(Get_POS_Y()>=LASER_LINE1_END_Y)
				{	
					 addvel=0;
					 xyTempUpdate();
					 SetFirst();
					 debug_flag++;					
					 status++;
//					 FPIDpos_Set(  0.30,	 0.02,	0.30,  0.05,	0.003,	0.05,fpid_value);
//					 FPIDangle_Set(80,		1.0,	 250,  25.00,	0.3,		30,  fpid_value);
//					 FPIDacvel_Set(0.15,	0.010,	0.10,  0.04,	0.003,	0.03,fpid_value);
//					 FPIDVal_Set(fpid_value,&fpid);	
				}
			  break;
			case 4://过渡直线2
				SlopLine(TAGVEL+ADDVELL+adjVel,-30,-30,(LINE_OY-Get_POS_Y())/1.732f,Get_POS_X(),PRIGHT,&fpid);
			  if(Get_POS_Y()>=POS_LINE2_END_Y)
				{
					 xyTempUpdate();
					 //fuzzy pid value setting
					 FPIDpos_Set(  0.15,	0.01,		0.30,  0.03,	0.003,	0.05,fpid_value);
					 FPIDangle_Set(60.0,	1.0, 		90.0,   7.0,	0.3,  	10.0,fpid_value);
					 FPIDacvel_Set(0.15,	0.010,	0.10,  0.04,	0.003,	0.03,fpid_value);
					 FPIDVal_Set(fpid_value,&fpid);
					 //debug flag move to next case
					 debug_flag++;
					 SetFirst();	
					 status++;
				}
				break;
			case 5://-30度直线(第二个山岗，可以用激光调节)
				FuzPidLine(TAGVEL+ADDVELL+adjVel,-30,-30,Trans_Dis_to_Laser(497),GetLaserValue(BASIC),PLEFT,&fpid);
				if(Get_POS_Y()>=HIGH_CIRCLE_START_Y)
				{
					 xyTempUpdate();
					 SetFirst();
				   debug_flag++;
					 FPIDpos_Set(  0.20,	0.01,		0.30,  0.03,	0.003,	0.05,fpid_value);
					 FPIDangle_Set(60.0,	1.0, 		90.0,   7.0,	0.3,  	10.0,fpid_value);
					 FPIDacvel_Set(0.15,	0.010,	0.10,  0.04,	0.003,	0.03,fpid_value);
					 FPIDVal_Set(fpid_value,&fpid);					
					 status++;
				}
			  break;
			case 6: //30度弧
				debug_flag=6;
				if(Get_POS_Y()>=START_ADJX_Y&&!adjFlag)
				{
					adjFlag=1;
					AdjPosX(GetLaserValue(BASIC),0);
          debug_flag=40;					
				}
				if(GetLaserValue(BASIC)>500&&adjFlag==1)
				{
					adjFlag=2;
					AdjPosY(Get_POS_Y());
					debug_flag=30;
				}				
				decvel+=70;
			  if(decvel>HIGH_DEC)decvel=HIGH_DEC;
				if((HighCircle(TAGVEL+ADDVELL+300-decvel+adjVel,-30,0,0,RADIUS2,ISROTATE,&fpid)==CIRCLE_END))
				{
					decvel=0;
			 		AdjPosX(GetLaserValue(BASIC),1);
		      xyTempUpdate();
				  debug_time[0]=timenumber;//高台时间
					debug_time_flag=1;
					SetFirst();
					debug_flag=7;
					FPIDpos_Set(  0.15,	0.01,		0.30,  0.03,	0.003,	0.05,fpid_value);
					FPIDangle_Set(60.0,	1.0, 		90.0,   7.0,	0.3,  	10.0,fpid_value);
					FPIDacvel_Set(0.15,	0.010,	0.10,  0.04,	0.003,	0.03,fpid_value);
					FPIDVal_Set(fpid_value,&fpid);
				  status++;
				}
			 break;
			case 7://高台直线
				debug_flag=7;
				if(Get_POS_Y()>=START_ADJX_Y&&!adjFlag)
				{
					adjFlag=1;
					AdjPosX(GetLaserValue(BASIC),0);
          debug_flag=40;					
				}
				if(GetLaserValue(BASIC)>500&&adjFlag==1)
				{
					adjFlag=2;
					AdjPosY(Get_POS_Y());
					debug_flag=30;
				}
				FuzPidLine(TAGVEL+ADDVELL-HIGH_DEC+adjVel,0,0,Get_POS_Xtemp(),Get_POS_X(),PRIGHT,&fpid);				
				if(Get_POS_Y()>=LAST_CIRCLE_START_Y)
				{
					 CLOSE_EXTRA_DUCTED;					
					 xyTempUpdate();					 
					 Unlash_HighPost();
					 //fuzzy pid value setting
					 FPIDpos_Set(  0.15,	0.02,		0.30,  0.03,	0.005,	0.05,fpid_value);
					 FPIDangle_Set(100,		2.0,	  250,   25.00,	0.3,		30,	 fpid_value);
					 FPIDacvel_Set(0.15,	0.010,	0.10,  0.04,	0.003,	0.03,fpid_value);
					 FPIDVal_Set(fpid_value,&fpid);
					 //debug flag move to next case
					 SetFirst();	
					 debug_flag=8;					
					 status++;
					 adjFlag=0;
					 LED_ON;
				}
				break;
			case 8:  //最后半径3800的圆
				debug_flag=8;
			  BasicCircle(TAGVEL+ADDVEL,0,90,0,RADIUS3,NOROTATE,&fpid);
				if(push_handCount<8){
					push_handCount++;
					Push_Hand();
				}
				if(push_handCount==8) Pull_Hand_Disable();
				#ifdef BLUE_TOOTH
        if(Get_POS_X()>=1000) SendToEco((uint16_t)Get_POS_X());
				#endif
			  if(((( GetLaserValue(POST)*Cos(Get_Angle()))<=(250-(700-Get_ActVel_Y())/5)&&
					     Get_POS_Y()>12700&&(GetLaserValue(POST)*Cos(Get_Angle()))<=250))||
				       Get_POS_X()>=(LOWSPEED_X-150)) 
			  {
				  ROBS_PosCrl(0,3000);
				  xyTempUpdate();	
				  debug_flag=9;
				  Push_Hand();//放爪子					
				  SetFirst();
			    adjFlag=0;			
			    LED_OFF;				  
				  status++;
				}
        break;	
			case 9://冲刺风立柱	
			  debug_flag=9;
			  #ifdef BLUE_TOOTH
			  SendToEco((uint16_t)Get_POS_X());
			  #endif
				FuzPidLine(TAGVEL+ADDVEL,90,0,(LASER_YVALUE+0),(GetLaserValue(POST)-(Get_ActVel_Y()/5)),PRIGHT,&fpid);
			  if(Get_POS_X()>=(LOWSPEED_X-150))
				{
					FPIDpos_Set(  0.25,	0.02,		0.15,  0.03,	0.002,	0.03,fpid_value);
					FPIDangle_Set(60,		1.5,	  120,   10.00,	0.2,		30,	 fpid_value);
					FPIDacvel_Set(0.15,	0.010,	0.10,  0.04,	0.003,	0.03,fpid_value);
					FPIDVal_Set(fpid_value,&fpid);
					xyTempUpdate();
					Unlash_HighPost();//A2处重启用
					status++;
					adjFlag=0;
					SetFirst();
					debug_flag=10;
				}
	      break;
			case 10:  //减速进柱子
				#ifdef BLUE_TOOTH
			  if(switchCount<4)SendToEco((uint16_t)Get_POS_X());
			  #endif
				debug_flag=10;
			  decvel+=70;
			  if(decvel>TAGVEL-600)			decvel=TAGVEL-600;
			  FuzPidLine(TAGVEL-decvel,90,0,(LASER_YVALUE+0),(GetLaserValue(POST)-(Get_ActVel_Y()/5)),PRIGHT,&fpid);
			  if(LIGHT_GRABPOST==1)			switchCount++;
			  if(switchCount==10)
				{
					LED_ON;
					#ifdef BLUE_TOOTH
					SendToEco(102);
					#endif
					switchCount=0;
					Grab_HighPost();
          elmo_Disable(2);
					debug_time[1]=timenumber;
					debug_time_flag=2;
					debug_flag=11;
					status++;
				}	
				break;
			case 11: //与小车交接风扇
			    VelCrl(5, 450000);
		      VelCrl(6, 450000);
			    #ifdef BLUE_TOOTH
			    SendToEco(102);		
			    #endif
	    	  EndCloseLoopLine();
				if(LIGHT_GRABFAN_R==1&&fabs(Get_Angle())>=7)EndCloseLoopLine();		
				if(LIGHT_GRABFAN_R==1&&fabs(Get_Angle())<7)	
				{
					 LockWheel();
					 time_count=0;	//清零计数值，case12 要用
					 #ifdef BLUE_TOOTH
					 SendToEco(103);
					 TIM_Delayms(TIM5,10);
					 SendToEco(103);
					 TIM_Delayms(TIM5,10);
					 SendToEco(103);
					 #endif
					 Pos_cfg(4,500000, 500000,300000);
					 debug_time[2]=timenumber;
					 debug_time_flag=3;
					 LockWheel();
					 PropUp();
           PosCrl_mm(-495);
					 TIM_Delayms(TIM5,200);
					 Grab_Fan();					
					 //TIM_Delayms(TIM5,60);	 
					 Grab_LowPost();
					 for(cy_count=0;cy_count<5;cy_count++)
					 {
							Pull_Hand();
							TIM_Delayms(TIM5,10);
							Pull_Hand_Disable();
							TIM_Delayms(TIM5,120);
					 }
				   Pull_Hand();
					 OSSemSet(PeriodSem,0,&os_err);					 
					 status++;
					 debug_flag=12;
				}
        break;
			case 12: //将风扇放上风立柱
			 if(LIGHT_GRABFAN_R==0)    //如果风扇被夹飞，停在柱子上
			 {
				    switchCount++;
				    if(switchCount==10)
						{	  
							switchCount=0;
							VelCrl(5, 0);
							VelCrl(6, 0);
						}
			 }
			 else
			 {
					 if(LIGHT_HIGH==1)          //减速光电触发
					 {		
						  BEEP_ON;						 
						  VelCrl(5, 200000);
						  VelCrl(6, 200000);
						  debug_tagVel[4]=200000;
						  debug_tagVel[5]=200000;
					 }
					 if(LIGHT_GRABFAN_L==1)      //防止行程开关一直未触发
					 {
							 time_count++;
							 if(time_count>50)
							 {
									time_count=0;
									debug_switch[0]  =SWITCH_END_0;
									debug_switch[1]  =SWITCH_END_1;
									debug_high_light =LIGHT_HIGH;//LIGHT_GRABFAN_L;
								  VelCrl(5,0);
									VelCrl(6,0);
//									VelCrl(5,50000);
//									VelCrl(6,50000);
									timeflag=0;
									TIM_Delayms(TIM5,100);
									Unlash_Fan();
									TIM_Delayms(TIM5,100);
									PosCrl_mm(-465);
									for(cy_count=0;cy_count<5;cy_count++)
									{
										Push_Hand();
										if(cy_count>=2)
										PosCrl_mm(-265);	
										TIM_Delayms(TIM5,8);
										Pull_Hand_Disable();
										TIM_Delayms(TIM5,120);
									}
									VelCrl(5,0);
									VelCrl(6,0);
									debug_time[3]=timenumber;
									debug_time_flag=4;
									status++;
								  debug_flag=13;
									OSSemSet(PeriodSem,0,&os_err);
							 }
					 }
					 if(((SWITCH_END_0==1)||(SWITCH_END_1==1))&&(LIGHT_GRABFAN_L==1))    //正常触发行程开关
					 {
						  debug_switch[0]  =SWITCH_END_0;
						  debug_switch[1]  =SWITCH_END_1;
              debug_high_light =LIGHT_HIGH;//LIGHT_GRABFAN_L;
						  VelCrl(5,0);
						  VelCrl(6,0);
//						  VelCrl(5,50000);
//							VelCrl(6,50000);
							timeflag=0;
							TIM_Delayms(TIM5,100);
							Unlash_Fan();
						  TIM_Delayms(TIM5,100);
						  //PosCrl_mm(-465);
              //TIM_Delayms(TIM5,500);

							for(cy_count=0;cy_count<5;cy_count++)
							{
								Push_Hand();
								if(cy_count>=2)
								PosCrl_mm(-265);	
								TIM_Delayms(TIM5,8);
								Pull_Hand_Disable();
								TIM_Delayms(TIM5,120);
							}
							VelCrl(5,0);
							VelCrl(6,0);
							debug_time[3]=timenumber;
							debug_time_flag=4;
							status++;
							debug_flag=13;
						  OSSemSet(PeriodSem,0,&os_err);
					 }
			 }
			 break;
			case 13:
				/*往下降到减速光电不触发为止*/
				if(LIGHT_HIGH==0)
				{
					status++;
					debug_flag=14;
				}
				break;
			case 14://END
				VelCrl(5,0);
			  VelCrl(6,0);
			  break;
			
			case 21:
				LockWheel();
			  TIM_SetCompare2(TIM3,0.05*2000);
			  CLOSE_EXTRA_DUCTED;
			  break;
			
/***************Washing Wheels***************/
/********************************************/
			
			case WASHING_WHEEL_CASE:
            VelCrl(1,80000);
						VelCrl(2,80000);
						VelCrl(3,80000);
			      TIM_Delayms(TIM5,10000);
			      VelCrl(1,0);
						VelCrl(2,0);
						VelCrl(3,0);
			      TIM_Delayms(TIM5,1000);
            VelCrl(1,-80000);
						VelCrl(2,-80000);
						VelCrl(3,-80000);
			      TIM_Delayms(TIM5,10000);
						VelCrl(1,0);
						VelCrl(2,0);
						VelCrl(3,0);
			      TIM_Delayms(TIM5,1000);	
			  break;
						
/***************自检部分***************/
/**************************************/
			
			case SELF_CHECK_CASE:/*电机检测*/
				SendToEco(0);
			  delay_ms(10);
			  SendToEco(1);
			  delay_ms(10);
			  SendToEco(2);
			  delay_ms(10);
			  SendToEco(3);
			  delay_ms(10);
			  SendToEco(4);
			  delay_ms(10);
			  SendToEco(5);
				PosCrl_mm(-300);
				anticlockwise();/*正转1s*/
				delay_ms(3000);
				stop();
				delay_ms(500);
				clockwise();/*反转1s*/
				delay_ms(3000);
				stop();
				delay_ms(100);
			  /*爬柱子电机转*/
				VelCrl(5,150000);
				VelCrl(6,150000);
				Grab_LowPost();   //抓下面柱子
        delay_ms(1500);	
				Unlash_LowPost();   //松开下面柱子
				delay_ms(1500);
        VelCrl(5,0);
				VelCrl(6,0);							
				status++;
				break;
				
			case (SELF_CHECK_CASE+1):/*气缸检测*/
				for(cy_count=0;cy_count<10;cy_count++)/*放手臂*/
				{
					Push_Hand();//放爪子	
					TIM_Delayms(TIM5,15);
					Pull_Hand_Disable();//放爪子泄气
					TIM_Delayms(TIM5,80);
				}
				Push_Hand();				
				delay_ms(500);
				Grab_Fan();/*收爪子*/
				delay_ms(500); 
				Unlash_Fan();/*松爪子*/
				delay_ms(500);
				for(cy_count=0;cy_count<10;cy_count++)/*收手臂*/
				{
					Pull_Hand();         	
					TIM_Delayms(TIM5,15);
					Pull_Hand_Disable();  
					TIM_Delayms(TIM5,80);
				}
				Pull_Hand();         	
				delay_ms(1000);
				Unlash_HighPost(); /*上面夹柱子气缸松开*/    
				delay_ms(1000);
				Grab_HighPost();/*上面夹柱子气缸夹紧*/     
				delay_ms(1000);
				PropUp();/*撑柱子*/
				delay_ms(1000);
				status++;
				break;
				
			case (SELF_CHECK_CASE+2):/*传感器和涵道部分*/
					 if(LIGHT_GRABFAN_R==1)
						 {beepfan();delay_ms(200);debug_start_flag=2;}//激光旁边
					 if(LIGHT_GRABPOST==1)    
						 {beepfan();delay_ms(200);debug_start_flag=2;}
					 if(LIGHT_GRABFAN_L==1)    
						 {beepfan();delay_ms(200);}//另外一边
					 if(SWITCH_END_0==1)    
						 {beepfan();delay_ms(200);}
					 if(SWITCH_END_1==1)    
						 {beepfan();delay_ms(200);}
					 if(Get_FanFlag()==2)
						{
							beepfan();delay_ms(400);
						}
					if(RESTART_DEPART==0)
						{
							beepfan();delay_ms(400);
						}						
					if(RESTART_POST==0)
						{
							beepfan();delay_ms(400);
						}						
					 if(LIGHT_HIGH==1)    
					 {
						   ROBS_PosCrl(110,3000);
							 TIM_SetCompare2(TIM3,0.06*2000);
						   TIM_SetCompare3(TIM2,0.07*2000);						 
							 BEEP_OFF;
						   debug_start_flag=2;
							 LED_ON;delay_ms(500);LED_OFF;delay_ms(500);
							 LED_ON;delay_ms(500);LED_OFF;delay_ms(500);
						   ROBS_PosCrl(-110,3000);
							 LED_ON;delay_ms(500);LED_OFF;delay_ms(500);
							 LED_ON;delay_ms(500);LED_OFF;LED_OFF;
							 TIM_SetCompare2(TIM3,0.05*2000);
							 TIM_SetCompare3(TIM2,0.05*2000);
							 status++;
					 }	
				break;
			case (SELF_CHECK_CASE+3):
				debug_start_flag=2;
			  UpdataAngle(Get_POS_X(),Get_POS_Y());
				break;
		
/****************出发区1重启*****************/
/*********************************************/
			
			case RESTART_IN_ORIGIN:
				elmo_Disable(1);
				elmo_Disable(2);
				elmo_Disable(3);
				elmo_Disable(4);
				elmo_Disable(5);
				elmo_Disable(6);
			  Unlash_HighPost();
			  Unlash_LowPost();
				for(cy_count=0;cy_count<10;cy_count++)
				{
					Pull_Hand();
					TIM_Delayms(TIM5,10);
					Pull_Hand_Disable();
					TIM_Delayms(TIM5,60);
				}
				Pull_Hand();
				TIM_Delayms(TIM5,2000);//防止连续触发重启按钮
			  status++;
				debug_flag++;
				break;
			case (RESTART_IN_ORIGIN+1):
				if(RESTART_POST==0)
				{
							BEEP_ON;
					    elmo_Enable(1);//Driver 1
							elmo_Enable(2);//Driver 2
							elmo_Enable(3);//Driver 3
							elmo_Enable(4);//fan_drivers
							elmo_Enable(5);//POST Driver 5
							elmo_Enable(6);//POST Driver 6
					    //气缸初始化
					    Unlash_Fan();
							Grab_HighPost();

							Unlash_LowPost();
							PropDown();
					    USART_OUT(USART3,"ACT0");
							//变量初始化
							gyroFlag=0;
							adjFlag=0;
					    startFlag=0;
					    //ClearFirstFlag();
							InitStaticVar();
							Reset_Offset_X();
							Reset_Offset_Y();
							TIM_Delayms(TIM5,1000);
							BEEP_OFF;
							status=0;
							debug_flag++;
				}
				break;
				
/****************出发区2重启*****************/
/*********************************************/				
				
		  case RESTART_IN_A2:
				 debug_flag=RESTART_IN_A2;
				 elmo_Disable(1);
				 elmo_Disable(2);
				 elmo_Disable(3);
				 Unlash_Fan();
				 Unlash_LowPost();
				 PropDown();
				 startFlag=0;
				 TIM_Delayms(TIM5,2000);//防止连续触发重启按钮
				 status++;
				 debug_flag++;
			  break;
			case (RESTART_IN_A2+1):
				if(RESTART_POST==0)
				{
							BEEP_ON;
					    elmo_Enable(1);//Driver 1
						  elmo_Enable(2);//Driver 2
						  elmo_Enable(3);//Driver 3
					    LockWheel();
							
					    //气缸初始化
					    Grab_HighPost();
						  Grab_Fan();
					    TIM_Delayms(TIM5,500);
							for(cy_count=0;cy_count<10;cy_count++)
							{
								Pull_Hand();
								TIM_Delayms(TIM5,10);
								Pull_Hand_Disable();
								TIM_Delayms(TIM5,60);
							}
							Pull_Hand();
							PropDown();
							
							AdjPosX_A2(ACCURATE_A2_X); //矫正坐标
							FPIDpos_Set(  0.1,	0.00,		0.15,  0.003,	0.002,	0.03,fpid_value);
							FPIDangle_Set(60,		1.5,	  120,   10.00,	0.2,		30,	 fpid_value);
							FPIDacvel_Set(0.15,	0.010,	0.10,  0.04,	0.003,	0.03,fpid_value);
							FPIDVal_Set(fpid_value,&fpid);   
							TIM_Delayms(TIM5,500);
							BEEP_OFF;
  
							status++;
					    debug_flag++;
				}
			  break;
			case (RESTART_IN_A2+2):
				if(LIGHT_HIGH==1||startFlag==1)
				{
						startFlag=1;
						addvel+=50;
						if(addvel>=TAGVEL+500)addvel=TAGVEL+500;
						FuzPidLine(addvel,90,0,LASER_YVALUE,GetLaserValue(POST),PRIGHT,&fpid);							
						if(Get_POS_X()>=(LOWSPEED_X+200))
						{
							FPIDpos_Set(  0.20,	0.01,		0.15,  0.03,	0.002,	0.03,fpid_value);
							FPIDangle_Set(60,		1.5,	  120,   10.00,	0.2,		30,	 fpid_value);
							FPIDacvel_Set(0.15,	0.010,	0.10,  0.04,	0.003,	0.03,fpid_value);
							FPIDVal_Set(fpid_value,&fpid);
							xyTempUpdate();
							Unlash_HighPost();
							adjFlag=0;
							SetFirst();
							status++;
							debug_flag=RESTART_IN_A2+3;
						}
				}
				break;
					
			case (RESTART_IN_A2+3):
				debug_flag=10;
				decvel+=70;
				if(decvel>TAGVEL-1000)			decvel=TAGVEL-1000;
				FuzPidLine(TAGVEL-decvel,90,0,(LASER_YVALUE+0),(GetLaserValue(POST)-(Get_ActVel_Y()/5)),PRIGHT,&fpid);
				if(LIGHT_GRABPOST==1)			switchCount++;
				if(switchCount==10)
				{
					LED_ON;
					SendToEco(102);
					switchCount=0;
					Grab_HighPost();
					elmo_Disable(2);
					debug_time[1]=timenumber;
					debug_time_flag=2;
					debug_flag=11;
					status=11;
				}	
				break;
			case (RESTART_IN_A2+4):
				LockWheel();
				break;

			default:
				break;
		 }
		 OSSemPost(DebugSem);
	}
}

/****************计算坐标*****************/
/*****************************************/
void CalTask(void)
{
	  CPU_INT08U  os_err;
	  while(1)
		{
			 	  OSSemPend(CalculateSem,0,&os_err);
				  Calculate();
		}
}

/****************蓝牙收数*****************/
/*****************************************/
void DebugTask(void)
{
	CPU_INT08U os_err;
	#ifdef DEBUG
	ChooseWay();
  while(1)
	{
		OSSemPend(DebugSem,0,&os_err);
        ShowData();
		ShowtheEnd();
   }
	#endif	 
}
