#include "debug.h"
#include "usart.h"
#include "adc.h"
#include "GET_SET.h"
#include "walk.h"
#include "action_math.h"

extern int8_t debug_start_flag;
extern int8_t debug_char_flag ;	
extern int8_t debug_flag      ;
extern int8_t debug_time_flag ;

extern int    debug_tagVel[3]	;
extern int    debug_actVel[6] ;
extern float  debug_Err[3]		;//0-errValue,1-errAngle,2-errActvel
extern float  debug_Actvelx		;
extern float  debug_Actvely   ;
extern float  debug_change[3]	;//0-changvel,1-changeward,2-rotate
extern float  debug_pidPos[3]	;   //0-P,1-I,2-D
extern float  debug_pidAng[3] ;
extern float  debug_pidAcv[3]	;               
extern int8_t debug_errCla[3]	;
extern int8_t debug_errcCla[3];
extern int    debug_time[4] 	;
extern float  debug_current[6];

extern int8_t debug_data_number;
extern int8_t debug_graph_number ;
extern int8_t debug_BLUE_time		 ;
extern float  debug_encoderl		 ;
extern float  debug_encoderr		 ;
extern float  debug_windspeed		 ;
extern int    debug_ecoPosY			 ;
extern int    debug_ecoPosX			 ;
extern int    debug_accurateLaser;
extern int8_t debug_switch[2]    ;
extern int8_t debug_high_light   ;


void ChooseWay(void)
{
	#if    defined RECEIVED_BY_ASSISANT
  if(debug_char_flag==1&&debug_start_flag==1)ShowInstruction();
	#elif  defined RECEIVED_BY_MATLAB
	if(debug_start_flag==1)ShowbyGraph();
	#endif
}

void ShowInstruction(void)
{
			 #ifdef READ_POSY
			 USART_OUT(UART5,(uint8_t *)"y\t");                   //pos
			 #endif
			 #ifdef READ_ERRPOS
			 USART_OUT(UART5,(uint8_t *)"ePos\t");            //err
			 #endif
			 #ifdef READ_ERRANGLE
			 USART_OUT(UART5,(uint8_t *)"eAng\t");            //err
			 #endif
			 #ifdef READ_ERRACVEL
			 USART_OUT(UART5,(uint8_t *)"eAcv\t");            //err
			 #endif
			 #ifdef  READ_PID_POS
			 USART_OUT(UART5,(uint8_t *)"Posp\tPosi\tPosd\t");            //pidPos
			 #endif
			 #ifdef  READ_PID_ANGLE
			 USART_OUT(UART5,(uint8_t *)"Angp\tAngi\tAngd\t");            //pidAng
			 #endif
			 #ifdef  READ_PID_ACVEL
			 USART_OUT(UART5,(uint8_t *)"Acvp\tAcvi\tAcvd\t");            //pidAcv
			 #endif
			 #ifdef READ_ACTVEL_X
			 USART_OUT(UART5,(uint8_t *)"ActvelX\t");                      //actvel_all
			 #endif
			 #ifdef READ_ACTVEL_Y
			 USART_OUT(UART5,(uint8_t *)"ActvelY\t");                      //actvel_all
			 #endif
			 #ifdef READ_ADJ_VEL
			 USART_OUT(UART5,(uint8_t *)"Adj\t");   
			 #endif
			 #ifdef  READ_CPU
			 USART_OUT(UART5,(uint8_t *)"CPU\t");                         //cpu
			 #endif
			 #ifdef  READ_WHEEL_VEL 
			 USART_OUT(UART5,(uint8_t *)"tVel1\ttVel2\ttVel3\t");         //tagvel
			 USART_OUT(UART5,(uint8_t *)"aVel1\taVel2\taVel3\t");         //actvel
//			 USART_OUT(UART5,"tVel4\ttVel5\t");         //tagvel
//			 USART_OUT(UART5,"aVel4\taVel5\t");         //actvel
			 #endif
			 #ifdef  READ_CLAMB_VEL
			 USART_OUT(UART5,(uint8_t *)"aVel5\taVel6\t");
			 #endif
			 #ifdef  READ_CURRENT
			 USART_OUT(UART5,(uint8_t *)"aCur1\taCur2\taCur3\t");         //current
			 USART_OUT(UART5,(uint8_t *)"aCur4\taCur5\taCur6\t");         //current
			 #endif
			 #ifdef  READ_PID_ERRCLA
			 USART_OUT(UART5,(uint8_t *)"eCla1\teCla2\tCla3\t");          //errClass
			 USART_OUT(UART5,(uint8_t *)"ecCal1\tecCal2\tecCal3\t");      //errcClass
			 #endif
			 #ifdef  READ_CHANGE_VEL
			 USART_OUT(UART5,(uint8_t *)"Changevel\t");     //change
			 #endif
			 #ifdef  READ_CHANGE_WARD
			 USART_OUT(UART5,(uint8_t *)"ward\t");     //change
			 #endif
			 #ifdef  READ_ROTATE
			 USART_OUT(UART5,(uint8_t *)"rotate\t");     //change
			 #endif
			 #ifdef READ_POSX
			 USART_OUT(UART5,(uint8_t *)"x\t");                   //pos
			 #endif 
			 #ifdef READ_ANGLE
			 USART_OUT(UART5,(uint8_t *)"ang\t");                   //pos
			 #endif
			 #ifdef READ_ENCODER
			 USART_OUT(UART5,(uint8_t *)"Ofy\t");
			 #endif
			 #ifdef READ_TRACK_LASER
			 USART_OUT(UART5,(uint8_t *)"TLas\t");
			 #endif
			 #ifdef READ_POST_LASER
			 USART_OUT(UART5,(uint8_t *)"PLAS\t");
			 #endif
			 #ifdef READ_BASIC_LASER
			 USART_OUT(UART5,(uint8_t *)"BLAS\t");
			 #endif
			 #ifdef READ_FANFLAG
			 USART_OUT(UART5,(uint8_t *)"Fanflag\t");
			 #endif 
			 #ifdef READ_FANFLAG_LIGHT
			 USART_OUT(UART5,(uint8_t *)"flaglight\t");
			 #endif 
			 #ifdef READ_FLAG_BT
			 USART_OUT(UART5,(uint8_t *)"flagbt\t");
			 #endif 
			 #ifdef READ_WINDSPEED
			 USART_OUT(UART5,(uint8_t *)"WindS\t");
			 #endif
			 #ifdef READ_WINDANGLE
			 USART_OUT(UART5,(uint8_t *)"WindA\t");
			 #endif
			 #ifdef  READ_SWITCH
			 USART_OUT(UART5,(uint8_t *)"switch0\tswitch1\t");
		   #endif
			 #ifdef  READ_HIGHT_LIGHT  
       USART_OUT(UART5,(uint8_t *)"high_light\t");
			 #endif
			 #ifdef READ_ECOPOS
       USART_OUT(UART5,(uint8_t *)"eco_pos\t");
			 #endif
			 #ifdef READ_ECOANGLE
       USART_OUT(UART5,(uint8_t *)"eco_ang\t");
			 #endif
			 #ifdef READ_TIME
			 #ifdef BLUE_FIELD
			 USART_OUT(UART5,(uint8_t *)"BTime\t");                        //time
			 #elif defined RED_FIELD
			 USART_OUT(UART5,(uint8_t *)"RTime\t");                        //time
			 #endif
			 #endif 
			 debug_char_flag=0;
			 debug_start_flag=0;
			 USART_OUT(UART5,(uint8_t *)"\n");
}

void ShowbyGraph(void)
{
     #ifdef READ_POSY
		 debug_data_number++;
		 #endif
		 #ifdef READ_ERRPOS
		 debug_data_number++;
		 debug_graph_number++;
		 #endif
		 #ifdef READ_ERRANGLE
		 debug_data_number++;
		 debug_graph_number++;
		 #endif
		 #ifdef READ_ERRACVEL
		 debug_data_number++;
		 debug_graph_number++;
		 #endif
	   #ifdef READ_ACTVEL_Y
		 debug_data_number++;
		 #endif
		 #ifdef READ_PID_POS
		 debug_data_number+=3;
		 debug_graph_number+=3;
	   #endif
		 #ifdef READ_PID_ANGLE
		 debug_data_number+=3;
		 debug_graph_number+=3;
		 #endif
		 #ifdef READ_PID_ACVEL
		 debug_data_number+=3;
		 debug_graph_number+=3;
		 #endif
		 #ifdef READ_ACTVEL_X
		 debug_data_number++;
		 debug_graph_number++;
		 #endif
		 #ifdef  READ_ADJ_VEL
		 debug_data_number++;
		 #endif
		 #ifdef READ_CPU
		 debug_data_number++;
		 #endif
		 #ifdef READ_WHEEL_VEL      //¶ÁÂÖËÙ
		 debug_data_number+=6;                                    
		 #endif
		 #ifdef READ_CURRENT
		 debug_data_number+=6;
		 #endif
		 #ifdef READ_PID_ERRCLA
		 debug_data_number+=6;
		 #endif
		 #ifdef READ_CHANGE_VEL
		 debug_data_number++;
		 #endif
		 #ifdef READ_ROTATE
		 debug_data_number++;
		 #endif
		 #ifdef READ_POSX
		 debug_data_number++;
		 #endif
		 #ifdef READ_TRACK_LASER
		 debug_data_number++;
		 #endif
		 #ifdef READ_POST_LASER
		 debug_data_number++;
		 #endif
		 #ifdef READ_BASIC_LASER
		 debug_data_number++;
		 #endif

		 #ifdef READ_FANFLAG
		 debug_data_number++;
		 #endif
		 #ifdef READ_WINDSPEED
		 debug_data_number++;
		 #endif
		 #ifdef READ_ANGLE
		 debug_data_number++;
		 #endif
		 #ifdef  READ_SWITCH
     debug_data_number+=2; 
     #endif		 
     #ifdef  READ_HIGHT_LIGHT
		 debug_data_number++;
		 #endif
		 #ifdef READ_TIME
		 debug_BLUE_time=1;
		 #endif
	 
		 USART_OUT(UART5,(uint8_t *)"%d\n",debug_data_number);
		 USART_OUT(UART5,(uint8_t *)"%d\n",debug_graph_number);
		 USART_OUT(UART5,(uint8_t *)"%d\n",debug_BLUE_time);
		 #ifdef READ_ERRPOS
		 USART_OUT(UART5,(uint8_t *)"err-pos\n");
		 #endif
		 #ifdef READ_ERRANGLE
		 USART_OUT(UART5,(uint8_t *)"err-ang\n");
		 #endif
		 #ifdef READ_ERRACVEL
		 USART_OUT(UART5,(uint8_t *)"err-acv\n");
		 #endif
		 #ifdef READ_PID_POS
		 USART_OUT(UART5,(uint8_t *)"pos-p\n");
		 USART_OUT(UART5,(uint8_t *)"pos-i\n");
		 USART_OUT(UART5,(uint8_t *)"pos-d\n");
		 #endif
		 #ifdef READ_PID_ANGLE
		 USART_OUT(UART5,(uint8_t *)"ang-p\n");                       
		 USART_OUT(UART5,(uint8_t *)"ang-i\n");
		 USART_OUT(UART5,(uint8_t *)"ang-d\n");
		 #endif
		 #ifdef READ_PID_ACVEL
		 USART_OUT(UART5,(uint8_t *)"acv-p\n");                       
		 USART_OUT(UART5,(uint8_t *)"acv-i\n");
		 USART_OUT(UART5,(uint8_t *)"acv-d\n");
		 #endif
		 #ifdef READ_ACTVEL
		 USART_OUT(UART5,(uint8_t *)"actvel\n");                       
		 #endif
		 debug_start_flag=0;
		 
 }

 
void ShowtheEnd(void)
{
	#if  defined RECEIVED_BY_ASSISANT
 if(debug_start_flag==1||debug_start_flag==2)
		{
		if(debug_flag==0) USART_OUT(UART5,(uint8_t *)"0SL\r\n");
		if(debug_flag==1) USART_OUT(UART5,(uint8_t *)"SC\r\n");
		if(debug_flag==2) USART_OUT(UART5,(uint8_t *)"1SL\r\n");
		if(debug_flag==3) USART_OUT(UART5,(uint8_t *)"1LL\r\n");
	  if(debug_flag==4) USART_OUT(UART5,(uint8_t *)"2SL\r\n");
	  if(debug_flag==5) USART_OUT(UART5,(uint8_t *)"2LL\r\n");
	  if(debug_flag==6) USART_OUT(UART5,(uint8_t *)"HC\r\n");
	  if(debug_flag==7) USART_OUT(UART5,(uint8_t *)"0L\r\n");
	  if(debug_flag==8) USART_OUT(UART5,(uint8_t *)"0-90C\r\n");	
	  if(debug_flag==9) USART_OUT(UART5,(uint8_t *)"HSL\r\n");
	  if(debug_flag==10)USART_OUT(UART5,(uint8_t *)"LSL\r\n");
		if(debug_flag==11)USART_OUT(UART5,(uint8_t *)"P11\r\n");
		if(debug_flag==12)USART_OUT(UART5,(uint8_t *)"P12\r\n");
		if(debug_flag==13)USART_OUT(UART5,(uint8_t *)"P13\r\n");
		if(debug_flag==14)USART_OUT(UART5,(uint8_t *)"P14\r\n");
		if(debug_flag==15)USART_OUT(UART5,(uint8_t *)"P15\r\n");
			
		if(debug_flag==30)USART_OUT(UART5,(uint8_t *)"YJ\r\n");
		if(debug_flag==40)USART_OUT(UART5,(uint8_t *)"XJ\r\n");
			
		if(debug_flag==RESTART_IN_ORIGIN)  USART_OUT(UART5,(uint8_t *)"RO\r\n"  );
		if(debug_flag==RESTART_IN_ORIGIN+1)USART_OUT(UART5,(uint8_t *)"RXYO\r\n");
		if(debug_flag==RESTART_IN_ORIGIN+2)USART_OUT(UART5,(uint8_t *)"RO+2\r\n");	
		
		if(debug_flag==RESTART_IN_A2)  USART_OUT(UART5,(uint8_t *)"A2\r\n"  );
		if(debug_flag==RESTART_IN_A2+1)USART_OUT(UART5,(uint8_t *)"A2+1\r\n");
		if(debug_flag==RESTART_IN_A2+2)USART_OUT(UART5,(uint8_t *)"A2+2\r\n");
		if(debug_flag==RESTART_IN_A2+3)USART_OUT(UART5,(uint8_t *)"A2+3\r\n");
		
		if(debug_flag>15&&debug_flag<30)USART_OUT(UART5,(uint8_t *)"\n");
		}
	#elif  defined RECEIVED_BY_MATLAB
		{
			#ifdef READ_TIME
			if(debug_flag==16) 
			{
			USART_OUT(UART5,(uint8_t *)"\n");
			USART_OUT(UART5,(uint8_t *)"%d\t",debug_time[0]);
			USART_OUT(UART5,(uint8_t *)"%d\t",debug_time[1]);
			USART_OUT(UART5,(uint8_t *)"%d\t",debug_time[2]);
			USART_OUT(UART5,(uint8_t *)"%d\n",debug_time[3]);
			}
			#endif
		}
		#endif
}


void ShowData(void)
{
		#if    defined RECEIVED_BY_ASSISANT
		if(debug_start_flag==2&&debug_char_flag==0){
		#elif  defined RECEIVED_BY_MATLAB
		if(debug_start_flag==2){
		#endif
	 
			 #ifdef READ_POSY
			 USART_OUT(UART5,(uint8_t *)"%d\t",(int)Get_POS_Y());
			 #endif		 
			 #ifdef READ_ERRPOS
			 USART_OUT(UART5,(uint8_t *)"%d\t", (int)debug_Err[0]);
			 #endif
			 #ifdef READ_ERRANGLE
			 USART_OUT(UART5,(uint8_t *)"%d\t", (int)debug_Err[1]);
			 #endif
			 #ifdef READ_ERRACVEL
			 USART_OUT(UART5,(uint8_t *)"%d\t", (int)debug_Err[2]);
			 #endif
			 #ifdef  READ_PID_POS
			 USART_OUT(UART5,(uint8_t *)"%d\t%d\t%d\t",(int)(debug_pidPos[0]*100),(int)(debug_pidPos[1]*1000),(int)(debug_pidPos[2]*100));
			 #endif
			 #ifdef READ_PID_ANGLE
			 USART_OUT(UART5,(uint8_t *)"%d\t%d\t%d\t",(int)(debug_pidAng[0]),(int)(debug_pidAng[1]*10),(int)(debug_pidAng[2]));
			 #endif
			 #ifdef READ_PID_ACVEL
			 USART_OUT(UART5,(uint8_t *)"%d\t%d\t%d\t",(int)(debug_pidAcv[0]*100),(int)(debug_pidAng[1]*1000),(int)(debug_pidAng[2]*100));
			 #endif		 
			 #ifdef  READ_ACTVEL_X
			 USART_OUT(UART5,(uint8_t *)"%d\t",(int)debug_Actvelx);
			 #endif
			 #ifdef READ_ACTVEL_Y
		   USART_OUT(UART5,(uint8_t *)"%d\t",(int)debug_Actvely);
		   #endif
			 #ifdef  READ_ADJ_VEL
			 USART_OUT(UART5,(uint8_t *)"%d\t",Get_AdjVel());
			 #endif	
			 #ifdef  READ_CPU
			 USART_OUT(UART5,(uint8_t *)"%d\t",(int8_t)OSCPUUsage);
			 #endif	 
			 #ifdef  READ_WHEEL_VEL
//			  USART_OUT(UART5,"%d\t%d\t",debug_tagVel[4]/10,debug_tagVel[5]/10);
//  			USART_OUT(UART5,"%d\t%d\t",debug_actVel[4]/10,debug_actVel[5]/10);
			 USART_OUT(UART5,(uint8_t *)"%d\t%d\t%d\t",debug_tagVel[0]/10,debug_tagVel[1]/10,debug_tagVel[2]/10);
			 USART_OUT(UART5,(uint8_t *)"%d\t%d\t%d\t",debug_actVel[0]/10,debug_actVel[1]/10,debug_actVel[2]/10);
			 #endif
			 #ifdef  READ_CLAMB_VEL
			 USART_OUT(UART5,(uint8_t *)"%d\t%d\t",debug_actVel[4]/10,debug_actVel[5]/10);
			 #endif   
			 #ifdef  READ_CURRENT
			 USART_OUT(UART5,(uint8_t *)"%d\t%d\t%d\t",(int)(debug_current[0]*1000),(int)(debug_current[1]*1000),(int)(debug_current[2]*1000));
			 USART_OUT(UART5,(uint8_t *)"%d\t%d\t%d\t",(int)(debug_current[3]*1000),(int)(debug_current[4]*1000),(int)(debug_current[5]*1000));	
			 #endif 
			 #ifdef  READ_PID_ERRCLA		 
			 USART_OUT(UART5,(uint8_t *)"%d\t%d\t%d\t",(int)debug_errCla[0],(int)debug_errCla[1],(int)debug_errCla[2]);
			 USART_OUT(UART5,(uint8_t *)"%d\t%d\t%d\t",(int)debug_errcCla[0],(int)debug_errcCla[1],(int)debug_errcCla[2]);
			 #endif 
			 #ifdef  READ_CHANGE_VEL 
			 USART_OUT(UART5,(uint8_t *)"%d\t",(int)debug_change[0]);
			 #endif
			 #ifdef  READ_CHANGE_WARD	 
			 USART_OUT(UART5,(uint8_t *)"%d\t",(int)debug_change[1]);
			 #endif
			 #ifdef  READ_ROTATE	 
			 USART_OUT(UART5,(uint8_t *)"%d\t",(int)debug_change[2]);
			 #endif
			 #ifdef READ_POSX
			 USART_OUT(UART5,(uint8_t *)"%d\t", (int)Get_POS_X());
			 #endif
			 #ifdef READ_ANGLE
			 USART_OUT(UART5,(uint8_t *)"%d\t",(int)(Get_Angle()*100));
			 #endif
			 #ifdef READ_TRACK_LASER
			 USART_OUT(UART5,(uint8_t *)"%d\t",debug_accurateLaser);
			 #endif
			 #ifdef READ_POST_LASER
			 if(Get_POS_Y()>10000)
			 USART_OUT(UART5,(uint8_t *)"%d\t",(int)(GetLaserValue(POST)*Cos(Get_Angle())));
			 else 
			 USART_OUT(UART5,(uint8_t *)"%d\t",GetLaserValue(POST));
			 #endif
			 #ifdef READ_BASIC_LASER
			 USART_OUT(UART5,(uint8_t *)"%d\t",GetLaserValue(BASIC));
			 #endif

			 #ifdef READ_FANFLAG
			 USART_OUT(UART5,(uint8_t *)"%d\t",Get_FanFlag());
			 #endif
			  #ifdef READ_FANFLAG_LIGHT
			 USART_OUT(UART5,(uint8_t *)"%d\t",Get_FanFlag_light());
			 #endif 
			  #ifdef READ_FLAG_BT
			 USART_OUT(UART5,(uint8_t *)"%d\t",Get_BlueTooth_Flag());
			 #endif 
			 #ifdef READ_WINDSPEED
			 USART_OUT(UART5,(uint8_t *)"%d\t",(int)(debug_windspeed*1000));
			 #endif
			 #ifdef READ_WINDANGLE
			 USART_OUT(UART5,(uint8_t *)"%d\t",Get_ROBS_Angle());
			 #endif
			 #ifdef READ_ENCODER
			 USART_OUT(UART5,(uint8_t *)"%d\t%d\t",(int)(debug_encoderl),(int)(debug_encoderr));
			 #endif 
			 #ifdef  READ_SWITCH
			 USART_OUT(UART5,(uint8_t *)"%d\t%d\t",debug_switch[0],debug_switch[1]);
		   #endif
			 #ifdef  READ_HIGHT_LIGHT
       USART_OUT(UART5,(uint8_t *)"%d\t",debug_high_light);
			 #endif
			 #ifdef READ_ECOPOS
       USART_OUT(UART5,(uint8_t *)"%d\t",Get_EcoPosition());
			 #endif
			 #ifdef READ_ECOANGLE
       USART_OUT(UART5,(uint8_t *)"%d\t",Get_Camera_Angle());
			 #endif
			 #ifdef RECEIVED_BY_ASSISANT
				 #ifdef READ_TIME
				 if(debug_time_flag==0)USART_OUT(UART5,(uint8_t *)"0\t"); 
				 if(debug_time_flag==1)USART_OUT(UART5,(uint8_t *)"%d\t",debug_time[0]);
				 if(debug_time_flag==2)USART_OUT(UART5,(uint8_t *)"%d\t",debug_time[1]);
				 if(debug_time_flag==3)USART_OUT(UART5,(uint8_t *)"%d\t",debug_time[2]);
				 if(debug_time_flag==4)USART_OUT(UART5,(uint8_t *)"%d\t",debug_time[3]);
				 //debug_time_flag=0;
				 #endif
			 #endif	 
		}
}
