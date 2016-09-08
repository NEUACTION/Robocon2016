/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include  <ucos_ii.h>
#include "GET_SET.h"
/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include "math.h"
#include "adc.h"
#include "usart.h"
#include "can.h"
#include "robs.h"
#include "walk.h"
#include "String.h"
#include "stm32f4xx_dma.h"
#include "gpio.h"
#include "stm32f4xx_exti.h"
#include "elmo.h"
#include "action_math.h"


#define d_circle                 145         //Distance between Encoder and Centrality of Automaton
#define d_angle                  60          //Angle between Initial coordinate and Word coordinate

extern int timeflag;
extern int timenumber;
/************************************************************/
/****************驱动器CAN1接口模块****start******************/
int receiveCount;    //临时使用，测试can通信
int iii;
void CAN1_RX0_IRQHandler(void)
{
	
	OS_CPU_SR  cpu_sr;
	
	static int preTim=0;
	int        curTim=0;
	
	
	union AngleMessage
  {
	 int8_t buffer8[2];
	 int16_t  buffer16;
  }angleMessage;//摄像头传回的角度信息
  union EcoMessage
	{
		uint8_t buffer[2];
		uint16_t u16data;
  };
  union EcoMessage Fanflag,Ecoposition;
	static uint8_t buffer[8];
	static uint32_t StdId=0;
	union can_message receive_pos,receive_current,receive_vel;   //驱动器位置变量
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	CAN_RxMsg(CAN1, &StdId,buffer,8);
	curTim=timenumber;
	
	if((StdId==0x281||StdId==0x282||StdId==0x283||StdId==0x284||StdId==0x285||StdId==0x286)&&(buffer[0]==0x50)&&(buffer[1]==0x58))     //get pos value
	{
		receive_pos.data8[0]=buffer[4];
		receive_pos.data8[1]=buffer[5];
		receive_pos.data8[2]=buffer[6];
		receive_pos.data8[3]=buffer[7];  //receive_pos4.data32_t
		if(StdId==0x281) Set_Pos(receive_pos.data32_t,1);
		if(StdId==0x282) Set_Pos(receive_pos.data32_t,2);
		if(StdId==0x283) Set_Pos(receive_pos.data32_t,3);
		if(StdId==0x284) Set_Pos(receive_pos.data32_t,4);
		if(StdId==0x285) Set_Pos(receive_pos.data32_t,5);
		if(StdId==0x286) Set_Pos(receive_pos.data32_t,6);
	}
	if((StdId==0x281||StdId==0x282||StdId==0x283||StdId==0x284||StdId==0x285||StdId==0x286)&&(buffer[0]==0x56)&&(buffer[1]==0x58))     //get pos value
	{
		receive_vel.data8[0]=buffer[4];
		receive_vel.data8[1]=buffer[5];
		receive_vel.data8[2]=buffer[6];
		receive_vel.data8[3]=buffer[7];  //receive_pos4.data32_t
		if(StdId==0x281) Set_Vel(receive_vel.data32_t,1);
		if(StdId==0x282) Set_Vel(receive_vel.data32_t,2);
		if(StdId==0x283) Set_Vel(receive_vel.data32_t,3);
		if(StdId==0x284) Set_Vel(receive_vel.data32_t,4);
		if(StdId==0x285) Set_Vel(receive_vel.data32_t,5);
		if(StdId==0x286) Set_Vel(receive_vel.data32_t,6);
	}
	if((StdId==0x281||StdId==0x282||StdId==0x283||StdId==0x284||StdId==0x285||StdId==0x286)&&(buffer[0]==0x49)&&(buffer[1]==0x51))
  {
			 receive_current.data8[0]=buffer[4];
	     receive_current.data8[1]=buffer[5];
			 receive_current.data8[2]=buffer[6];
	     receive_current.data8[3]=buffer[7];
		if(StdId==0x281) Set_Current(receive_current.dataf,1);
		if(StdId==0x282) Set_Current(receive_current.dataf,2);
		if(StdId==0x283) Set_Current(receive_current.dataf,3);
		if(StdId==0x284) Set_Current(receive_current.dataf,4);
		if(StdId==0x285) Set_Current(receive_current.dataf,5);
		if(StdId==0x286) Set_Current(receive_current.dataf,6);
	}
  if(StdId==0x40&&buffer[0]=='y')
	{
		 angleMessage.buffer8[0]=buffer[1];
		 angleMessage.buffer8[1]=buffer[2];
		 Set_Camera_Angle(angleMessage.buffer16);
		 Set_FanFlag_light(buffer[3]);
	}
	if(StdId==0x40&&buffer[0]=='n')
	{
		 angleMessage.buffer16=OUT_OF_SIGHT;
		 Set_Camera_Angle(angleMessage.buffer16);
	}
  if(StdId==0x66)
	{
		 Ecoposition.buffer[0]=buffer[0];
		 Ecoposition.buffer[1]=buffer[1];
		 Set_EcoPosition(Ecoposition.u16data);
     Fanflag.buffer[0]=buffer[2];
		 Fanflag.buffer[1]=buffer[3];//  1正常 2小风  1大风 0收风
		 Set_FanFlag(Fanflag.u16data);
		 preTim=curTim;		 
	}		
	
 //连续80ms没有收到
  if(curTim-preTim>8)
	  Set_BlueTooth_Flag(0);
  else 
		#ifdef BLUE_TOOTH
		Set_BlueTooth_Flag(1);
	  #else
	  Set_BlueTooth_Flag(0);
	  #endif
	CAN_ClearFlag(CAN1,CAN_FLAG_EWG);
	CAN_ClearFlag(CAN1,CAN_FLAG_EPV);
	CAN_ClearFlag(CAN1,CAN_FLAG_BOF);
	CAN_ClearFlag(CAN1,CAN_FLAG_LEC);
	
	CAN_ClearFlag(CAN1,CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN1,CAN_FLAG_FF0);
	CAN_ClearFlag(CAN1,CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN1,CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN1,CAN_FLAG_FF1);
	CAN_ClearFlag(CAN1,CAN_FLAG_FOV1);
	OSIntExit();
}

/****************驱动器CAN1接口模块****end******************/
/************************************************************/

/*************定时器2******start************/
//每1ms调用一次  用于读取编码器的值和计算坐标

extern  OS_EVENT 		*PeriodSem;
extern  OS_EVENT 		*CalculateSem; 

//int posx,posy;
void TIM2_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
   
	
	if(TIM_GetITStatus(TIM2, TIM_IT_Update)==SET)
	{
		 OSSemPost(PeriodSem);
     SetActualVel();
		 if(timeflag==1)timenumber++;
		 TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
   }
	 OSIntExit();
}


//定时器1  左编码器中断
void TIM1_UP_TIM10_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if(TIM_GetITStatus(TIM1, TIM_IT_Update)==SET)    
	{                                                
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
	OSIntExit();
}

//定时器8  右编码器中断
void TIM8_UP_TIM13_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if(TIM_GetITStatus(TIM8, TIM_IT_Update)==SET)    
	{                                                
		TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
	}
	OSIntExit();
}

/********************************************************/
/*****************普通定时TIM5*****Start*****************/
void TIM5_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if(TIM_GetITStatus(TIM5, TIM_IT_Update)==SET)    
	{              
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	}
	OSIntExit();
}


void TIM3_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if(TIM_GetITStatus(TIM3, TIM_IT_Update)==SET)    
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
	OSIntExit();
}



//定时器4  
void TIM4_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if(TIM_GetITStatus(TIM4, TIM_IT_Update)==SET)
	{                                  
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
	OSIntExit();
}



/********************************************************/
/****************陀螺仪接受中断****start****************/
/********************************************************/

float pos_x=0;
float pos_y=0;
float zangle=0;

void USART3_IRQHandler(void)
{	 
	static uint8_t ch;
	static union
  {
	 uint8_t data[12];
	 float ActVal[3];
  }posture;
	static uint8_t count=0;
	static uint8_t i=0;
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if(USART_GetITStatus(USART3, USART_IT_RXNE)==SET)   
	{
		USART_ClearITPendingBit( USART3,USART_IT_RXNE);
		ch=USART_ReceiveData(USART3);
		 switch(count)
		 {
			 case 0:
				 if(ch==0x0d)
					 count++;
				 else
					 count=0;
				 break;
				 
			 case 1:
				 if(ch==0x0a)
				 {
					 i=0;
					 count++;
				 }
				 else if(ch==0x0d);
				 else
					 count=0;
				 break;
				 
			 case 2:
				 posture.data[i]=ch;
			   i++;
			   if(i>=12)
				 {
					 i=0;
					 count++;
				 }
				 break;
				 
			 case 3:
				 if(ch==0x0a)
					 count++;
				 else
					 count=0;
				 break;
			 case 4:
				 if(ch==0x0d)
				 {
  				 zangle=posture.ActVal[0];
			     pos_x =posture.ActVal[1];
			     pos_y =posture.ActVal[2];
					 #ifdef  BLUE_FIELD
					 Set_Original_Angle(zangle-30);
					 #elif	 defined RED_FIELD
           Set_Original_Angle(-zangle-30);
					 #endif
					 Set_Original_POS_X(pos_x);
					 Set_Original_POS_Y(pos_y);
					 OSSemPost(CalculateSem);
					 
					 

					 

				 }
			   count=0;
				 break;
			 
			 default:
				 count=0;
			   break;		 
		 }
		 
		 
	 }
	OSIntExit();
}

/******************舵机串口****************/
void USART1_IRQHandler(void)
{
	static uint8_t count=0;
	uint8_t ch;
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	
	 if(USART_GetITStatus(USART1, USART_IT_RXNE)==SET)   
	 {
			USART_ClearITPendingBit( USART1,USART_IT_RXNE);
		  ch=USART_ReceiveData(USART1);
		
		  switch(count)
		  {
			 case 0:
				 if(ch=='@')
					 count++;
				 else
					 count=0;
				 break;
			 case 1:
				 if(ch=='1')
					 count++;
				 else 
					 count=0;
				 break;
			 case 2:
				 //if(ch==' ')
					 count++;
				 //else
					// count=0;
				 break;
			case 3:
         if(ch=='A')
					 count++;
				 else
					 count=0;
				 break;
			 case 4:
				 if(ch=='C')
					 count++;
				 else
					 count=0;
				 break;
			 case 5:
				 if(ch=='K')
					 count++;
				 else
					 count=0;
				 break;
			 case 6:
				 if(ch==' ')
					 count++;
				 else
					 count=0;
				 break;
			 case 7:
				 if(ch=='5')
					 count++;
				 else
					 count=0;
				 break;
			 case 8:
				 if(ch=='6')
					 count++;
				 else
					 count=0;
				 break;
			 case 9:
				 if(ch==',')
					 count++;
				 else
					 count=0;
				 break;
			 case 10:
				 if(ch=='2')
					 count++;
				 else
					 count=0;
				 break;
			 case 11:
				 if(ch==',')
					 count++;
				 else
					 count=0;
				 break;
			 case 12:
				 if(ch>=0x30)
				 {

            updaterobs_data(0,ch,1);	
				    count++;
				 }
				 else
				 {		   
					   count=0; 
				 }

				 break;	
			 case 13:
				 if(ch>=0x30)
				 {
					  updaterobs_data(1,ch,1);	
				    count++;
				 }
				 else
				 {
					   count=0;
					   updaterobs_data(1,ch,1);
             updaterobs_data(2,ch,1);
             updaterobs_data(3,ch,0);					 
				 }
				 break;	
			 case 14:
				 if(ch>=0x30)
				 {
					  updaterobs_data(2,ch,1);	
				    count++;
				 }
				 else
				 {
					   count=0;
					   updaterobs_data(2,ch,1);
             updaterobs_data(3,ch,0);					 
				 }
				 break;	
			case 15:
				 if(ch>=0x30)
				 {
					  updaterobs_data(3,ch,0);	
				    count=0;
				 }
				 else
				 {
					   count=0;
             updaterobs_data(3,ch,0);					 
				 }
				 break;	 
			 default:
				 count=0;
			   break;		 
	  }
			
	}
	 
   OSIntExit();
}

/******************蓝牙串口****************/
void UART4_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	
	 if(USART_GetITStatus(UART4, USART_IT_RXNE)==SET)   
	 {
			USART_ClearITPendingBit( UART4,USART_IT_RXNE);
	 }
	 
   OSIntExit();
}

//调试串口中断
void UART5_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	 if(USART_GetITStatus(UART5, USART_IT_RXNE)==SET)   
	 {
		USART_ClearITPendingBit( UART5,USART_IT_RXNE);

	 }
	  OSIntExit();	 
	 
}


/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
   while (1)
   {
   }
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{

  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{

  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
 
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

