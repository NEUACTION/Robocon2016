/****************************************Copyright (c)****************************************************
**                                      
**                                 http://www.powermcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               app_cfg.h
** Descriptions:            ucosii configuration
**
**--------------------------------------------------------------------------------------------------------
** Created by:              AVRman
** Created date:            2010-11-9
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/

#ifndef  __APP_CFG_H__
#define  __APP_CFG_H__
#include  <os_cpu.h>					  
/*
*********************************************************************************************************
*                                       MODULE ENABLE / DISABLE
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                              TASKS NAMES
*********************************************************************************************************
*/
extern  void  App_Task(void);

static  void  App_TaskStart(void); 
static 	void  ConfigTask(void);
static 	void  WalkTask(void);
static 	void  CalTask(void);
static  void  DebugTask(void);
/*
*********************************************************************************************************
*                                            TASK PRIORITIES
*********************************************************************************************************
*/

#define  APP_TASK_START_PRIO                               10u
#define  Config_TASK_START_PRIO                            11u
#define  Cal_TASK_PRIO                                     12u
#define  Walk_TASK_PRIO                                    13u
#define  Debug_TASK_PRIO                                   14u




/*
*********************************************************************************************************
*                                            TASK STACK SIZES
*                             Size of the task stacks (# of OS_STK entries)
*********************************************************************************************************
*/
#define  APP_TASK_START_STK_SIZE                          256u
#define  Config_TASK_START_STK_SIZE                       256u
#define  Cal_TASK_STK_SIZE                                256u
#define  Debug_TASK_STK_SIZE                              512u
#define  Walk_TASK_STK_SIZE                               512u

/*
*********************************************************************************************************
*                                            TASK STACK
*                             
*********************************************************************************************************
*/
static  OS_STK  App_TaskStartStk[APP_TASK_START_STK_SIZE];
static  OS_STK  App_ConfigStk[Config_TASK_START_STK_SIZE];
static  OS_STK  CalTaskStk[Cal_TASK_STK_SIZE];
static  OS_STK  WalkTaskStk[Walk_TASK_STK_SIZE];
static  OS_STK  DebugTaskStk[Debug_TASK_STK_SIZE];


/*
*********************************************************************************************************
*                                                  LIB
*********************************************************************************************************
*/



#endif

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/

