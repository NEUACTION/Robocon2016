#ifndef __DEBUG_H
#define __DEBUG_H


//#ifdef DEBUG
//#pragma message("DEBUG macro activated!")
//#else
//#pragma message("DEBUG macro not activated!")
//#endif


/****************Debug Macro Define********************/
 #define RECEIVED_BY_ASSISANT
//#define RECEIVED_BY_MATLAB

 #define READ_POSY     
 #define READ_ERRPOS
 #define READ_ERRANGLE
// #define READ_ERRACVEL
// #define READ_PID_POS
// #define READ_PID_ANGLE      
// #define READ_PID_ACVEL
// #define READ_ACTVEL_X
 #define READ_ACTVEL_Y
 #define READ_ADJ_VEL
// #define READ_CPU
// #define READ_WHEEL_VEL      //读轮速
 #define READ_CLAMB_VEL
// #define READ_CURRENT        //读电流
// #define READ_PID_ERRCLA     //收PID数据
// #define READ_CHANGE_VEL
 #define READ_CHANGE_WARD
// #define READ_ROTATE
 #define  READ_POSX
 #define  READ_ANGLE
 
// #define  READ_TRACK_LASER
 #define  READ_POST_LASER
 #define  READ_BASIC_LASER
// #define  READ_FANFLAG
// #define  READ_FANFLAG_LIGHT
// #define  READ_FLAG_BT
 #define  READ_WINDSPEED     //读风速
// #define  READ_WINDANGLE    //读舵机角度
// #define  READ_SWITCH        
 #define  READ_HIGHT_LIGHT
// #define READ_ECOPOS
// #define READ_ECOANGLE
 #define  READ_TIME           //读时间

void ChooseWay(void);
void ShowInstruction(void);
void ShowbyGraph(void);
void ShowtheEnd(void);
void ShowData(void);


#endif
