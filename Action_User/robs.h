#ifndef __ROBS_H
#define __ROBS_H
//打开舵机的扭力输出
void Enable_ROBS(void);

//使能伺服模式
void Enable_ServoMode(void) ;

//使能电机模式
void Enable_MotorMode(void);

void ROBS_VelCrl(int vel,int time);

void ROBS_PosCrl(float angle,int vel);

void TurnLeft(int vel);

void TurnRight(int vel);

void Stop(void);

void ReadROBSAngle(void);


#endif
