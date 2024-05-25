#ifndef __PID_H__
#define __PID_H__

#include "headfile.h"

typedef struct 
{
    float SumError;  //误差累积
	float CompensateError; //补偿误差
    float Kp;      //比例系数
    float Ki;      //积分系数
    float Kd;      //微分系数
	float Kc;	   // 积分补偿系数
    float LastError;  //上一次误差
    float LLastError; //上上次误差
}PID;

//结构体变量申明
extern PID TurnPID;
extern PID SpeedPID;
extern PID L_SpeedPID;
extern PID R_SpeedPID;
extern PID Turn_NeiPID;
extern PID TurnPID_wan;			//弯道分段PID
extern PID Turn_NeiPID_wan; //弯道分段PID
extern PID TurnPID_RTwan;			//直角弯道分段PID
extern PID Turn_NeiPID_RTwan; //直角弯道分段PID


//函数声明
void IncPIDInit(PID *sptr);
int16 LocP_DCalc(PID*sptr,int16 Setpoint,int16 Turepoint);
int16 IncPIDCalc(PID *sptr,int16 Setpoint,int16 Turepoint);//增量式PID控制
int16 PlacePID_Control(PID *sptr, int16 Setpoint,int16 Turepiont);//动态位置式PID控制

#endif
