#include "PID.h"
#define MAX_SUMERROR 2500
#define MAX_OUTPUT 1600
/************************************************
函数名：IncPIDInit(PID *sptr)
功  能：PID参数初始化
参  数：
返回值：void
************************************************/
void IncPIDInit(PID *sptr)
{
	sptr->CompensateError = 0;
    sptr->SumError=0;
    sptr->LastError=0.0001;
    sptr->LLastError=0;
    sptr->Kp=0;
    sptr->Ki=1000;
    sptr->Kd=0;
}
/************************************************
函数名：LocP_DCalc(PID *sptr,int16 Setpoint,int16 Turepoint)
功  能：位置式PID控制
参  数：PID *sptr,int16 Setpoint,int16 Turepoint
返回值：float 
************************************************/
int16 LocP_DCalc(PID*sptr,int16 Setpoint,int16 Turepoint)
{
    int16 iError,dError;
    int16 output;

    float ErrorRate = 0.0;
    iError=Setpoint-Turepoint;  //偏差

    dError=(int16)(iError-(sptr->LastError));     //微分
	sptr->SumError+=iError;            //积分(采样时间很短时，用一阶差分代替一阶微分，用累加代替积分)
    sptr->LastError=iError;
	//积分限幅
	sptr->SumError = sptr->SumError>MAX_SUMERROR?MAX_SUMERROR:sptr->SumError<-MAX_SUMERROR?-MAX_SUMERROR:sptr->SumError;
	// 多加一个积分补偿器，改进算法 
	// 一般情况下，积分补偿器不做任何作用 当车需要快速修正或者超调时才会起作用
	ErrorRate = iError / sptr->LastError;
    ErrorRate = ErrorRate < 0?-ErrorRate:ErrorRate;
    if(ErrorRate > 1)
    {
	    sptr->CompensateError += iError * ErrorRate;
    }
    else
    {
        	sptr->CompensateError = (iError < 10 || sptr->CompensateError < 10)? 0:sptr->CompensateError / 10;
    }
    sptr->CompensateError = sptr->CompensateError>MAX_SUMERROR?MAX_SUMERROR:sptr->CompensateError<-MAX_SUMERROR?-MAX_SUMERROR:sptr->CompensateError;
    output=(int16)(sptr->Kp*iError  //比例项
          +(sptr->Ki*sptr->SumError)//积分项
          +sptr->Kd*dError);        //微分项
          
    output = output > MAX_OUTPUT? MAX_OUTPUT:output < -MAX_OUTPUT?-MAX_OUTPUT:output;
    output = output + sptr->Kc * sptr->CompensateError; // 积分超调补偿器
    return(output);
}
/************************************************
函数名：IncPIDCalc(PID *sptr,int16 Setpoint,int16 Turepoint)
功  能：增量式PID控制
参  数：PID *sptr,int16 Setpoint,int16 Turepoint
返回值：int32 iIncpid
************************************************/
int16 IncPIDCalc(PID *sptr,int16 Setpoint,int16 Turepoint)
{
    int16 iError,iIncpid;
    //当前误差
    iError=Setpoint-Turepoint;      //偏差
    //增量计算
    iIncpid=(int16)(sptr->Kp*(iError-sptr->LastError)
            +sptr->Ki*iError
            +sptr->Kd*(iError-2*sptr->LastError+sptr->LLastError));
    //储存误差，用于下次计算
    sptr->LLastError=sptr->LastError;
    sptr->LastError=iError;
    return(iIncpid);
}
/************************************************
函数名：PlacePID_Control(PID *sptr, int16 Setpoint,int16 Turepiont)
功  能：动态位置式PID控制 (一般用于转向控制)
参  数：PID *sptr,int16 Setpoint,int16 Turepoint
返回值：int32 Actual
************************************************/
int16 PlacePID_Control(PID *sptr, int16 Setpoint,int16 Turepiont)
{
    int16 iError,Actual;
    float KP;  //动态P，注意与Kp区分
    iError = Setpoint - Turepiont;
    KP = 1.0 * (iError*iError)/sptr->Ki+sptr->Kp ;    //动态P的计算
    sptr->SumError+=iError;
    Actual = (int16)(KP * iError + sptr->Kd* ((3.2*iError + 0.8*sptr->LastError) - sptr->LastError));
    sptr->LastError = iError;
    return Actual;
}