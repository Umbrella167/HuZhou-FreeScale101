#include "PID.h"
#define MAX_SUMERROR 2500
#define MAX_OUTPUT 1600
/************************************************
��������IncPIDInit(PID *sptr)
��  �ܣ�PID������ʼ��
��  ����
����ֵ��void
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
��������LocP_DCalc(PID *sptr,int16 Setpoint,int16 Turepoint)
��  �ܣ�λ��ʽPID����
��  ����PID *sptr,int16 Setpoint,int16 Turepoint
����ֵ��float 
************************************************/
int16 LocP_DCalc(PID*sptr,int16 Setpoint,int16 Turepoint)
{
    int16 iError,dError;
    int16 output;

    float ErrorRate = 0.0;
    iError=Setpoint-Turepoint;  //ƫ��

    dError=(int16)(iError-(sptr->LastError));     //΢��
	sptr->SumError+=iError;            //����(����ʱ��ܶ�ʱ����һ�ײ�ִ���һ��΢�֣����ۼӴ������)
    sptr->LastError=iError;
	//�����޷�
	sptr->SumError = sptr->SumError>MAX_SUMERROR?MAX_SUMERROR:sptr->SumError<-MAX_SUMERROR?-MAX_SUMERROR:sptr->SumError;
	// ���һ�����ֲ��������Ľ��㷨 
	// һ������£����ֲ����������κ����� ������Ҫ�����������߳���ʱ�Ż�������
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
    output=(int16)(sptr->Kp*iError  //������
          +(sptr->Ki*sptr->SumError)//������
          +sptr->Kd*dError);        //΢����
          
    output = output > MAX_OUTPUT? MAX_OUTPUT:output < -MAX_OUTPUT?-MAX_OUTPUT:output;
    output = output + sptr->Kc * sptr->CompensateError; // ���ֳ���������
    return(output);
}
/************************************************
��������IncPIDCalc(PID *sptr,int16 Setpoint,int16 Turepoint)
��  �ܣ�����ʽPID����
��  ����PID *sptr,int16 Setpoint,int16 Turepoint
����ֵ��int32 iIncpid
************************************************/
int16 IncPIDCalc(PID *sptr,int16 Setpoint,int16 Turepoint)
{
    int16 iError,iIncpid;
    //��ǰ���
    iError=Setpoint-Turepoint;      //ƫ��
    //��������
    iIncpid=(int16)(sptr->Kp*(iError-sptr->LastError)
            +sptr->Ki*iError
            +sptr->Kd*(iError-2*sptr->LastError+sptr->LLastError));
    //�����������´μ���
    sptr->LLastError=sptr->LastError;
    sptr->LastError=iError;
    return(iIncpid);
}
/************************************************
��������PlacePID_Control(PID *sptr, int16 Setpoint,int16 Turepiont)
��  �ܣ���̬λ��ʽPID���� (һ������ת�����)
��  ����PID *sptr,int16 Setpoint,int16 Turepoint
����ֵ��int32 Actual
************************************************/
int16 PlacePID_Control(PID *sptr, int16 Setpoint,int16 Turepiont)
{
    int16 iError,Actual;
    float KP;  //��̬P��ע����Kp����
    iError = Setpoint - Turepiont;
    KP = 1.0 * (iError*iError)/sptr->Ki+sptr->Kp ;    //��̬P�ļ���
    sptr->SumError+=iError;
    Actual = (int16)(KP * iError + sptr->Kd* ((3.2*iError + 0.8*sptr->LastError) - sptr->LastError));
    sptr->LastError = iError;
    return Actual;
}