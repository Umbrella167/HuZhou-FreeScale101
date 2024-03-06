#include "PID.h"
/************************************************
��������IncPIDInit(PID *sptr)
��  �ܣ�PID������ʼ��
��  ����
����ֵ��void
************************************************/
void IncPIDInit(PID *sptr)
{
    sptr->SumError=0;
    sptr->LastError=0;
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
    iError=Setpoint-Turepoint;  //ƫ��
    sptr->SumError+=iError;            //����(����ʱ��ܶ�ʱ����һ�ײ�ִ���һ��΢�֣����ۼӴ������)
    dError=(int16)(iError-(sptr->LastError));     //΢��
    sptr->LastError=iError;
    if(sptr->SumError>2500) sptr->SumError=2500;   //�����޷�
    if(sptr->SumError<-2500) sptr->SumError=-2500;
    output=(int16)(sptr->Kp*iError  //������
          +(sptr->Ki*sptr->SumError)//������
          +sptr->Kd*dError);        //΢����
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
    Actual = (int16)(KP * iError + sptr->Kd* ((0.8*iError + 0.2*sptr->LastError) - sptr->LastError));
    sptr->LastError = iError;
    return Actual;
}