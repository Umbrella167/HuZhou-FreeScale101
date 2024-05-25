#ifndef __PID_H__
#define __PID_H__

#include "headfile.h"

typedef struct 
{
    float SumError;  //����ۻ�
	float CompensateError; //�������
    float Kp;      //����ϵ��
    float Ki;      //����ϵ��
    float Kd;      //΢��ϵ��
	float Kc;	   // ���ֲ���ϵ��
    float LastError;  //��һ�����
    float LLastError; //���ϴ����
}PID;

//�ṹ���������
extern PID TurnPID;
extern PID SpeedPID;
extern PID L_SpeedPID;
extern PID R_SpeedPID;
extern PID Turn_NeiPID;
extern PID TurnPID_wan;			//����ֶ�PID
extern PID Turn_NeiPID_wan; //����ֶ�PID
extern PID TurnPID_RTwan;			//ֱ������ֶ�PID
extern PID Turn_NeiPID_RTwan; //ֱ������ֶ�PID


//��������
void IncPIDInit(PID *sptr);
int16 LocP_DCalc(PID*sptr,int16 Setpoint,int16 Turepoint);
int16 IncPIDCalc(PID *sptr,int16 Setpoint,int16 Turepoint);//����ʽPID����
int16 PlacePID_Control(PID *sptr, int16 Setpoint,int16 Turepiont);//��̬λ��ʽPID����

#endif
