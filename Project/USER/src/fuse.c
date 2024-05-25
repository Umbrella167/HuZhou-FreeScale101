#include "fuse.h"

PID SpeedPID = {0};
PID L_SpeedPID ={0};
PID R_SpeedPID ={0};
PID TurnPID ={0};
PID Turn_NeiPID ={0};
PID TurnPID_wan ={0};
PID Turn_NeiPID_wan ={0};
PID TurnPID_RTwan ={0};
PID Turn_NeiPID_RTwan ={0};

/****************************PID������ʼ��**************************************
������  void PID_int(void)
������  void
˵����  PIDÿ����������ʼ��
����ֵ��void
********************************************************************************/
void PID_int(void)
{
	L_SpeedPID.Kp=0.5;     //�ٶȻ�PID������D���ã��ٶȻ�2ms��
	L_SpeedPID.Ki=0.05;
	L_SpeedPID.Kd=0;
	
	R_SpeedPID.Kp=0.1;       //�ٶȻ�PID������D���ã��ٶȻ�2ms�� 
	R_SpeedPID.Ki=0.05;
	R_SpeedPID.Kd=0;
	
	TurnPID.Kp=20.0;       //ת���⻷PID���� 
	TurnPID.Ki=1;
	TurnPID.Kd=20;
	TurnPID.Kc=1;
	
//	Turn_NeiPID.Kp=0.125;  //���ٶ��ڻ�PID����
//	Turn_NeiPID.Ki=0;
//	Turn_NeiPID.Kd=0.5;
//	
//	TurnPID_wan.Kp=100.0;       //���ת���⻷PID���� 
//	TurnPID_wan.Ki=0;
//	TurnPID_wan.Kd=8;
//	
//	Turn_NeiPID_wan.Kp=0.13;  //������ٶ��ڻ�PID����
//	Turn_NeiPID_wan.Ki=0.3;
//	Turn_NeiPID_wan.Kd=0.5;
//	
//	TurnPID_RTwan.Kp=80.0;       //ֱ�����ת���⻷PID���� 
//	TurnPID_RTwan.Ki=0;
//	TurnPID_RTwan.Kd=8;
//	
//	Turn_NeiPID_RTwan.Kp=0.4;  //ֱ��������ٶ��ڻ�PID����
//	Turn_NeiPID_RTwan.Ki=0;
//	Turn_NeiPID_RTwan.Kd=0.5;
}
static TASK_COMPONENTS TaskComps[] =
{
    {0,  1,  1, Motor_output_control},         //���ٶ��ڻ���D���ٶȻ�2ms
    {0,  5,  5, Trailing_control},           //ת���⻷5ms
};
/**************************************************************************************
* FunctionName   : TaskRemarks()
* Description    : �����־����
* EntryParameter : None
* ReturnValue    : None
* attention      : ***�ڶ�ʱ���ж��е��ô˺�������***
**************************************************************************************/
void TaskRemarks(void)
{
    uint8 i;
    for (i=0; i<TASKS_MAX; i++)          // �������ʱ�䴦��
    {
        if (TaskComps[i].Timer)          // ʱ�䲻Ϊ0
        {
           TaskComps[i].Timer--;         // ��ȥһ������
           if (TaskComps[i].Timer == 0)       // ʱ�������
           {
             TaskComps[i].Timer = TaskComps[i].ItvTime; // �ָ���ʱ��ֵ��������һ��
             TaskComps[i].Run = 1;           // �����������
           }
        }
   }
}

/**************************************************************************************
* FunctionName   : TaskProcess()
* Description    : ������|�ж�ʲôʱ���ִ����һ������
* EntryParameter : None
* ReturnValue    : None
* * attention      : ***����mian��while(1)����***
**************************************************************************************/
void TaskProcess(void)
{
    uint8 i; 
	  for (i=0; i<TASKS_MAX; i++)           // �������ʱ�䴦��
    {
        if (TaskComps[i].Run)           // ʱ�䲻Ϊ0
        {
            TaskComps[i].TaskHook();       // ��������
            TaskComps[i].Run = 0;          // ��־��0
        }
    }
}

/****************************���ٶ��ڻ�**************************************
������  void Motor_output_control()
������  void
˵����  ���ٶ��ڻ���D���ٶȻ�
����ֵ��void
***************************************************************************************/
void Motor_output_control()
{
			//imu660ra_get_gyro();   //��ȡ�����ǽ��ٶ�ֵ
			speed_measure();       //����������
			
			//Speed_pwm_all = LocP_DCalc(&SpeedPID,aim_speed,real_speed); //D���ٶȻ���λ��ʽ��

			aim_speed = 800;       //Ŀ���ٶ�
			//Speed_pwm_all += IncPIDCalc(&SpeedPID,aim_speed,real_speed);//D���ٶȻ�������ʽ��
	
			//Steer_pwm=LocP_DCalc(&TurnPID,imu660ra_gyro_z,ADC_PWM);   //ת���ڻ�PWM	
			//Steer_pwm=range_protect(Steer_pwm,-5000,5000);   							//ת���ڻ�PWM�޷�

			All_PWM_left = aim_speed + ADC_PWM;                         //��������PWM��� 
			All_PWM_right= aim_speed - 0.6 * ADC_PWM;                   //�ҵ������PWM���
	
		
//			All_PWM_left = IncPIDCalc(&L_SpeedPID,All_PWM_left,left_speed*2);//D���ٶȻ�������ʽ��
//			All_PWM_right = IncPIDCalc(&R_SpeedPID,All_PWM_right,right_speed*2);//D���ٶȻ�������ʽ��
	
	
	
			go_motor(All_PWM_left,All_PWM_right);											 //�������
		
		
		  ips114_showint16(122,6,All_PWM_left);
			ips114_showint16(122,7,All_PWM_right);
			ips114_showint16(122,8,Steer_pwm);
//		printf("left_pwm=%d \n",All_PWM_left);
//		printf("right_pwm=%d \n",All_PWM_right);
			     
}
/****************************ת�򻷣�ת���⻷��**************************************
������  void Trailing_control()
������  void
˵����  ת�򻷣�ת���⻷��
����ֵ��void
***************************************************************************************/
void Trailing_control()
{
	  Get_deviation();  //��Ųɼ�����ȡ����ƫ��
	//  Annulus_assist(); //������������

		ADC_PWM = LocP_DCalc(&TurnPID,0,Current_Dir);//λ��ʽPD����ת��

}

/***************************************************************************************
��������int16 range_protect(int16 duty, int16 min, int16 max)
��  �ܣ��޷����� 
��  ����
����ֵ��duty
**************************************************************************************/
int16 range_protect(int16 duty, int16 min, int16 max)//�޷�����
{
  if (duty >= max)
  {
    return max;
  }
  if (duty <= min)
  {
    return min;
  }
  else
  {
    return duty;
  }
}
