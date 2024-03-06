#include "fuse.h"
PID SpeedPID = {0};
PID TurnPID ={0};
PID Turn_NeiPID ={0};
/****************************PID������ʼ��**************************************
������  void PID_int(void)
������  void
˵����  PIDÿ����������ʼ��
����ֵ��void
********************************************************************************/
void PID_int(void)
{
	SpeedPID.Kp=3.8;     //�ٶȻ�PID����
	SpeedPID.Ki=0.8;
	SpeedPID.Kd=0;
	
	TurnPID.Kp=80;       //ת���⻷PID���� 
	TurnPID.Ki=0;
	TurnPID.Kd=5;
	
	Turn_NeiPID.Kp=2.8;  //���ٶ��ڻ�PID����
	Turn_NeiPID.Ki=0;
	Turn_NeiPID.Kd=0.5;
}
static TASK_COMPONENTS TaskComps[] =
{
    {0,  2,  2, Motor_output_control},         //���ٶ��ڻ����ٶȻ�2ms
    {0, 10, 10, Trailing_control},           //ת���⻷10ms
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
* * attention      : ***����main��while(1)***
**************************************************************************************/
void TaskProcess(void)
{
    uint8 i; 
	  for (i=0; i<TASKS_MAX; i++)           // �������ʱ�䴦��
    {
        if (TaskComps[i].Run)           
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
    imu660ra_get_gyro();   //��ȡ�����ǽ��ٶ�ֵ
		speed_measure();       //����������
	  aim_speed = 400;       //Ŀ���ٶ�
	  //Speed_pwm_all = LocP_DCalc(&SpeedPID,aim_speed,real_speed);     //�ٶȻ���λ��ʽ��
	  Speed_pwm_all += IncPIDCalc(&SpeedPID,aim_speed,real_speed);      //�ٶȻ�������ʽ��
	  Steer_pwm=LocP_DCalc(&Turn_NeiPID,imu660ra_gyro_z,ADC_PWM);      //ת���ڻ�PWM	
    Steer_pwm=range_protect(Steer_pwm,-5000,2000);                  //ת���ڻ�PWM�޷�
		All_PWM_left=Speed_pwm_all+Steer_pwm ;                          //��������PWM��� 
		All_PWM_right=Speed_pwm_all -Steer_pwm ;                        //�ҵ������PWM���
    go_motor(All_PWM_left,All_PWM_right);                             //�������
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
	  Annulus_assist(); //������������
		ADC_PWM = LocP_DCalc(&TurnPID,Current_Dir,0);//λ��ʽPD����ת��
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
