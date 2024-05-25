#include "park.h"

int8   flag_start           = 0;    //������ɱ�־λ
int8   flag_open_reed       = 0;    //�򿪸ɻɹ�
int8   reed_state           = 0;    //�ɻɹ�״̬
int8   flag_end             = 0;    //��ʼͣ����־λ
uint16 T_outku              = 0;    //����ֱ�ߺʹ�Ƕ�ʱ

/*****************************************���⺯��***************************************
������  void Handle_Barn_Out(uint8 type) 
������  type-----1Ϊ����⣬2Ϊ�ҳ���
˵����  ���⺯��

*ע�⣺���ô˺�����ִ�г��������ֱ�ߵ�ʱ��ʹ��ʱ�估ռ�ձȰ����Լ��޸ĵ���
����ֵ����   
******************************************************************************************/
void Handle_Barn_Out(uint8 type)
{//1Ϊ����⣬2Ϊ�ҳ���
    if(type ==1)
    {
		 if(!flag_start)
		 {
			 //pwm_duty(PWMB_CH1_P74,STEER_MID);              //��C���ã�
			 go_motor(2000, 2000);
			 if (T_outku >= T_OUT_PARK1) 
			 {
				 //pwm_duty(PWMB_CH1_P74,STEER_MID+STEER_LIM);	//��C���ã�
				 go_motor(0, 2500);
			 }
			 if(T_outku > T_OUT_PARK2)
			 {
				  flag_start = 1;
				  //T_outku=0;
			 }		
		  }	
	  }
		if(type ==2)
    {
			if(!flag_start)
			{
				//pwm_duty(PWMB_CH1_P74,STEER_MID);            //��C���ã�
			 go_motor(2000, 2000);
			 if (T_outku >= T_OUT_PARK1) 
			 {
				 //pwm_duty(PWMB_CH1_P74,STEER_MID+STEER_LIM); //��C���ã�
				 go_motor(2500, 0);
			 }
			 if(T_outku > T_OUT_PARK2)
			 {
				  flag_start = 1;
				  //T_outku=0;
			 }															
			}
    }
}
/*****************************************�ɻɹܼ��ͣ��***************************************
������  void Reed(void) 
������  void
˵����  �ɻɹܼ��ͣ��

*ע�⣺ �ɻɹ�ʹ�÷����ͺͰ������ƣ�ͨ����ȡIO�ڵ�ƽ����
����ֵ��void  
*********************************************************************************************/
void Reed(void)
{
	if(flag_start)//��ʼʱ�������ɻɹܼ�⣬��ֹ����ʱ���
	{
		//�߹�һ�ξ�������ɻɹܼ��
		if(Open_pack_time > START_T)
		{
			flag_open_reed = 1;
			Open_pack_time=0;
		}
	}
	if(flag_open_reed)             //�ɻɹܼ���־λ������ſ�ʼ���
	{
		reed_state = Reed_Switch_Pin;//�ɻɹ�״̬
		if(reed_state==0)
		{
			flag_end = 1;              //ʶ��ͣ����־λ����
		}
	 }
}
/*****************************************��⺯��***************************************
������  void Reed(void) 
������  void
˵����  ��⺯��

*ע�⣺ Ŀǰֻ�Ǽ�⵽�����߽���ͣ����������û��д������⣬�������ϵ��Ը���
����ֵ��void  
*********************************************************************************************/
void In_park(uint8 type)
{
	if(type ==1)
  {
		if(flag_end)
		{
			 aim_speed = 0;
			 go_motor(0,0);
		}
 }
	if(type ==2)
  {
		if(flag_end==1)
		{
			 aim_speed = 0;
			 go_motor(0,0);
		}
 }
}