#include "headfile.h"
void all_init()
{
	ips114_init();    																				//1.14��Һ������ʼ��
	ADC_int();      																					//ADC�ɼ���ʼ��
	ips114_showstr(0,0,(unsigned char*)"Electromagnetic-Car");//�ַ�����ʾ
	ips114_showstr(0,1,"interlize...");
	delay_ms(500);
	pwm_init(PWMB_CH4_P77, 50, 0);                             //��������ʼ��
	init_PWM(MOTOR_MODE_SELECT);                               //��ʼ��DRV������ʽ��1-DRV/0-BTN��   
	encoder_init();                                            //��������ʼ��
	wireless_uart_init();                                      //����������ʼ��
	BUZZ_DiDiDi(200);                                          //��������һ��
	delay_ms(50);
	gpio_mode(Reed_Switch_Pin,GPO_PP);                         //ͣ��ʶ��ĸɻɹ�IO��ʼ��
  PID_int();                                                 //PID������ʼ��                                        
//	while(imu660ra_init())                                     //�����ǳ�ʼ��
//	{
//		ips114_showstr(0,2,"IMU660RA_int...");                   //��whileд��ֱ����ʼ���ɹ��Ż��˳�ѭ��
//		delay_ms(500);
//	}
	ips114_showstr(0,3,"IMU660RA_int...");
//	while(dl1a_init())                                         //����ģ���ʼ��
//	{
//		ips114_showstr(0,4,"dlla_int...");                       //��whileд��ֱ����ʼ���ɹ��Ż��˳�ѭ��
//		delay_ms(500);
//	}	
	ips114_showstr(0,5,"dlla_intok...");
	pit_timer_ms(TIM_1, 1);                                    //��ʼ����ʱ��1��Ϊ�����жϴ�������1MS��һ���ж�
	ips114_showstr(0,6,"intall_intok...");
	delay_ms(500);
  ips114_clear(WHITE);   	//����
}
void main()
{
	
	WTST = 0;       //���ó������ȴ���������ֵΪ0�ɽ���Ƭ��ִ�г�����ٶ�����Ϊ���
	DisableGlobalIRQ();     																	//�ر����ж�
	board_init();		       																		//��ʼ���Ĵ���,��ɾ���˾���롣
	delay_ms(800);          																	//�����΢��ʱһ��
  all_init();
	EnableGlobalIRQ();             														//��ʼ����ϣ��������ж�
	/****����Ĳ��Ժ���ֻ�ǲ����ã����Խ�����ע�͹رգ�һ��ֻ����һ�����Ժ�������******/
	//Test_Motor_Hardware();//���Ե��ʹ��
	//Test_Electric_Hardware();//���Ե�ŵ�вɼ�
	/*************************************************************************************/
	while(1)
	{	
   		TaskProcess();        //�ж���������Ųɼ�����������
			ADC_Collect();  //��в�ֵ
//		TaskProcess();        //�ж���������Ųɼ�����������
//    ips114_showint16(0,0,left_speed);
//    ips114_showint16(0,1,right_speed);
//    ips114_showint16(0,2,real_speed);
//	  ips114_showuint8(122,0,adc_value[0]);  
//		
		//speed_measure();       //����������
		//Current_Dir=Direction_error(); //�������ƫ��
		//ips114_showfloat(122,1,Current_Dir);
//		go_motor(800,800);							
		//Trailing_control();
		//Speed_pwm_all += IncPIDCalc(&SpeedPID,aim_speed,real_speed);//D���ٶȻ�������ʽ��                    
//   �����ʾ
	 ips114_showuint8(100,0,Left_Adc);  
	 ips114_showuint8(100,1,Left_Shu_Adc);
	 ips114_showuint8(100,2,Right_Shu_Adc);
	 ips114_showuint8(100,3,Right_Adc); 
	 ips114_showuint16(0,0,left_speed);
	 ips114_showuint16(0,1,right_speed);
	 //printf("%d,%d,%d,%d,%d,%d\n",road_type.straightbend,road_type.bend,Left_Adc,Left_Shu_Adc,Right_Shu_Adc,Right_Adc);
	 printf("%d,%d,%d,%d\n",Left_Adc,Left_Shu_Adc,Right_Shu_Adc,Right_Adc);
	 //printf("type=%d\n",road_type.bend);
	
	// ips114_showfloat(122,6,Current_Dir,2,1);//��ʾ������   ������ʾ2λ   С����ʾ1λ
  }	
}
