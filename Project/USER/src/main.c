#include "headfile.h"
void all_init()
{
	ips114_init();    																				//1.14寸液晶屏初始化
	ADC_int();      																					//ADC采集初始化
	ips114_showstr(0,0,(unsigned char*)"Electromagnetic-Car");//字符串显示
	ips114_showstr(0,1,"interlize...");
	delay_ms(500);
	pwm_init(PWMB_CH4_P77, 50, 0);                             //蜂鸣器初始化
	init_PWM(MOTOR_MODE_SELECT);                               //初始化DRV驱动方式（1-DRV/0-BTN）
	encoder_init();                                            //编码器初始化
	wireless_uart_init();                                      //无线蓝牙初始化
	BUZZ_DiDiDi(200);                                          //蜂鸣器滴一声
	delay_ms(50);
	gpio_mode(Reed_Switch_Pin,GPO_PP);                         //停车识别的干簧管IO初始化
  PID_int();                                                 //PID参数初始化                                        
	while(imu660ra_init())                                     //陀螺仪初始化
	{
		ips114_showstr(0,2,"IMU660RA_int...");                   //用while写法直到初始化成功才会退出循环
		delay_ms(500);
	}
	ips114_showstr(0,3,"ICM20602_intok...");
	while(dl1a_init())                                         //避障模块初始化
	{
		ips114_showstr(0,4,"dlla_int...");                       //用while写法直到初始化成功才会退出循环
		delay_ms(500);
	}	
	ips114_showstr(0,5,"dlla_intok...");
	pit_timer_ms(TIM_1, 1);                                    //初始化定时器1作为周期中断触发器，1MS进一次中断
	ips114_showstr(0,6,"intall_intok...");
	delay_ms(500);
  ips114_clear(WHITE);                                       //清屏
}
void main()
{
	WTST = 0;       //设置程序代码等待参数，赋值为0可将单片机执行程序的速度设置为最快
	DisableGlobalIRQ();     																	//关闭总中断
	board_init();		       																		//初始化寄存器,勿删除此句代码。
	delay_ms(800);          																	//软件稍微延时一下
  all_init();
	EnableGlobalIRQ();                                         //初始化完毕，开启总中断
	/****下面的测试函数只是测试用，测试结束请注释关闭，一次只允许开一个测试函数！！******/
	//Test_Motor_Hardware();//调试电机使用
	//Test_Electric_Hardware();//测试电磁电感采集
	/*************************************************************************************/
  while(1)
	{	
	 //Strategy_Slect();     //拨码开关策略选择
	 //Handle_Barn_Out(1);     //执行出库函数后flag_start标志位置一才会进中断
	 flag_start=1;         //平时调车直接给1，不用等出库完毕
   if(flag_start)
	 {
		 TaskProcess();        //中断任务处理，电磁采集，电机输出等
	  //Reed();               //干簧管停车检测
    //datasend();
    //ips114_showint16(0,0,left_speed);
    //ips114_showint16(0,1,right_speed);
    //ips114_showint16(0,2,real_speed);
   }
	 //In_park(1);             //停车入库处理
	 //电感显示
	 /*ips114_showuint8(0,0,Left_Adc);  
	 ips114_showuint8(0,1,Left_Shu_Adc);
	 ips114_showuint8(0,2,Right_Shu_Adc);
	 ips114_showuint8(0,3,Right_Adc);  
	 ips114_showfloat(0,5,Current_Dir,2,1);//显示浮点数   整数显示2位   小数显示1位*/
  }
	
	
	
}
