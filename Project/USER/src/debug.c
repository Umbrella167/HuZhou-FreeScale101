#include "debug.h"
/*************************ʹ��˵��****************************************
��Э���롰Visual Scope�����Э����ݣ��ù��Ŀ���ֱ����ԭ������λ��Э�鼴��
�״�ʹ��ʱ��
1.����outputdata.c���͡�outputdata.h����ӵ���Ĺ�����
2.�ڡ�outputdata.c���а�����ԭ����Ĵ��ڷ��ͺ���ͷ�ļ�
3.��uart_putchar(databuf[i]);����滻Ϊ��Ĵ����ֽڷ��ͺ�������send_char(databuf[i]);
4.����ĳ�����Ҫ���Ͳ������ݵ�.c�ļ�����Ӱ�����#include "outputdata.h"�����ڱ��ļ��е��ú���OutPut_Data(x,y,z,w);
  �����β�x��y��z��w���Ǵ����ĸ�short int 16λ���ݣ��ֱ��Ӧͨ��1,2,3,4
************************************************************************/
//�˴������Ĵ���ͷ�ļ�����������������������������
#include "zf_uart.h"
//****************************��ֲ**************************//

void Data_Send(UARTN_enum uratn,signed short int *pst)
{
		unsigned char _cnt=0; unsigned char sum = 0;
    unsigned char data_to_send[23] = {0};         //���ͻ���
    unsigned char i;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x02;
    data_to_send[_cnt++]=0;
    data_to_send[_cnt++]=(unsigned char)(pst[0]>>8);  //��8λ
    data_to_send[_cnt++]=(unsigned char)pst[0];  //��8λ
    data_to_send[_cnt++]=(unsigned char)(pst[1]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[1];
    data_to_send[_cnt++]=(unsigned char)(pst[2]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[2];
    data_to_send[_cnt++]=(unsigned char)(pst[3]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[3];
    data_to_send[_cnt++]=(unsigned char)(pst[4]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[4];
    data_to_send[_cnt++]=(unsigned char)(pst[5]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[5];
    data_to_send[_cnt++]=(unsigned char)(pst[6]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[6];
    data_to_send[_cnt++]=(unsigned char)(pst[7]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[7];
    data_to_send[_cnt++]=(unsigned char)(pst[8]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[8];


    data_to_send[3] = _cnt-4;

    sum = 0;
    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];

    data_to_send[_cnt++] = sum;

        for(i=0;i<_cnt;i++)
    uart_putchar(uratn,data_to_send[i]);
}

//===================================================��λ����ص�==========================================================
//=============================================��λ��ʹ�������ƴ�����վ======================================================
/******************************************���ݴ���****************************************************
������void datasend()
������  ��
˵���� ����ͬʱ����6����  icm_gyro_x   icm_acc_x ICM_Real.gyro.y  ICM_Real.acc.z   
�����ı�����icm_acc_x  icm_gyro_y  Angle  adc_date[0] Left_Adc

����ֵ����
���ڣ�
���ߣ� 
*****************************************************************************************************/
void datasend()
{  
   short send_data[6];                      

   send_data[0]= left_speed; ////////ICM_Start.acc.x 
   send_data[1]= 400; //////////////////    MpuStart.gyro.x   Angle  
   send_data[2]= right_speed; //////////
   send_data[3]= 400; //
   send_data[4]= 0;
   send_data[5]= 0;
	 //Data_Send(UART_4,send_data);
   Data_Send(UART_User,send_data);
}

//====================================================��Ļ��ص�=(��ɾ��)=============================================================
//============================================================================================================================
//sprintf(temp," date20=%d",date);
//TFTSPI_P8X8Str(0,19,temp,BLACK,WHITE);break;

//==========================================================���뿪�ؼ��������=========================================================
//====================================================================================================================================

//���뿪�����ź궨��
#define Switch_Pin_1       P75
#define Switch_Pin_2       P76
#define Switch_Pin_3       P33
#define Switch_Pin_4       P33
#define Switch_Pin_5       P33
#define Switch_Pin_6       P33
//���尴������
#define KEY1    P70      
#define KEY2    P71      
#define KEY3    P72        
#define KEY4    P73     
#define KEY5    P33 

//***************�����궨��****(������Щ�������޸ĺ궨��Ϊ��Ӧ��GPIO�⺯������)***********
#define KEY_INT(key_x)           gpio_mode(key_x,GPO_PP)//����ΪGPO_PP:������� 
#define SWITCH_INT(switch_x)     gpio_mode(switch_x,GPO_PP)//����ΪGPO_PP:�������
#define READ_GPIO(Pin_X)         Pin_X
#define TiaoCan_DelayMs(M_S)     delay_ms(M_S)   //��ʱ����

unsigned char TiaoCan=0;////////////////////////���α�־λ
unsigned char TFT_SHOW=0;///////////////////////��Ļ����
unsigned char Switch1=0,Switch2=0,Switch3=0,Switch4=0,Switch5=0,Switch6=0;//���뿪��
char parameter=0;//����ѡ��

//����״̬����
uint8 key1_status = 1,key2_status = 1,key3_status = 1, key4_status = 1,key5_status = 1;
//��һ�ο���״̬����
uint8 key1_last_status, key2_last_status, key3_last_status, key4_last_status,key5_last_status;
//���ر�־λ
uint8 key1_flag=0,key2_flag=0,key3_flag=0, key4_flag=0,key5_flag=0;
/*****************���뿪�ؼ�������ʼ��*****************
������void Switch_Key_init()
���ܣ���ʼ��IO
������  ��
˵���� ��ʼ��IO��   gpio_init(D1, GPI, GPIO_HIGH, GPI_PULL_UP); GPO_PUSH_PULL
����ֵ����*/
void Switch_Key_init()
{

  //���뿪�س�ʼ��  �������޸ģ������޸ģ�
	SWITCH_INT(Switch_Pin_1) ;
	SWITCH_INT(Switch_Pin_2) ;
	//SWITCH_INT(Switch_Pin_3) ;
	//SWITCH_INT(Switch_Pin_4) ;
	//SWITCH_INT(Switch_Pin_5) ;
	//SWITCH_INT(Switch_Pin_6) ;
    
  //������ʼ�� �������޸ģ������޸ģ�
  KEY_INT(KEY1);
	KEY_INT(KEY2);
	KEY_INT(KEY3);
	KEY_INT(KEY4);
	//KEY_INT(KEY5);
}

/*****************���뿪�ز���ѡ��*****************
������void Strategy_Slect()
���ܣ�ͨ�����뿪�ص�������
������  ��
˵����  6λ���뿪�أ���������ӻ��߼��ٿɶ����޸�,�������6��Ҳ��Ҫɾ������ģ��������������Ÿĸ�û�õļ���
        ʹ���㶨��ľͺ��ˣ�����û���õ����������
����ֵ����*/
void Strategy_Slect()
{
  //��ȡ���뿪��״̬
      if(!READ_GPIO(Switch_Pin_1))//��
      {
       Switch1=1;
       //TFT_SHOW = 1;
			 //��ʾ���ԭʼֵ
			 /*ips114_showuint8(0,0,adc_value[0]);
	     ips114_showuint8(0,1,adc_value[1]);
	     ips114_showuint8(0,2,adc_value[2]);
	     ips114_showuint8(0,3,adc_value[3]);*/
      }
      if(!READ_GPIO (Switch_Pin_2))//��
      {
       Switch2=1;
			 //��ʾ��й�һ��ֵ
			 /*ips114_showuint8(0,0,Left_Adc);
	     ips114_showuint8(0,1,Left_Shu_Adc);
	     ips114_showuint8(0,2,Right_Shu_Adc);
	     ips114_showuint8(0,3,Right_Adc);*/
       //LEFT_RIGHT=0;//����������
      }
      if(!READ_GPIO (Switch_Pin_3))
      {
       Switch3=1;
      }
      if(!READ_GPIO (Switch_Pin_4))
      {
       Switch4=1;

      }
      if(!READ_GPIO (Switch_Pin_5))
      {
       Switch5=1;

      }
      if(!READ_GPIO (Switch_Pin_6))
      {
       Switch6=1;
 
      }

    if(Switch1||Switch2||Switch3||Switch4||Switch5||Switch6)//�������뿪��
      {

      }
}

/*****************����ɨ���ȡ*****************
������void  Key_Scan_Deal ()
���ܣ���ȡ������ִ�ж�Ӧ����
������  ��
˵���� �ο�������� ��5λ��������������ӻ��߼��ٿɶ����޸�
      // 1��Ϊ���Ƽ���2��Ϊ�ϼ���3��Ϊ���Ƽ���4��Ϊ�м��̣�5��Ϊ�¼�
     //���γ���û��ʹ�õ��Σ�stc��Ƭ�����س���Ҳ�죬�����վͿ��ԣ����Ҫ�ӵĻ��Լ�����������Լ��ӾͿ���
����ֵ����     */
uint8 gogo=0;
void  Key_Scan_Deal ()
{
    //ʹ�ô˷����ŵ����ڣ�����Ҫʹ��while(1) �ȴ������⴦������Դ�˷�
    //���水��״̬
    key1_last_status = key1_status;
    key2_last_status = key2_status;
    key3_last_status = key3_status;
    key4_last_status = key4_status;
    key5_last_status = key5_status;
    //��ȡ��ǰ����״̬
    key1_status = READ_GPIO(KEY1);
    key2_status = READ_GPIO(KEY2);
    key3_status = READ_GPIO(KEY3);
    key4_status = READ_GPIO(KEY4);
    key5_status = READ_GPIO(KEY5);
    //��⵽��������֮��  ���ſ���λ��־λ
    if(key1_status && !key1_last_status)    key1_flag = 1;
    if(key2_status && !key2_last_status)    key2_flag = 1;
    if(key3_status && !key3_last_status)    key3_flag = 1;
    if(key4_status && !key4_last_status)    key4_flag = 1;
    if(key5_status && !key5_last_status)    key5_flag = 1;
    //��־λ��λ֮�󣬿���ʹ�ñ�־λִ���Լ���Ҫ�����¼�
 
           if(key1_flag&&(gogo==1||gogo==2)) //S1��
           {
               key1_flag = 0;//ʹ�ð���֮��Ӧ�������־λ
               /*����Ϊ�û�����  */
               switch(parameter)
               {
                 //-----------------------�������޸�����--��ע���޸Ķ�Ӧ����ʾ��----------------------------------------------------------------
                 //��һҳ��ʾ�Ķ���
                 case 0:  break;
                 case 1:  break;
                 case 2:  break;
                 case 3:  break;
                 case 4:  break;
                 case 5:  break;

                 /// case 6:  ; break;//������ܼ��κβ����������ˣ���ҳʹ����

                 //�ڶ�ҳ��ʾ�Ķ���
                 case 7:  break;
                 case 8:  break;
                 case 9:  break;
                 case 10: break;
                 case 11: break;
                 case 12: break;
                //--------------------�������޸�����------------------------------------------------------------------
               }
                 /*  ����Ϊ�û�����  */
          }
          if(key2_flag&&(gogo==1||gogo==2))//S2��
          {
             key2_flag = 0;//ʹ�ð���֮��Ӧ�������־λ
             /*  ����Ϊ�û�����  */
             switch(parameter)
             {
                //----------------------�������޸�����--��ע���޸Ķ�Ӧ����ʾ��--------------------------------------------------------------
                //��һҳ��ʾ�Ķ���
                case 0: break;
                case 1: break;
                case 2: break;
                case 3: break;
                case 4: break;
                case 5: break;

                /// case 6:  ; break;//������ܼ��κβ����������ˣ���ҳʹ����

                //�ڶ�ҳ��ʾ�Ķ���
                case 7: break;
                case 8: break;
                case 9: break;
                case 10: break;
                case 11: break;
                case 12: break;
                //--------------------�������޸�����------------------------------------------------------------------
               }
               /*  ����Ϊ�û�����  */  
           }
           if(key3_flag&&(gogo==1||gogo==2))//S3��
           {
               key3_flag = 0;//ʹ�ð���֮��Ӧ�������־λ
               /*  ����Ϊ�û�����  */
                      parameter--;
               /*  ����Ϊ�û�����  */

           }

           if(key4_flag&&(gogo==1||gogo==2))//S4��
           {
               key4_flag = 0;//ʹ�ð���֮��Ӧ�������־λ
               /*  ����Ϊ�û�����  */
                      parameter++;
               /*  ����Ϊ�û�����  */

           }
            if(key5_flag) //S5��
           {
              key5_flag = 0;//ʹ�ð���֮��Ӧ�������־λ
            /*  ����Ϊ�û�����  */
						

						/*  ����Ϊ�û�����  */

           }
    if(gogo==6)//����������ν���������������������������
		{

			 TiaoCan = 1;      //���ν�����־λ

		}
    //*******************************��Ļ��ʾ��һҳ***********************

    if(parameter<6&&gogo>=1)//��ʾ����0��5��ʵ����ʾ1��6
    {
          
    }
    //*******************************��Ļ��ʾ�ڶ�ҳ**************************************************
    if(parameter>6&&parameter<13)//�����кŴ�4��9   һҳ��6������  //��ʾ����7��5��ʵ����ʾ7��12
    {
          
    }
    //*******************************��Ļ��ʾ����ҳ**************************************************
    if(parameter>13&&parameter<20)
    {
                  
    }
    //###########����Ҫ����ҳ������д�Ϳ��Կ�######################����Ͳ�д�� ����
    if(parameter==6||parameter==13||parameter==20)  //��ҳ׼��
     {
                     
     }//����
}

/**********************************************�������εε�******************************************
������void BUZZ_DiDiDi()
���ܣ��������εε�
������  ��
˵����
����ֵ����
***************************************************************************************************/
void BUZZ_DiDiDi(uint16 PinLV)
{
  BUZZ_ON;
  TiaoCan_DelayMs(PinLV);
  BUZZ_OFF;
}
/****************************�������*********************************************
 *  �������ƣ�Test_Motor_Hardware(void)
 *  ����˵�������Ա궨���PWM���Ƶ��
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺
 *  ��    ע������2�����
 ��ע�����ע�⣬һ��Ҫ�Ե�������������
 1.��ʹ�����ñ������ص�ѹ����ر�֤��ص�ѹ��7V���ϣ�������������Ӧ��
 2.�Ӻ�ĸ�嵽��������ź��߼���Դ�ߣ�
 3.�Ӻ������嵽����ĵ��ߣ�
 4.��д�������У�ȷ�����������ת���󣬿����������Դ���أ�
 5.����K0/K1ȷ�����ת���ٶȼ�����
 6.������ַ�ת������K1�����ص���ģʽ������ֱ�ӹر��������Դ��
 ******************************************************************************/
void Test_Motor_Hardware (void)
{
    int16 duty = 2000;
    ips114_clear(YELLOW);  //��ʼ����
	  ips114_showstr(2, 0, "Test_Motor_Hardware:");
    init_PWM(1);
	
    while (1)
    {
        if (!READ_GPIO(KEY1))                       //����KEY1��   ���ֵ�����ת
        {
             go_motor (duty,0);
					   ips114_showstr(0,4,"Left  Front");     //�ַ�����ʾ
        } 
        if (!READ_GPIO(KEY2))                       //����KEY2��  ���ֵ�����ת
        {
           	 go_motor (0,duty);
						 ips114_showstr(0,4,"Right Front");   //�ַ�����ʾ
        }
				if (!READ_GPIO(KEY3))                       //����KEY3����������ͬʱ��ת
        {
             go_motor (-duty,-duty);
					   ips114_showstr(0,4,"All  Black");      //�ַ�����ʾ
           	  	 
        }
				if((READ_GPIO(KEY1))&&(READ_GPIO(KEY2))&&(READ_GPIO(KEY3)))
        go_motor (0,0);
      	TiaoCan_DelayMs(100);  
    }
}

/****************************�������*********************************************
 *  �������ƣ�void Test_Electric_Hardware (void)
 *  ����˵�������Ե�ŵ��Ӳ��
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2021-5-30
 *  ��    ע��
 *************************************************************************/
void Test_Electric_Hardware (void)
{
	char txt[16];
	ips114_clear(YELLOW);  //��ʼ����
	ips114_showstr(2, 0, "Test_Electric_Hardware:");
	ADC_int();
	while(1)
	{
			  //datasend();
		    if (!READ_GPIO(KEY1)) //����KEY1��
        {
					ips114_showstr(2, 1, "Normalize_Deal....");   //�ַ�����ʾ
					ADC_Collect();  //��в�ֵ
					
					sprintf(txt,"adc0= %02f",adc_value[0]);
					ips114_showstr(1, 2, txt); //��ʾ
					sprintf(txt,"adc1= %02f",adc_value[1]);
					ips114_showstr(1, 3, txt); //��ʾ
					sprintf(txt,"adc2= %02f",adc_value[2]);
					ips114_showstr(1, 4, txt); //��ʾ
					sprintf(txt,"adc3= %02f",adc_value[3]);
					ips114_showstr(1, 5, txt); //��ʾ				  	  	 
        }
				if(!READ_GPIO(KEY2)) //����KEY2��
				{
					ips114_showstr(2, 1, "GYH_Normalize_Deal....");   //�ַ�����ʾ
					ADC_Collect();  //��в�ֵ
	        Data_current_analyze();  //���ֵ��һ������
					Current_Dir=Cha_bi_he(Left_Adc,Right_Adc,100);
					
					sprintf(txt,"adc0= %05d",Left_Adc);
					ips114_showstr(1, 2, txt); //��ʾ
					sprintf(txt,"adc1= %05d",Left_Shu_Adc);
					ips114_showstr(1, 3, txt); //��ʾ
					sprintf(txt,"adc2= %05d",Right_Shu_Adc);
					ips114_showstr(1, 4, txt); //��ʾ
					sprintf(txt,"adc3= %05d",Right_Adc);
					ips114_showstr(1, 5, txt); //��ʾ
					sprintf(txt,"Current_Dir= %05d",Current_Dir);
					ips114_showstr(1, 6, txt); //��ʾ
        }
  }	
}

/*
			         	encoder_init();						
	   while(1)
		 {
			 		 speed_measure();
		delay_ms(50);
			 	sprintf(txt,"Left_Speed  = %05d",left_speed);
		ips114_showstr(1, 3, txt); //��ʾ
			sprintf(txt,"Right_Speed = %05d",right_speed);
		ips114_showstr(1, 4, txt); //��ʾ
		  sprintf(txt,"Speed     = %05d",real_speed);
		ips114_showstr(1, 5, txt); //��ʾ
		 }
*/



