#include "ADC.h"
#include "math.h"

uint8 adc_value[4];                 //�����вɼ�ֵԭʼֵ    4����� 
int16 AD_V[4];                      //�����вɼ�ֵ��һ��ֵ�м���� ��������ģ�����ɾ����
uint8 adc_max[4]={237,220,240,235}; //��в�ֵ���ֵ ��Ҫ�Լ��ɼ�
uint8 adc_min[4]={1,1,17,1};        //��в�ֵ��Сֵ
uint8 Left_Adc,Right_Adc,Left_Shu_Adc,Right_Shu_Adc;//���ֵ
int8 NM=4;                          //��и���

//��������
uint16 annulus_s     = 0;           //�������־���
uint16 annulus_z     = 0;           //�����ڻ��ִ��
uint16 annulus_t = 0;

struct ROAD_TYPE road_type = {0};
int16 obstacle_Current_Dir[]={
	                            30,31,32,33,34,35,36,37,38,39,
	                            40,41,42,43,44,45,46,47,48,49,
	                            -69,-68,-67,-66,-65,-64,-63,-62,-61,-60,
	                            -59,-58,-57,-56,-55,-54,-53,-52,-51,-50,
	                            -49,-48,-47,-46,-45,-44,-43,-42,-41,-40,
                              -39,-38,-37,-36,-35,-34,-33,-32,-31,-30,
                             };
/***��ǰλ��*************/
float Current_Dir = 0;

int16 ADC_PWM=0;
uint8 flag_obstacle=0;
uint16 obstacle_time=0;
uint8 temp=0;				 
/***************************��вɼ�ͨ����ʼ��****************************
������  void ADC_int(void)  
���ܣ�  ��в�ֵ���г�ʼ��
������  void
˵����  ��вɼ���ʼ��
����ֵ����
************************************************************************/
void ADC_int(void)
{
	adc_init(Left_ADC_Pin,ADC_SYSclk_DIV_2);//��ʼ��P0.0ΪADC����
  adc_init(LeftXie_ADC_Pin,ADC_SYSclk_DIV_2);//��ʼ��P0.1ΪADC����
  adc_init(RightXie_ADC_Pin,ADC_SYSclk_DIV_2);//��ʼ��P0.5ΪADC����
  adc_init(Right_ADC_Pin,ADC_SYSclk_DIV_2);//��ʼ��P0.6ΪADC���� 
}

/***************************��ֵ�˲�����*********************************
������uint16 adc_mid(ADCN_enum adcn,ADCCH_enum ch)  
���ܣ� 3�ε�в�ֵ������ֵ�˲�
������ adcn        ѡ��ADCͨ��       resolution      �ֱ���
˵���� 8λADC�����0~255��2��8�η�����5v��ѹƽ���ֳ�255�ݣ��ֱ���Ϊ5/255=0.196
����ֵ��k(uint8)�м��Ǹ�ֵ
************************************************************************/
uint16 adc_mid(ADCN_enum adcn,ADCRES_enum ch)
{
	uint16 i,j,k,tmp;
	i=adc_once(adcn,ch);
	j=adc_once(adcn,ch);
	k=adc_once(adcn,ch);
	if(i>j)
	{
		tmp=i,i=j,j=tmp;
	}
	if(k>j)
	{
		tmp=j;
	}
	else if(k>i)
	{
		tmp=k;
	}
	else
	{
		tmp=i;
	}
	return(tmp);
}

/***************************��ֵ�˲�����****************************
������  uint16 adc_ave(ADCN_enum adcn,ADCCH_enum ch,uint8 N) 
���ܣ�  ��ֵ�˲����5�����ֵ��ƽ��ֵ
������  adcn        ѡ��ADCͨ��         
˵����  �ú���������ֵ�˲������������ֵ����λ��
����ֵ��tmp
ʾ����  adc_ave(ADC_P10, ADC_8BIT)-->ADCͨ��ΪP-10���ֱ���Ϊ8bit 
*******************************************************************/
uint16 adc_ave(ADCN_enum adcn,ADCRES_enum ch,uint8 N)
{
	uint16 tmp=0;
	uint8 i;
	for(i=0;i<N;i++)
	{
	  tmp+=adc_mid(adcn,ch);
	}
	tmp=tmp/N;
	return(tmp);
}
/***************************��в�ֵ************************************
������  void ADC_Collect()   
���ܣ�  ��в�ֵ
������  void
˵����  8λADC�����0~255��2��8�η�����5v��ѹƽ���ֳ�255�ݣ��ֱ���Ϊ5/255=0.196
����ֵ��void
***********************************************************************/
void ADC_Collect()
{
	adc_value[0]=adc_ave(Left_ADC_Pin,ADC_8BIT,5);     //�����
	adc_value[1]=adc_ave(LeftXie_ADC_Pin,ADC_8BIT,5);  //�������
	adc_value[2]=adc_ave(RightXie_ADC_Pin,ADC_8BIT,5); //�������
  adc_value[3]=adc_ave(Right_ADC_Pin,ADC_8BIT,5);    //�Һ���
}
/*********************************��в�ֵ********************************
������  void Data_current_analyze()   
���ܣ�  ��в�ֵԭʼֵ��һ����0~100��
������  void
˵����  ��һ������
����ֵ��void       
*************************************************************************/
void Data_current_analyze()
{
	uint8 i;
  for(i=0;i < NM; i++)              
  {
   AD_V[i] = ((adc_value[i]-adc_min[i])*100)/adc_max[i];         
   if( AD_V[i]<=0)
   {
      AD_V[i]=0;
   }
   else if( AD_V[i]>=100)
   {
      AD_V[i]=100;
   }
  }
  Left_Adc = AD_V[0];       //��������ֵ
  Left_Shu_Adc = AD_V[1];   //�����������ֵ
  Right_Shu_Adc = AD_V[2];  //�����������ֵ
  Right_Adc = AD_V[3];	    //�ҵ������ֵ	
}
/*********************************��Ⱥͺ���**********************************
������  float Cha_bi_he(int16 data1, int16 data2,int16 x)
���ܣ�  ��Ⱥ�������ƫ��
������  int16 data1, int16 data2,int16 x
˵����  ��Ⱥ�������ƫ��
����ֵ��result         
****************************************************************************/
float Cha_bi_he(int16 data1, int16 data2,int16 x)
{
    int16 cha;
    int16 he;
    float result;

    cha = data1-data2;
    he = data1+data2+1;
    result = (cha*x)/(1.0*he);

    return result;
}
/*****************************************���籣������*************************************
������  void Out_protect() 
������  ��
˵����  ��ֹ�����������ײ������,�����������ж�ʧ�ܣ����ͣת���Ż������ж�ʹ�ܼ�����
*ע�⣺������ƽʱ����ʱ���Դ򿪣����˱��ϴ������Ҫ�رմ˺�������Ȼ�п����޷�ʵ�ֱ��Ϲ��ܣ�����
����ֵ����  
******************************************************************************************/
void Out_protect(void)
{
	if(Left_Adc<OUTSIDE&&Right_Adc<OUTSIDE)
	{
		DisableGlobalIRQ();//�ر����ж�
		go_motor(0,0);
	}
	else
	{
		EnableGlobalIRQ();
	}
}
/*****************************************�ж���������*************************************
������  void Road_type_judge(void)
������  ��
˵����  ���������ж�--����--���--ֱ��--
����ֵ����  
******************************************************************************************/
void Road_type_judge(void)
{	 
	  //�����ж�
	  if((Left_Adc+Right_Adc)>IN_ANNULUS_H_LIMIT)
	  {
			road_type.annulus        = 1;
			road_type.straight       = 0;
			road_type.bend           = 0;  
		}
		//����ж�
	  else if((Left_Adc > 85 && Right_Shu_Adc < 50 && Left_Shu_Adc>40)||(Left_Adc <50 && Right_Adc > 85 && Right_Shu_Adc>40))
		{     
			road_type.annulus        = 0;
			road_type.straight       = 0;
			road_type.bend           = 1;
		}
		//ֱ���жϣ��������;���������Ϊֱ��
		else
		{   
			road_type.annulus        = 0;
		  road_type.straight       = 1;
		  road_type.bend           = 0;	  
		}
}
/*****************************************��������***************************************
������  void Annulus_handle(void)
������  ��
˵����  ����������
*ע�⣺�������������������
����ֵ����  
******************************************************************************************/
void Annulus_handle(void)
{
	  //���ж�
		if(annulus_s > DISTANCE_ANNULUS_S&&road_type.annulus==1&&road_type.in_annulus_left==0&&(Left_Shu_Adc>30))	 
		{		  
			road_type.in_annulus_left = 1;
			BUZZ_ON;
			P52                      = 0;
		}
		//�һ��ж�
	  else if(annulus_s > DISTANCE_ANNULUS_S&&road_type.annulus==1&&road_type.in_annulus_right==0&&(Right_Shu_Adc>30))
		{
			road_type.in_annulus_right = 1;
			BUZZ_ON;
			P52                      = 0;
		}
		//�󻷴���
		if(road_type.in_annulus_left == 1)
		{
			if(annulus_z > DISTANCE_ANNULUS_Z&&road_type.in_annulus_left==1&&road_type.on_annulus_left==0)
		  {
			  road_type.on_annulus_left = 1;
				BUZZ_ON;
			  P52                      = 1;
		  }
		  if(road_type.on_annulus_left==1&&road_type.out_annulus==0&& Left_Adc+Right_Adc>OUT_ANNULUS_S_LIMIT)
		  {			
				road_type.out_annulus = 1;
        BUZZ_ON;				
		  }
		}
		//�һ�����
	  else if(road_type.in_annulus_right == 1)
		{
			if(annulus_z > DISTANCE_ANNULUS_Z&&road_type.in_annulus_right==1&&road_type.on_annulus_right==0)
		  {
			  road_type.on_annulus_right = 1;
			  P52                      = 1;
				BUZZ_ON;
		  }
		  if(road_type.on_annulus_right==1&&road_type.out_annulus==0&& Left_Adc+Right_Adc>OUT_ANNULUS_S_LIMIT)  
		  {			
				road_type.out_annulus = 1;		
				BUZZ_ON;
		  }
		}
		//��������
		if(annulus_t>DISTANCE_ANNULUS_T)
		{				
				road_type.annulus          = 0;
				road_type.in_annulus_left  = 0;
			  road_type.in_annulus_right = 0;
				road_type.on_annulus_left  = 0;
			  road_type.on_annulus_right = 0;
				road_type.out_annulus      = 0;
				annulus_s             		 = 0;
			  annulus_z                  = 0;
			  annulus_t                  = 0;
			  BUZZ_OFF;	
		}
}
/*************************************������������*************************************
������  void Annulus_assist(void)
������  ��
˵����  ���������������֣��������֣��������ֵ�
*ע�⣺ ����ֵ��������ʱ��Ĳ�ͬ���ı䣬��Ҫ�Լ������Ƴ�ȥ��������Ļ��ʾ������¼ȥ�޸�
����ֵ����  
******************************************************************************************/
void Annulus_assist(void)
{
	 if(road_type.annulus==1&&road_type.in_annulus_left==0&&road_type.in_annulus_right==0)
   {
        annulus_s += fabs(real_speed*0.1);  
   }
	 if((road_type.in_annulus_left==1 ||road_type.in_annulus_right==1) && road_type.on_annulus_left==0)
   {
        annulus_z += fabs(icm20602_gyro_z*0.01);
   }
	 if((road_type.on_annulus_left==1 ||road_type.on_annulus_right==1) && road_type.out_annulus==1)
   {
        annulus_t=annulus_t+10;
   }
}
/*************************************���ϼ�⺯��*************************************
������  void obstacle_avoidance(void)
������  ��
˵����  TFO����ģ���⣬ʹ�����ģ��IICͨ�ţ��������κ����Ŷ�����ʹ�ã�����Ҫע�ⲻ������
        ���á�
*ע�⣺ TOFģ�����ϰ���ԽԶ��ֵԽ��Խ����ֵԽС
����ֵ����  
******************************************************************************************/
void obstacle_avoidance(void)
{
	dl1a_get_distance();                                       //�������
	if(dl1a_finsh_flag==1&&dl1a_distance_mm<SET_DLLA_DISTANCE) //��������С���趨ֵ��־λ����
	{
		dl1a_finsh_flag=0;
		flag_obstacle=1;
	}
}
/*************************������������ѡ��ͬ�ķ���ƫ����㷽��*************************
������  int16 Direction_error(void)
���ܣ�  ������������ѡ��ͬ�ķ���ƫ��
������  ��
˵����  ������������ѡ��ͬ�ķ���ƫ��--ֱ��--���--���������л���ͬ��Ⱥ�
����ֵ��error--��������ƫ��
****************************************************************************************/
int16 Direction_error(void)
{
    int16 error = 0;
    //ֱ������ƫ�����
    if(road_type.straight==1)
    {
			  error = Cha_bi_he(Left_Adc,Right_Adc,100);  
    }
    //�������ƫ�����
    else if(road_type.bend)
    {
			  error = Cha_bi_he(Left_Adc+Left_Shu_Adc,Right_Adc+Right_Shu_Adc,100);  
    }
    //��������ƫ�����
    else if(road_type.annulus==1)
    {
			  //׼���뻷������ƫ�����
        if(road_type.in_annulus_left==0 && road_type.in_annulus_right==0 && road_type.on_annulus_left==0 && road_type.on_annulus_right==0 && road_type.out_annulus==0)
				{
            error = Cha_bi_he(Left_Adc, Right_Adc, 10);
				}
        //���󻷵�����ƫ�����
        if(road_type.in_annulus_left ==1 && road_type.on_annulus_left==0 && road_type.out_annulus==0)
				{
				    error = Cha_bi_he(Left_Shu_Adc, Right_Shu_Adc,60);
				}
				//���һ�������ƫ�����
			  if(road_type.in_annulus_right ==1 && road_type.on_annulus_right==0 && road_type.out_annulus==0)
				{
				    error = Cha_bi_he(Left_Shu_Adc, Right_Shu_Adc,90);			
				}
        //����������ƫ�����
        //if((road_type.in_annulus_left||road_type.in_annulus_right) && (road_type.on_annulus_left||road_type.on_annulus_right)&&road_type.out_annulus)
				    //error = Cha_bi_he(Left_Adc,Right_Adc,30);
    }
		//��������
	  if(flag_obstacle==1)
		{
				 BUZZ_ON;
				 aim_speed=100;
				 error = obstacle_Current_Dir[temp];
				 temp++;
				 if(temp==40)
				 {
					 temp=0;
					 flag_obstacle=0;
					 obstacle_time=0;
				   aim_speed=400;
				   BUZZ_OFF;
				 }
		}
    return error;
}

/**********************************��������ܴ���***************************************
������  void Get_deviation(void)
���ܣ�  ��������ܴ���
������  ��
˵����  ���жϵ��ô˺�������
****************************************************************************************/
void Get_deviation(void)
{
	ADC_Collect();           //���ԭʼֵ��ֵ
	Data_current_analyze();  //���ֵ��һ������
	Road_type_judge();       //���������ж�---ֱ��---���---����
	Annulus_handle();        //��������
	obstacle_avoidance();    //�ϰ�����
	Current_Dir=Direction_error(); //�������ƫ��
	//Out_protect();         //���籣��
}