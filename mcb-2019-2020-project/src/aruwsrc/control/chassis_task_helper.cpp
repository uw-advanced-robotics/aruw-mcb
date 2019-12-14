#include <math.h>
#include "chassis_task_helper.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/algorithms/kalman.h"

using namespace aruwlib::algorithms;

#define iTermChassis_Max             3000     //΢���޷�

#define		Omni_Speed_Max            9000//7600     //����ˮƽ�ƶ��ٶ��޷�,��ֹ����ģʽ���ٶȳ������ֵ
#define		STANDARD_MAX_NORMAL       9000//7600     //ƽ�ؿ�������ٶȣ���ֹҡ�˱���*660�������ֵ
#define		REVOLVE_MAX_NORMAL        9000//7600     //ƽ��Ťͷ����ٶ�
#define     REVOLVE_KD                (235.f)
#define     REVOLVE_ANGLE             35

//��ͬģʽ��,б�º�����Ӧ��ʱ��ֵ,һ������ͨ�ٶȾ���
#define    TIME_INC_NORMAL           10//6//4	  //����б��,Խ�������ٶ�Խ��,���ʱ��Խ��
#define    TIME_DEC_NORMAL           500//180        //����б��,Խ���С�ٶ�Խ��(һ��Ҫ��INC��һ��,�����ɿ������ܸ���Ϊ0,̫�������ɵ���ͣ������ʱ����Ծ)

#define    TIME_INC_SLOW             1		  //����ģʽ���ٶȱ仯����
#define    TIME_INC_SZUPUP           3		  //�ֶ�����ģʽ���ٶȱ仯����

#define    TIME_INC_SALTATION        1        //ͻȻ����������ٶȱ仯����

#define    REVOLVE_SLOPE_NORMAL      80       //������ͨģʽб��,Խ��Խ��,���ʱ��Խ��
#define    REVOLVE_SLOPE_CORGI       50       //����Ťƨ��ģʽб��,Խ��Խ��,���ʱ��Խ��


//��ͬģʽ�µ�����ٶ�
#define    REVOLVE_MAX_CORGI         9000//5000     //����Ťƨ������ٶ�,̫����ýǶȹ���

#define    STANDARD_MAX_SLOW         1500//3000	  //����ģʽ��ˮƽ�ƶ��ٶ�
#define    REVOLVE_MAX_SLOW          2000//4000     //����ģʽ��Ťͷ�ٶ�

#define    STANDARD_MAX_SZUPUP       3000//5000//6000//4000//3600	  //�ֶ�����ģʽ��ˮƽ�ƶ��ٶ�
#define    REVOLVE_MAX_SZUPUP        9000     //�ֶ�����ģʽ��Ťͷ�ٶ�

#define    LIMIT_CHASSIS_MAX         9000     //������������µ��̵������������
#define    CHAS_CURRENT_LIMIT        36000//40000    //�ĸ����ӵ��ٶ��ܺ����ֵ,�������*4,�޹��ʵ���������

//ע����ת��ƽ�Ƶı���Ҫ�ֿ�

/*************������************/
//��еģʽ�µ��̱���ϵ��,����ҡ����Ӧ�ٶ�,�����СҲ���������ת��,max = ��ϵ�� *660
float kRc_Mech_Chassis_Standard, kRc_Mech_Chassis_Revolve;//ƽ�ƣ���ת,

//������ģʽ�µ��̱���ϵ��,����ҡ����Ӧ�ٶ�,�����СҲ���������ת��,max = ��ϵ�� *660
float kRc_Gyro_Chassis_Standard, kRc_Gyro_Chassis_Revolve;//ƽ�ƣ���ת

//��еģʽ�µ��̱���ϵ��,���Ƽ���б�±仯��
float kKey_Mech_Chassis_Standard, kKey_Mech_Chassis_Revolve;//ƽ�ƣ���ת

//������ģʽ�µ��̱���ϵ��,���Ƽ���б�±仯��
float kKey_Gyro_Chassis_Standard, kKey_Gyro_Chassis_Revolve;//ƽ�ƣ���ת

/**************ȫ���ƶ�����****************/
float Chassis_Move_X;//ǰ��
float Chassis_Move_Y;//����ƽ��
float Chassis_Move_Z;//������ת


/***********PID����***************/
//���������ٶ�
float Chassis_Speed_Target[4];//ID

//���̲����ٶ�
int16_t Chassis_Speed_Measure[4];

//���̲����Ƕ�
int16_t Chassis_Angle_Measure[4];

//���̲����ٶ�
int16_t Chassis_Current_Measure[4];

//�����ٶ����
float Chassis_Speed_Error[4];//ID

//�����ٶ�����
float Chassis_Speed_Error_Sum[4];//ID
float Chassis_Speed_Error_NOW[4], Chassis_Speed_Error_LAST[4];

//������ģʽ�µ���ƫ��(���YAW�����ķ����е�Ƕ�)
int16_t Chassis_Gyro_Error;

//����PID����
float Chassis_Speed_kpid[4][3];//	motorID kp/ki/kd

float   pTermChassis[4], iTermChassis[4], dTermChassis[4];//ID
float	pidTermChassis[4];//ID,���������

//���̵�������
float Chassis_Final_Output[4];

//����Ťͷ
float Chassis_Z_kpid[3] = {-11, 0, 5};
float pTermChassisZ;
float dTermChassisZ;

/**************�޷�**************/
float Chassis_Final_Output_Max = LIMIT_CHASSIS_MAX;//����PID�����������ֵ,���̹�������,���ݵ��ת��ʵʱ�仯
float Chassis_Standard_Move_Max;//����ǰ������ƽ������
float Chassis_Revolve_Move_Max;//����������ת����,���ݲ�ͬ�˶�ģʽʵʱ����,���Բ����ú궨��

float Chassis_Limit_Output_Max=LIMIT_CHASSIS_MAX;//���̹����޷�


/**************б��***************/
float Slope_Chassis_Move_Z;//б�¼�������ƶ�����,����Ŀ���������ʵʱб��ֵ

uint16_t timeInc;//б�����ӱ仯ʱ��
uint16_t timeInc_Saltation;//ǰ����ͻ���µ�б��������,�����������ҪС
int16_t   timeXFron,    timeXBack,    timeYLeft,    timeYRigh;//����  s  w  d   a

//����ģʽ��ȫ���ƶ�����,б����
float Slope_Chassis_Move_Fron, Slope_Chassis_Move_Back; 
float Slope_Chassis_Move_Left, Slope_Chassis_Move_Righ;

//����ģʽ��Ťͷб��,��Ҫ����Ťƨ��ģʽ��
float Slope_Chassis_Revolve_Move;


/**************����ϵͳ������������*****************/
#if JUDGE_VERSION == JUDGE_18
	float WARNING_REMAIN_POWER = 50;//����ϵͳʣ�ཹ���������������ֵ��ʼ�޹���,40Ťƨ�ɻᳬ����,ƽ�ؿ����ᳬ
#elif JUDGE_VERSION == JUDGE_19
	float WARNING_REMAIN_POWER = 60;
#endif

float fChasCurrentLimit = CHAS_CURRENT_LIMIT;//����4�����ӵ��ٶ��ܺ�
float fTotalCurrentLimit;//��������,ƽ��ģʽ�·����Ǿ��ȵ�


/***********���̸���ģʽ��һЩ��������*************/

//Ťƨ��
bool Chass_Switch_F = 1;
uint8_t 	 Chass_Key_F_Change = 0;

//45��
bool Chass_Switch_X = 1;
uint8_t 	 Chass_Key_X_Change = 0;

//�Զ�����
#define   MISS_MAX_TIME    1000    //�Զ���������λʱ��,��λ2*ms
uint32_t  Miss_Mode_Time = 0;//�Զ������ѹ�ʱ��

//float  Corgi_XY_Compensation = 1.1;//Ťƨ��ģʽ�µ�Z���ٶȲ���

#define RADIUS     76      //���ְ뾶
#define PERIMETER  478     //�����ܳ�
#define WHEELTRACK 360//398     //�����־�
#define WHEELBASE  300     //ǰ�����
#define GIMBAL_X_OFFSET 0//75  //��̨��Ե������ĵ�ǰ��ƫ����
#define GIMBAL_Y_OFFSET 0 //��̨��Ե������ĵ�����ƫ����
#define CHASSIS_DECELE_RATIO (1.0f/19.0f)  //���������
#define RADIAN_COEF 57.3f  

extKalman_t Chassis_Error_Kalman;//����һ��kalmanָ��

/************************************************************************************/
/************************************************************************************/
extern bool Hurt_Data_Update;

eChassisCtrlMode  modeChassis;
eChassisAction  actChassis;

/********************************************************************************/
/**
  * @brief  ���̲�����ʼ��
  * @param  void
  * @retval void
  * @attention 
  */
void CHASSIS_InitArgument(void)
{
	/* ������ */
	kRc_Mech_Chassis_Standard = 14.f;//����ҡ��������,�����ٶ�
	kRc_Mech_Chassis_Revolve  = 11.4f;//10.6;//���ڻ�еģʽҡ��Ťͷ������(̫С��Ӱ������ٶ�)
	kRc_Gyro_Chassis_Standard = 14;//������ģʽҡ��ƽ��������
	kRc_Gyro_Chassis_Revolve  = -10;//10;//-7.1;//������ģʽҡ��Ťͷ������,̫�����
	
	kKey_Mech_Chassis_Revolve = 40;//���̻�еģʽ��Ťͷ�ٶ���Ӧ����,��̫��,��ȻŤͷ̫��
	kKey_Gyro_Chassis_Revolve = -10;//-8.1;//ע������,����������ģʽ�µ��̸�����̨ת�����ٶ�,���̫��,�����𵴻������
	
	/* ����PID���� */
	Chassis_Speed_kpid[LEFT_FRON_201][KP] = 8;
	Chassis_Speed_kpid[LEFT_FRON_201][KI] = 100;
	Chassis_Speed_kpid[LEFT_FRON_201][KD] = 10;
	
	Chassis_Speed_kpid[RIGH_FRON_202][KP] = 8;
	Chassis_Speed_kpid[RIGH_FRON_202][KI] = 100;
	Chassis_Speed_kpid[RIGH_FRON_202][KD] = 10;
	
	Chassis_Speed_kpid[LEFT_BACK_203][KP] = 8;
	Chassis_Speed_kpid[LEFT_BACK_203][KI] = 100;
	Chassis_Speed_kpid[LEFT_BACK_203][KD] = 10;
	
	Chassis_Speed_kpid[RIGH_BACK_204][KP] = 8;
	Chassis_Speed_kpid[RIGH_BACK_204][KI] = 100;
	Chassis_Speed_kpid[RIGH_BACK_204][KD] = 10;

	KalmanCreate(&Chassis_Error_Kalman, 1, 0);
	
    Chassis_Standard_Move_Max  = Omni_Speed_Max;//6800;//9000;//ҡ��ˮƽ�ƶ��޷�
}

/**
  * @brief  ����ʧ��ֹͣ
  * @param  void
  * @retval void
  * @attention �����ֵΪ��,�ǵ�Ҫ���㷢�ͳ�ȥ
  */
void CHASSIS_StopMotor(void)
{
	Chassis_Final_Output[ 0 ] = 0;
	Chassis_Final_Output[ 1 ] = 0;
	Chassis_Final_Output[ 2 ] = 0;
	Chassis_Final_Output[ 3 ] = 0;
}

/**
  * @brief  ��������״̬
  * @param  void
  * @retval void
  * @attention ״ֵ̬0
  */
void CHASSIS_REST(void)
{
	Slope_Chassis_Move_Z = 0;//Ťƨ��ʵʱ���б��
	Chassis_Move_X = 0;
	Chassis_Move_Y = 0;
	Chassis_Move_Z = 0;
}

/******************************/
/**
  * @brief  ң�ؿ��Ƶ����ƶ�
  * @param  void
  * @retval void
  * @attention �ڴ˼����ƶ�����
  */
void CHAS_Rc_Ctrl(void)
{
	float k_rc_z = 1;//����Z�ٶȵ���ǰ������ƽ�����ٱ�
	
	if (IF_RC_SW2_DOWN)//S2��������
	{
		modeChassis = CHASSIS_GYRO_MODE;
	}
	else
	{
		modeChassis = CHASSIS_MECH_MODE;
	}
	
	//Ťͷб���л�����ͨģʽ(Ťƨ��ģʽб�¸�λ)
	Slope_Chassis_Revolve_Move = REVOLVE_SLOPE_NORMAL;
	//�ƶ��ٶ�����
	Chassis_Standard_Move_Max = STANDARD_MAX_NORMAL;//ƽ���޷�
	Chassis_Revolve_Move_Max  = REVOLVE_MAX_NORMAL;//Ťͷ�޷�
	
	//��¼ҡ���ƶ�����
	if(modeChassis == CHASSIS_MECH_MODE)//��еģʽ
	{	
		Chassis_Move_Z = limitVal<float>( kRc_Mech_Chassis_Revolve*RC_CH0_RLR_OFFSET, -Chassis_Revolve_Move_Max, Chassis_Revolve_Move_Max);//��ת
		
		if(fabs(Chassis_Move_Z) > 800)//Ťͷ�ٶ�Խ��,ǰ���ٶ�Խ��,��ֹת��뾶����
		{
			k_rc_z = ( (Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800) * (Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800) )
						/ ( Chassis_Revolve_Move_Max * Chassis_Revolve_Move_Max );
			
			k_rc_z = limitVal<float>(k_rc_z,0,1);
		}
		else
		{
			k_rc_z = 1;
		}
		
		Chassis_Move_X = limitVal<float>( kRc_Mech_Chassis_Standard*RC_CH3_LUD_OFFSET, -Chassis_Standard_Move_Max*k_rc_z, Chassis_Standard_Move_Max*k_rc_z);//ǰ��
		
		Chassis_Move_Y = limitVal<float>( kRc_Mech_Chassis_Standard*RC_CH2_LLR_OFFSET, -Chassis_Standard_Move_Max*k_rc_z, Chassis_Standard_Move_Max*k_rc_z);//ˮƽ
	
	}
	else if(modeChassis == CHASSIS_GYRO_MODE)//������ģʽ
	{
		Chassis_Gyro_Error = GIMBAL_GetOffsetAngle( );//�������YAW��ƫ��

		Chassis_Move_Z = Chassis_SpeedZ_PID(Chassis_Gyro_Error, kRc_Gyro_Chassis_Revolve);
	
		if(fabs(Chassis_Move_Z) > 800)//Ťͷ�ٶ�Խ��,ǰ���ٶ�Խ��,��ֹת��뾶����
		{
			k_rc_z = ( (Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800) * (Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800) )
						/ ( Chassis_Revolve_Move_Max * Chassis_Revolve_Move_Max );
			
			k_rc_z = limitVal<float>(k_rc_z,0,1);
		}
		else
		{
			k_rc_z = 1;
		}
		Chassis_Move_X = limitVal<float>( kRc_Gyro_Chassis_Standard*RC_CH3_LUD_OFFSET, -Chassis_Standard_Move_Max * k_rc_z, Chassis_Standard_Move_Max * k_rc_z);//ǰ��
			
		Chassis_Move_Y = limitVal<float>( kRc_Gyro_Chassis_Standard*RC_CH2_LLR_OFFSET, -Chassis_Standard_Move_Max * k_rc_z, Chassis_Standard_Move_Max * k_rc_z);//ˮƽ

	}
	
}

/**
  * @brief  ���̿��Ƶ����ƶ�
  * @param  void
  * @retval void
  * @attention ģʽѡ��,����ĳģʽ��ǵ�д�˳�����ͨģʽ���ж�
  * �ް������»�һֱ�����Զ�����ģʽ,����ģʽ�л���İ�����������ģʽ�л�ѡ��ģʽ
  */
void CHAS_Key_Ctrl(void)
{
	if(remot_change == TRUE)//�մ�ң��ģʽ�й���,Ĭ��Ϊ������ģʽ
	{
		modeChassis = CHASSIS_GYRO_MODE;
		remot_change = FALSE;
	}
	
	if(GIMBAL_IfBuffHit() == TRUE)//���ģʽΪ��е
	{
		modeChassis = CHASSIS_MECH_MODE;
	}
	
	switch (actChassis)      //SB keil �о���,����ģʽѡ�񣬲�����������ͨģʽ
	{
		/*------------------��ͨģʽ,����ģʽ�л��ж�-------------------*/	
		case CHASSIS_NORMAL:		
			Chassis_NORMAL_Mode_Ctrl();						
		break;
		
		/*------------------Ťƨ��ģʽ-------------------*/			
		case CHASSIS_CORGI:	
			if(!IF_KEY_PRESSED_F)//F�ɿ�
			{
				Chass_Switch_F = 1;
			}
			
			if(IF_KEY_PRESSED_F && Chass_Switch_F == 1)
			{
				Chass_Switch_F = 0;
				Chass_Key_F_Change ++;
				Chass_Key_F_Change %= 2;
			}
			//����ǰ���ƶ�,��������ƽ�ơ�QE���л���еģʽ�˳�
			if(Chass_Key_F_Change)
			{
				modeChassis = CHASSIS_GYRO_MODE;//������ģʽ,���̸�����̨��
				
				Chassis_Keyboard_Move_Calculate(STANDARD_MAX_NORMAL, TIME_INC_NORMAL);
				CHASSIS_CORGI_Mode_Ctrl( REVOLVE_MAX_CORGI, REVOLVE_SLOPE_CORGI);
//				CHASSIS_CORGI_Mode_Ctrl_Time( REVOLVE_MAX_CORGI, REVOLVE_SLOPE_CORGI);				
			}
			else
			{
				actChassis = CHASSIS_NORMAL;//�˳�Ťƨ��ģʽ
			}
			
			//���ֿ���ʱҪ�ر�Ť��
			if((IF_KEY_PRESSED_CTRL || GIMBAL_IfGIMBAL_LEVEL()==TRUE ) && !IF_KEY_PRESSED_F)
			{
				Chass_Switch_F = 1;
				Chass_Key_F_Change ++;
				Chass_Key_F_Change %= 2;
				actChassis = CHASSIS_NORMAL;//�˳�Ťƨ��ģʽ
			}
			else if(IF_KEY_PRESSED_X)//����ʱ����45��ģʽ
			{
				Chass_Switch_F = 1;
				Chass_Key_F_Change ++;
				Chass_Key_F_Change %= 2;
				actChassis = CHASSIS_PISA;
			}			
		break;
			
		/*-------------���ģʽ-------------*/
		case CHASSIS_ROSHAN:
						
			if(GIMBAL_IfBuffHit( ) != TRUE)
			{
				actChassis = CHASSIS_NORMAL;//�˳����ģʽ
				modeChassis = CHASSIS_GYRO_MODE;//�л�������ģʽ
			}	
			else
			{
				modeChassis = CHASSIS_MECH_MODE;//������̽����еģʽ
				CHASSIS_REST();//Ŀ���ٶ���0
			}				
		break;
				
		/*---------------����ģʽ,���̵���----------------*/
		case CHASSIS_SLOW:
			if (Magazine_IfOpen() != TRUE)//���ֹر�
			{
				actChassis = CHASSIS_NORMAL;//�����˳�����ģʽ
			}
			else
			{
				modeChassis = CHASSIS_MECH_MODE;//����ʱ���̽����еģʽ
			
				Chassis_Keyboard_Move_Calculate( STANDARD_MAX_SLOW, TIME_INC_SLOW );
				Chassis_Mouse_Move_Calculate( REVOLVE_MAX_SLOW );
			}
		break;
				
		/*-------------�ֶ�����ģʽ-------------*/
		case CHASSIS_SZUPUP:
			CHASSIS_SZUPUP_Mode_Ctrl();
		break;
			
		/*-------------�Զ�����ģʽ-------------*/
//		case CHASSIS_MISS:
//			CHASSIS_MISS_Mode_Ctrl();
//		break;
		
		/*-------------45��Ե�ģʽ-------------*/
		case CHASSIS_PISA:
			if(!IF_KEY_PRESSED_X)//F�ɿ�
			{
				Chass_Switch_X = 1;
			}
			
			if(IF_KEY_PRESSED_X && Chass_Switch_X == 1)
			{
				Chass_Switch_X = 0;
				Chass_Key_X_Change ++;
				Chass_Key_X_Change %= 2;
			}
			
			if(Chass_Key_X_Change)//ǰ���˳�
			{
				Chassis_Keyboard_Move_Calculate(STANDARD_MAX_NORMAL, TIME_INC_NORMAL);
				CHASSIS_PISA_Mode_Ctrl();
			}
			else
			{
				actChassis = CHASSIS_NORMAL;
			}
			
			//���ֿ���ʱҪ�ر�
			if( (IF_KEY_PRESSED_CTRL 
					|| GIMBAL_IfGIMBAL_LEVEL()==TRUE 
						|| IF_KEY_PRESSED_W || IF_KEY_PRESSED_S) 
							&& !IF_KEY_PRESSED_X)
			{
				Chass_Switch_X = 1;
				Chass_Key_X_Change ++;
				Chass_Key_X_Change %= 2;
				actChassis = CHASSIS_NORMAL;//�˳�Ťƨ��ģʽ
			}
			else if(IF_KEY_PRESSED_F)//����ʱ����Ť��ģʽ
			{
				Chass_Switch_X = 1;
				Chass_Key_X_Change ++;
				Chass_Key_X_Change %= 2;
				actChassis = CHASSIS_CORGI;
			}
		break;
	}
}

/*************************���̼���ģʽ����ģʽС����****************************/

/**
  * @brief  ����ģʽ�µ����˶�����
  * @param  �ٶ���������    �����ٶ�(���293)
  * @retval void
  * @attention  ���̿���ǰ������ƽ��,ƽ���޻�е��������ģʽ֮��
  *             ��Ҫ��ȡʱ��������б�º�������
  */
void Chassis_Keyboard_Move_Calculate( int16_t sMoveMax, int16_t sMoveRamp )
{
	static portTickType  ulCurrentTime = 0;
	static uint32_t  ulDelay = 0;
	float k_rc_z = 1;//����Z�ٶȵ���ǰ������ƽ�����ٱ�
	
	Chassis_Standard_Move_Max = sMoveMax;//�����ٶ��޷�,ˮƽ�ƶ�
	timeInc      = sMoveRamp;
	
	ulCurrentTime = xTaskGetTickCount();//��ǰϵͳʱ��
	
	if(fabs(Chassis_Move_Z) > 800)//Ťͷ�ٶ�Խ��,ǰ���ٶ�Խ��,��ֹת��뾶����
	{
		k_rc_z = ( (Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800) * (Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800) )
					/ ( Chassis_Revolve_Move_Max * Chassis_Revolve_Move_Max );
		
		k_rc_z = limitVal<float>(k_rc_z,0,1);
	}
	else
	{
		k_rc_z = 1;
	}
	
	if (ulCurrentTime >= ulDelay)//ÿ10ms�仯һ��б����
	{
		ulDelay = ulCurrentTime + TIME_STAMP_10MS;


		if(actChassis == CHASSIS_NORMAL && !KEY_PRESSED_OFFSET_SHIFT)//ֻ��һ��ģʽ�²��ж��ٶ�ͻ�����,��ֹ��
		{
			if (IF_KEY_PRESSED_W)//�ȳ������ݳ����ٲ��Ե��ݷŵ�ʱ�Ƿ�Ҫȫ������
			{
				timeXBack = 0;//����ǰ�������б�¹���,�����´μ������б��
				//ǰ��X������,���ڴ˼��볬�����ݰ����ж�
				if( Chassis_Move_X < sMoveMax/2.5 )//ת��ͻ��,�տ�ʼ��һС��ʱ��б�½���,��ֹ���Ӵ��˷ѹ���
				{//�����ٶ��Ƿ�����ٶȵ�1/5���жϲ�֪���Ƿ����
					timeInc_Saltation = TIME_INC_SALTATION;//������ģʽ�����ٶȷ���ͻ��
				}
				else			//�Ѿ����˴�ʱ������������һ�����ٶ�
				{
					timeInc_Saltation = sMoveRamp;
				}
			}

			if (IF_KEY_PRESSED_S)
			{
				timeXFron = 0;//ͬ��
				//����X�Ǹ�
				if( Chassis_Move_X > (-sMoveMax)/2.5 )//ת��ͻ��,�տ�ʼ��һС��ʱ��б�½���,��ֹ���Ӵ��˷ѹ���
				{
					timeInc_Saltation = TIME_INC_SALTATION;//������ģʽ�����ٶȷ���ͻ��
				}
				else			//�Ѿ����˴�ʱ������������һ�����ٶ�
				{
					timeInc_Saltation = sMoveRamp;
				}
			}

			if (IF_KEY_PRESSED_D)
			{
				timeYRigh = 0;
			}

			if (IF_KEY_PRESSED_A)
			{
				timeYLeft = 0;
			}
		
			//����ģʽ��ȫ���ƶ�,б��������,ע������,��������*б�±����õ��������ӵ�ֵ,ģ��ҡ��
			//ǰ�������б���Ǳ仯��
			Slope_Chassis_Move_Fron = (int16_t)( Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_W, &timeXFron, timeInc_Saltation, TIME_DEC_NORMAL ) );

			Slope_Chassis_Move_Back = (int16_t)( -Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_S, &timeXBack, timeInc_Saltation, TIME_DEC_NORMAL ) );

			//���ҵ�����б�¸�ǰ��һ��,����
			Slope_Chassis_Move_Left = (int16_t)( -Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_A, &timeYRigh, timeInc/1.5, TIME_DEC_NORMAL ) );

			Slope_Chassis_Move_Righ = (int16_t)( Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_D, &timeYLeft, timeInc/1.5, TIME_DEC_NORMAL ) );


			Chassis_Move_X  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) * k_rc_z;//ǰ�����
			Chassis_Move_Y  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) * k_rc_z;//���Ҽ���
		}
		else		//����ģʽ����Ҫ�����ٶȷ���ͻ�����⴦��
		{
			if (IF_KEY_PRESSED_W)
			{
				timeXBack = 0;//����ǰ�������б�¹���,�����´μ������б��
			}

			if (IF_KEY_PRESSED_S)
			{
				timeXFron = 0;//ͬ��
			}

			if (IF_KEY_PRESSED_D)
			{
				timeYRigh = 0;
			}

			if (IF_KEY_PRESSED_A)
			{
				timeYLeft = 0;
			}
			
			Slope_Chassis_Move_Fron = (int16_t)( Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_W, &timeXFron, timeInc, TIME_DEC_NORMAL ) );

			Slope_Chassis_Move_Back = (int16_t)( -Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_S, &timeXBack, timeInc, TIME_DEC_NORMAL ) );

			Slope_Chassis_Move_Left = (int16_t)( -Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_A, &timeYRigh, timeInc, TIME_DEC_NORMAL ) );

			Slope_Chassis_Move_Righ = (int16_t)( Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_D, &timeYLeft, timeInc, TIME_DEC_NORMAL ) );

			if(actChassis != CHASSIS_CORGI)
			{
				Chassis_Move_X  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) * k_rc_z;//ǰ�����
				Chassis_Move_Y  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) * k_rc_z;//���Ҽ���
			}
		}
	}
}

/**
  * @brief  �����Ƶ�����ת,����QEC���ƿ���תȦ
  * @param  �ٶ��������� 
  * @retval void
  * @attention  ������������ת
  */
void Chassis_Mouse_Move_Calculate( int16_t sRevolMax )
{
//	static int16_t sErrorPrev = 0;//�ϴ�ƫ�����
	int16_t sErrorReal = 0;//yawƫ���������
	
	
	Chassis_Revolve_Move_Max = sRevolMax;//������ת����
	
	if(modeChassis == CHASSIS_GYRO_MODE)//������ģʽ
	{
		sErrorReal = GIMBAL_GetOffsetAngle();//��ȡʵʱƫ��,����Ťƨ���µĵ���λ�ò���
		Chassis_Move_Z = Chassis_SpeedZ_PID(sErrorReal, kKey_Gyro_Chassis_Revolve);
	}
	else     //��еģʽ
	{
		Chassis_Move_Z = limitVal<float>( MOUSE_X_MOVE_SPEED*kKey_Mech_Chassis_Revolve, -Chassis_Revolve_Move_Max, Chassis_Revolve_Move_Max);
	}
}

/**
  * @brief  ���̼���ģʽѡ��,������Ӧ
  * @param  void
  * @retval void
  * @attention ���̼��̿���״̬�µ�����ģʽ�л�������
  */
void Chassis_NORMAL_Mode_Ctrl(void)
{
	if(!IF_KEY_PRESSED_F)//F�ɿ�
	{
		Chass_Switch_F = 1;
	}
	
	if(!IF_KEY_PRESSED_X)//X�ɿ�
	{
		Chass_Switch_X = 1;
	}
	
	if (IF_KEY_PRESSED_F && !IF_KEY_PRESSED_CTRL 
			&& GIMBAL_IfBuffHit() == FALSE && Chass_Switch_F == 1)//F����,�л���Ťƨ��(����һֱ��F)
	{
		Chass_Switch_F = 0;
		Chass_Key_F_Change ++;
		Chass_Key_F_Change %= 2;
		actChassis = CHASSIS_CORGI;//�ǵ�д�����˳�Ťƨ��ģʽ�ĺ���
	}
	else if(Magazine_IfOpen() == TRUE)//���ֿ���,���벹��ģʽ
	{
		actChassis = CHASSIS_SLOW;
	}
	else if(IF_KEY_PRESSED_W && IF_KEY_PRESSED_CTRL)//W Ctrlһ�𰴽�������ģʽ
	{
		actChassis = CHASSIS_SZUPUP;//����ģʽ
	}
//	else if(JUDGE_IfArmorHurt() == TRUE && GIMBAL_IfBuffHit() == FALSE)//ֻҪ�˺��������ް�������,�ͻ�����Զ�����ģʽ
//	{
//		actChassis = CHASSIS_MISS;//���ʱ�ر��Զ�����
//	}
	else if(GIMBAL_IfBuffHit() == TRUE)//���ģʽ
	{
		actChassis = CHASSIS_ROSHAN;
	}
	else if (IF_KEY_PRESSED_X && !IF_KEY_PRESSED_CTRL 
			&& GIMBAL_IfBuffHit() == FALSE && Chass_Switch_X == 1)//x����,�л���45��
	{
		Chass_Switch_X = 0;
		Chass_Key_X_Change ++;
		Chass_Key_X_Change %= 2;
		actChassis = CHASSIS_PISA;
	}
	else 					
	{		
		//���ʱǿ�ƽ����еģʽ
		if(IF_KEY_PRESSED_CTRL || GIMBAL_IfBuffHit() == TRUE)      //��סCTRL�����еģʽ
		{
			modeChassis = CHASSIS_MECH_MODE;
		}
		else			//�ɿ�CTRL����������ģʽ
		{
			modeChassis = CHASSIS_GYRO_MODE;
		}

		//�ƶ��ٶȿ���
		if(Cap_Out_Can_Open() == TRUE)//���ݷŵ�
		{
			Chassis_Keyboard_Move_Calculate(Omni_SupCap_Max, TIME_INC_NORMAL);//�ŵ�ʱ����ٶ�Ҫ��
			Chassis_Mouse_Move_Calculate(REVOLVE_MAX_NORMAL);
		}
		else
		{
			Chassis_Keyboard_Move_Calculate(STANDARD_MAX_NORMAL, TIME_INC_NORMAL);//�����ٶ����ֵ��б��ʱ��
			Chassis_Mouse_Move_Calculate(REVOLVE_MAX_NORMAL);			
		}
	}	
}

/**
  * @brief  Ťƨ��ģʽ(λ�ò����)
  * @param  �ٶ���������    ���ӵ����������ʱ��
  * @retval void
  * @attention  ����ʱ�䣬Ť��λ�˾ͻ���
  */
//Ťƨ�ɻ���ѡ��
#define    CORGI_BEGIN    0    
#define    CORGI_LEFT     1
#define    CORGI_RIGH     2

uint16_t  stateCorgi = CORGI_BEGIN;//�������Ť,Ĭ�ϲ�Ť
bool    IfCorgiChange = FALSE;//�Ƿ�Ť������һ��
int16_t  corgi_angle_target = 0;//����Ŀ��Ƕ�
void CHASSIS_CORGI_Mode_Ctrl(int16_t sRevolMax, int16_t sRevolRamp)
{
	int16_t  sAngleError   = 0;
	float    vectorXBuffer = 0;
	float    vectorYBuffer = 0;
	float    angle         = 0;


	Chassis_Revolve_Move_Max = sRevolMax;//����ٶ�����
	Slope_Chassis_Revolve_Move = sRevolRamp;//Ťͷб������

	sAngleError = GIMBAL_GetOffsetAngle();//����yaw����ƫ��,��֤����Ťƨ�ɵ�ʱ��Ҳ�ܸ�����̨��

	//����Ƕ�ƫ��,��е�Ƕ�ת����ŷ����,����ǰ���ٶȲ���
	angle = -(float)sAngleError / (float)8192 * 6.283f;

	vectorXBuffer = Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron;//�ݴ�ʵʱX�仯
	vectorYBuffer = Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ;

	Chassis_Move_X = vectorXBuffer * cos( angle ) - vectorYBuffer * sin( angle );
	Chassis_Move_Y = vectorXBuffer * sin( angle ) + vectorYBuffer * cos( angle );

	//�ؼ�:��������......
	switch (stateCorgi)
	{
		case CORGI_BEGIN:	//�Ժ���������ø����(��־λ��ͣȡ��),���ÿ�ʼŤͷ�ķ������	  
			corgi_angle_target = -900;//�ɸ�����ƶ��Ƕ�,�Զ�����ģʽ�±�����ʱ��Ť���Ƕ�
			IfCorgiChange = FALSE;
			stateCorgi    = CORGI_LEFT;
		break;
		
		case CORGI_LEFT:
			corgi_angle_target = -1024;//�ɸ�����ƶ��Ƕ�			
			IfCorgiChange = FALSE;

			if (sAngleError < -700)//�Ƕ�������700
			{
					stateCorgi = CORGI_RIGH;
				  IfCorgiChange = TRUE;//��ǿ��Ի���
			}			
		break;
			
		case CORGI_RIGH:		
			corgi_angle_target = 1024;
			IfCorgiChange = FALSE;

			if (sAngleError > 700)//�Ƕ�������700
			{
				stateCorgi = CORGI_LEFT;
				IfCorgiChange = TRUE;//��ǿ��Ի���
			}			
		break;
	}

	Chassis_Move_Z = Chassis_SpeedZ_PID( (sAngleError - corgi_angle_target), kKey_Gyro_Chassis_Revolve);
}

/**
  * @brief  Ťƨ��ģʽ(ʱ�䲻���)
  * @param  �ٶ���������    ���ӵ����������ʱ��
  * @retval void
  * @attention  ����ŤûŤ��λ��ʱ�䵽�˾ͻ���
  */
void CHASSIS_CORGI_Mode_Ctrl_Time(int16_t sRevolMax, int16_t sRevolRamp)
{
	static uint32_t twist_count;
	static float  chass_yaw_set;
	float    angle         = 0;
	int16_t  sAngleError   = 0;
	float    vectorXBuffer = 0;
	float    vectorYBuffer = 0;
	
	
	static int16_t twist_period = 800;//500*2msΪһ��Ť������
	static int16_t twist_angle  = 40;//����������ŷ���Ƕ�����
	
	
	twist_count++; 

	Chassis_Revolve_Move_Max = sRevolMax;//����ٶ�����
	Slope_Chassis_Revolve_Move = sRevolRamp;//Ťͷб������

	sAngleError = GIMBAL_GetOffsetAngle();//����yaw����ƫ��,��֤����Ťƨ�ɵ�ʱ��Ҳ�ܸ�����̨��

	//����Ƕ�ƫ��,��е�Ƕ�ת����ŷ����,����ǰ���ٶȲ���
	angle = -(float)sAngleError / (float)8192 * 360;//6.283f;

	vectorXBuffer = Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron;//�ݴ�ʵʱX�仯
	vectorYBuffer = Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ;
	
	//���Һ���Ťƨ��
	Chassis_Move_X = vectorXBuffer * cos( angle/57.3f ) - vectorYBuffer * sin( angle/57.3f );
	Chassis_Move_Y = vectorXBuffer * sin( angle/57.3f ) + vectorYBuffer * cos( angle/57.3f );

	chass_yaw_set = twist_angle * sin(2 * PI / twist_period * twist_count);//�������Ŀ�����Ƕ�
	Chassis_Move_Z = -Chassis_Z_Corgi(angle, chass_yaw_set);
}

/**
  * @brief  �ֶ�����ģʽ
  * @param  void
  * @retval void
  * @attention  
  */
void CHASSIS_SZUPUP_Mode_Ctrl(void)
{
	if( !IF_KEY_PRESSED_W || !IF_KEY_PRESSED_CTRL)//�ɿ�����һ���˳�����ģʽ
	{
		actChassis = CHASSIS_NORMAL;//�����˳�����ģʽ
	}
	else
	{
		modeChassis = CHASSIS_GYRO_MODE;//������ģʽ
		
		Chassis_Keyboard_Move_Calculate( STANDARD_MAX_SZUPUP, TIME_INC_SZUPUP );
		Chassis_Mouse_Move_Calculate( REVOLVE_MAX_SZUPUP );
	}
}

/**
  * @brief  �Զ�����ģʽ
  * @param  void
  * @retval void
  * @attention  
  */
void CHASSIS_MISS_Mode_Ctrl(void)
{
	int16_t  sAngleError   = 0;
	sAngleError = GIMBAL_GetOffsetAngle();//����yaw����ƫ��,��֤����Ťƨ�ɵ�ʱ��Ҳ�ܸ�����̨��

	//�а������»��߳�ʱ��û���ܵ��������˳��Զ�����,��֤��������
	if( IF_KEY_PRESSED || Miss_Mode_Time > MISS_MAX_TIME )
	{
		actChassis = CHASSIS_NORMAL;//�����л�������ģʽ
		Miss_Mode_Time = 0;
	}
	else
	{
		modeChassis = CHASSIS_GYRO_MODE;
		
		if(JUDGE_IfArmorHurt() == TRUE 	//װ�װ����ݸ���,���ܵ��µ��˺�
			|| IfCorgiChange == FALSE)  //ƨ��û��Ť���Ա�
		{
			//����Ťƨ��һ��
			CHASSIS_CORGI_Mode_Ctrl( REVOLVE_MAX_CORGI, REVOLVE_SLOPE_CORGI);
			Miss_Mode_Time = 0;
		}
		else
		{
			Slope_Chassis_Move_Z = 0;//Ťƨ��ʵʱ���б��
			Chassis_Move_X = 0;
			Chassis_Move_Y = 0;
			Chassis_Move_Z = Chassis_SpeedZ_PID( (sAngleError - corgi_angle_target), kKey_Gyro_Chassis_Revolve);
			Miss_Mode_Time++;
		}	
	}
}

/**
  * @brief  45��ģʽ
  * @param  void
  * @retval void
  * @attention  
  */
void CHASSIS_PISA_Mode_Ctrl(void)
{
	int16_t  corgi_angle_target = 0;//����Ŀ��Ƕ�
	int16_t  sAngleError   = 0;
	float    vectorXBuffer = 0;
	float    vectorYBuffer = 0;
	float    angle         = 0;

	sAngleError = GIMBAL_GetOffsetAngle();//����yaw����ƫ��,��֤����Ťƨ�ɵ�ʱ��Ҳ�ܸ�����̨��

	//����Ƕ�ƫ��,��е�Ƕ�ת����ŷ����,����ǰ���ٶȲ���
	angle = -(float)sAngleError / (float)8192 * 6.283f;

	vectorXBuffer = Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron;//�ݴ�ʵʱX�仯
	vectorYBuffer = Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ;

	Chassis_Move_X = vectorXBuffer * cos( angle ) - vectorYBuffer * sin( angle );
	Chassis_Move_Y = vectorXBuffer * sin( angle ) + vectorYBuffer * cos( angle );

	corgi_angle_target = -1024;//�ɸ�����ƶ��Ƕ�

	Chassis_Move_Z = Chassis_SpeedZ_PID( (sAngleError - corgi_angle_target), kKey_Gyro_Chassis_Revolve);
}

/*******************���̵�����ݸ���,CAN1�ж��е���*********************/

/**
  * @brief  ��ȡ����Ƕ�
  * @param  ID,CAN����
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�),CAN1�е���
  */
void CHASSIS_UpdateMotorAngle( eChassisWheel eWheel, int16_t angle )
{
    Chassis_Angle_Measure[ eWheel ] = angle;
}

/**
  * @brief  ��ȡ���ת��
  * @param  ID,CAN����
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�),CAN1�е���
  */
void CHASSIS_UpdateMotorSpeed( eChassisWheel eWheel, int16_t speed )
{
	Chassis_Speed_Measure[ eWheel ] = speed;
}

/**
  * @brief  ��ȡ���ת�ص���
  * @param  ID,CAN����
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�),CAN1�е���
  */
void CHASSIS_UpdateMotorCur( eChassisWheel eWheel, int16_t current )
{
	Chassis_Current_Measure[ eWheel ] = current;
}

/**
  * @brief  ���͵��̵���ֵ
  * @param  void
  * @retval void
  * @attention 
  */
void CHASSIS_CANbusCtrlMotor(void)
{	
	CAN1_Chassis_QueueSend(Chassis_Final_Output);
}

/**********************�����������**************************/

/**
  * @brief  ����ȫ���㷨,��������ת��
  * @param  void
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�)
  *              	Xǰ(+)��(-)     Y��(-)��(+)     ZŤͷ
  */
void Chassis_Omni_Move_Calculate(void)
{
	/* OLD */
//	float speed_max;

//	//ȫ���㷨
//	Chassis_Speed_Target[LEFT_FRON_201] = +( +Chassis_Move_Y +Chassis_Move_X +Chassis_Move_Z );
//	Chassis_Speed_Target[RIGH_FRON_202] = -( -Chassis_Move_Y +Chassis_Move_X -Chassis_Move_Z );
//	Chassis_Speed_Target[LEFT_BACK_203] = +( -Chassis_Move_Y +Chassis_Move_X +Chassis_Move_Z );
//	Chassis_Speed_Target[RIGH_BACK_204] = -( +Chassis_Move_Y +Chassis_Move_X -Chassis_Move_Z );
//	
//	//�޷�
//	if(Cap_Out_Can_Open() == TRUE)//���ݷŵ�
//	{
//		speed_max = Omni_SupCap_Max;
//	}
//	else
//	{
//		speed_max = Omni_Speed_Max;
//	}
//	
//	Chassis_Speed_Target[LEFT_FRON_201] = limitVal<float>( Chassis_Speed_Target[LEFT_FRON_201], -speed_max, speed_max);
//	Chassis_Speed_Target[RIGH_FRON_202] = limitVal<float>( Chassis_Speed_Target[RIGH_FRON_202], -speed_max, speed_max);
//	Chassis_Speed_Target[LEFT_BACK_203] = limitVal<float>( Chassis_Speed_Target[LEFT_BACK_203], -speed_max, speed_max);
//	Chassis_Speed_Target[RIGH_BACK_204] = limitVal<float>( Chassis_Speed_Target[RIGH_BACK_204], -speed_max, speed_max);

//	if(actChassis == CHASSIS_SZUPUP)//����ǰ�ֻ�򻬣������޵ñȺ���ҪС
//	{
//		//ǰ���޷�����ʵ����������������ֲ��ù�
//		Chassis_Speed_Target[LEFT_FRON_201] = limitVal<float>( Chassis_Speed_Target[LEFT_FRON_201], -STANDARD_MAX_SZUPUP, STANDARD_MAX_SZUPUP);
//		Chassis_Speed_Target[RIGH_FRON_202] = limitVal<float>( Chassis_Speed_Target[RIGH_FRON_202], -STANDARD_MAX_SZUPUP, STANDARD_MAX_SZUPUP);
//		Chassis_Speed_Target[LEFT_BACK_203] = limitVal<float>( Chassis_Speed_Target[LEFT_BACK_203], -STANDARD_MAX_SZUPUP, STANDARD_MAX_SZUPUP);
//		Chassis_Speed_Target[RIGH_BACK_204] = limitVal<float>( Chassis_Speed_Target[RIGH_BACK_204], -STANDARD_MAX_SZUPUP, STANDARD_MAX_SZUPUP);
//	}

	/* NEW */
	static float rotate_ratio_fl;//ǰ��
	static float rotate_ratio_fr;//ǰ��
  	static float rotate_ratio_bl;//����
  	static float rotate_ratio_br;//����
  	static float wheel_rpm_ratio;
	
	float speed_max;

	if(1)//��̨���ڵ�������
	{
		rotate_ratio_fr = ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET)/RADIAN_COEF;
		rotate_ratio_fl = ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET)/RADIAN_COEF;
		rotate_ratio_bl = ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET)/RADIAN_COEF;
		rotate_ratio_br = ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET)/RADIAN_COEF;
	}
	else
	{
		rotate_ratio_fr = ((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF;
		rotate_ratio_fl = rotate_ratio_fr;
		rotate_ratio_bl = rotate_ratio_fr;
		rotate_ratio_br = rotate_ratio_fr;
	}
	wheel_rpm_ratio = 60.0f/(PERIMETER * CHASSIS_DECELE_RATIO);             //	60/�ܳ�*������
	
	//ȫ���㷨
	Chassis_Speed_Target[LEFT_FRON_201] = +( +Chassis_Move_Y/(60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) +Chassis_Move_X/(60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) +Chassis_Move_Z / ( (((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF) * (60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) )*rotate_ratio_fl ) * wheel_rpm_ratio;
	Chassis_Speed_Target[RIGH_FRON_202] = -( -Chassis_Move_Y/(60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) +Chassis_Move_X/(60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) -Chassis_Move_Z / ( (((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF) * (60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) )*rotate_ratio_fr ) * wheel_rpm_ratio;
	Chassis_Speed_Target[LEFT_BACK_203] = +( -Chassis_Move_Y/(60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) +Chassis_Move_X/(60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) +Chassis_Move_Z / ( (((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF) * (60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) )*rotate_ratio_bl ) * wheel_rpm_ratio;
	Chassis_Speed_Target[RIGH_BACK_204] = -( +Chassis_Move_Y/(60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) +Chassis_Move_X/(60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) -Chassis_Move_Z / ( (((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF) * (60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) )*rotate_ratio_br ) * wheel_rpm_ratio;
	
	//�޷�
	if(Cap_Out_Can_Open() == TRUE)//���ݷŵ�
	{
		speed_max = Omni_SupCap_Max;
	}
	else
	{
		speed_max = Omni_Speed_Max;
	}
	
//	float   max = 0;
//	//find max item
//	for (uint8_t i = 0; i < 4; i++)
//	{
//		if (abs(Chassis_Speed_Target[i]) > max)
//		max = abs(Chassis_Speed_Target[i]);
//	}
//	//equal proportion
//	if (max > speed_max)
//	{
//		float rate = speed_max / max;
//		for (uint8_t i = 0; i < 4; i++)
//			Chassis_Speed_Target[i] *= rate;
//	}
	
	Chassis_Speed_Target[LEFT_FRON_201] = limitVal<float>( Chassis_Speed_Target[LEFT_FRON_201], -speed_max, speed_max);
	Chassis_Speed_Target[RIGH_FRON_202] = limitVal<float>( Chassis_Speed_Target[RIGH_FRON_202], -speed_max, speed_max);
	Chassis_Speed_Target[LEFT_BACK_203] = limitVal<float>( Chassis_Speed_Target[LEFT_BACK_203], -speed_max, speed_max);
	Chassis_Speed_Target[RIGH_BACK_204] = limitVal<float>( Chassis_Speed_Target[RIGH_BACK_204], -speed_max, speed_max);

	if(actChassis == CHASSIS_SZUPUP)//����ǰ�ֻ�򻬣������޵ñȺ���ҪС
	{
		//ǰ���޷�����ʵ����������������ֲ��ù�
		Chassis_Speed_Target[LEFT_FRON_201] = limitVal<float>( Chassis_Speed_Target[LEFT_FRON_201], -STANDARD_MAX_SZUPUP, STANDARD_MAX_SZUPUP);
		Chassis_Speed_Target[RIGH_FRON_202] = limitVal<float>( Chassis_Speed_Target[RIGH_FRON_202], -STANDARD_MAX_SZUPUP, STANDARD_MAX_SZUPUP);
		Chassis_Speed_Target[LEFT_BACK_203] = limitVal<float>( Chassis_Speed_Target[LEFT_BACK_203], -Omni_Speed_Max, Omni_Speed_Max);
		Chassis_Speed_Target[RIGH_BACK_204] = limitVal<float>( Chassis_Speed_Target[RIGH_BACK_204], -Omni_Speed_Max, Omni_Speed_Max);
	}
}

/**
  * @brief  ���̵��PID����,����
  * @param  ���ID
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�)
  */
void Chassis_Motor_Speed_PID( eChassisWheel eWheel ) 
{
	//�����ٶ����
	Chassis_Speed_Error[eWheel] = Chassis_Speed_Target[eWheel] - Chassis_Speed_Measure[eWheel];
//	Chassis_Speed_Error[eWheel] = KalmanFilter(&Chassis_Speed_Kalman[eWheel], Chassis_Speed_Error[eWheel]);
	Chassis_Speed_Error_Sum[eWheel] += Chassis_Speed_Error[eWheel];
	
	pTermChassis[eWheel] =  Chassis_Speed_Error[eWheel]*Chassis_Speed_kpid[eWheel][KP];
	iTermChassis[eWheel] =  Chassis_Speed_Error_Sum[eWheel]*Chassis_Speed_kpid[eWheel][KI] * 0.002f;
	//�����޷�
	iTermChassis[eWheel] = limitVal<float>(iTermChassis[eWheel],-iTermChassis_Max,iTermChassis_Max);
	
	Chassis_Speed_Error_NOW[eWheel] = Chassis_Speed_Error[eWheel];
	dTermChassis[eWheel] = (Chassis_Speed_Error_NOW[eWheel] - Chassis_Speed_Error_LAST[eWheel])*Chassis_Speed_kpid[eWheel][KD];
	Chassis_Speed_Error_LAST[eWheel] = Chassis_Speed_Error_NOW[eWheel];
	
	//���������С,��ֹ���Ϊ0ʱͻȻʧ��
	if( pTermChassis[eWheel] * iTermChassis[eWheel] < 0 )
	{
		Chassis_Speed_Error_Sum[eWheel] = limitVal<float>(Chassis_Speed_Error_Sum[eWheel],
															-(iTermChassis_Max/Chassis_Speed_kpid[eWheel][KI]/5.f),
															 (iTermChassis_Max/Chassis_Speed_kpid[eWheel][KI]/5.f));
	}
	
	pidTermChassis[eWheel] = pTermChassis[eWheel] + iTermChassis[eWheel] + dTermChassis[eWheel];

	pidTermChassis[eWheel] = limitVal<float>(pidTermChassis[eWheel],-Chassis_Final_Output_Max,Chassis_Final_Output_Max);	
	
	//��¼�������
	Chassis_Final_Output[eWheel] = pidTermChassis[eWheel];
}

/**
  * @brief  �ֱ��4�������PID����,���������������(���͸������ֵ)
  * @param  void
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�)
  */
void Chassis_MotorOutput(void)
{
	Chassis_Motor_Speed_PID(LEFT_FRON_201);
	Chassis_Motor_Speed_PID(RIGH_FRON_202);
	Chassis_Motor_Speed_PID(LEFT_BACK_203);
	Chassis_Motor_Speed_PID(RIGH_BACK_204);
}


float Chassis_Z_Speed_PID(void)
{
	static float error[2];
	
	error[NOW] = Chassis_Gyro_Error;
	
	Chassis_Move_Z = error[NOW]*Chassis_Z_kpid[KP] + Chassis_Z_kpid[KD]*(error[NOW] - error[LAST]);
	Chassis_Move_Z = limitVal<float>(Chassis_Move_Z, -Chassis_Revolve_Move_Max, Chassis_Revolve_Move_Max);

	error[LAST] = error[NOW];
	
	return Chassis_Move_Z;
}


/*****************���̹���*************************/

/**
  * @brief  ���̹�������
  * @param  void
  * @retval void
  * @attention  �ڵ��������������,��Ҫ�Ǳ������㷨,ICRA
  */
void Chassis_Power_Limit(void)
{	
	/*********************�洫�㷨*************************/
	float    kLimit = 0;//��������ϵ��
	float    chassis_totaloutput = 0;//ͳ�����������
	float    Joule_Residue = 0;//ʣ�ཹ����������
	int16_t  judgDataCorrect = 0;//����ϵͳ�����Ƿ����	
	static int32_t judgDataError_Time = 0;
	
	
	judgDataCorrect = JUDGE_sGetDataState();//����ϵͳ�����Ƿ����
	Joule_Residue = JUDGE_fGetRemainEnergy();//ʣ�ཹ������	
	
	//ͳ�Ƶ��������
	chassis_totaloutput = abs(Chassis_Final_Output[0]) + abs(Chassis_Final_Output[1])
							+ abs(Chassis_Final_Output[2]) + abs(Chassis_Final_Output[3]);
	
	if(judgDataCorrect == JUDGE_DATA_ERROR)//����ϵͳ��Чʱǿ������
	{
		judgDataError_Time++;
		if(judgDataError_Time > 100)
		{
			fTotalCurrentLimit = 9000;//��Ϊ����1/4
		}
	}
	else
	{
		judgDataError_Time = 0;
		//ʣ�ཹ������С,��ʼ�������,����ϵ��Ϊƽ����ϵ
		if(Joule_Residue < WARNING_REMAIN_POWER)
		{
			kLimit = (float)(Joule_Residue / WARNING_REMAIN_POWER)
						* (float)(Joule_Residue / WARNING_REMAIN_POWER);
			
			fTotalCurrentLimit = kLimit * fChasCurrentLimit;
		}
		else   //���������ָ���һ����ֵ
		{
			fTotalCurrentLimit = fChasCurrentLimit;
		}
	}

	//���̸�����������·���
	if (chassis_totaloutput > fTotalCurrentLimit)
	{
		Chassis_Final_Output[0] = (int16_t)(Chassis_Final_Output[0] / chassis_totaloutput * fTotalCurrentLimit);
		Chassis_Final_Output[1] = (int16_t)(Chassis_Final_Output[1] / chassis_totaloutput * fTotalCurrentLimit);
		Chassis_Final_Output[2] = (int16_t)(Chassis_Final_Output[2] / chassis_totaloutput * fTotalCurrentLimit);
		Chassis_Final_Output[3] = (int16_t)(Chassis_Final_Output[3] / chassis_totaloutput * fTotalCurrentLimit);	
	}
}


/**************����ģʽ��������********************/

/**
  * @brief  ���̼���б�º���
  * @param  �жϰ����Ƿ񱻰���, ʱ����, ÿ�����ӵ���, һ��Ҫ��С����
  * @retval б�±���ϵ��
  * @attention  0~1
  */
float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec )
{
	float  factor = 0;

	
	factor = 0.15 * sqrt( 0.15 * (*time) );  //�����ٶ�б��,time�ۼӵ�296.3б�¾����
	
	if (status == 1)//����������
	{
		if (factor < 1)//��ֹtime̫��
		{
			*time += inc;
		}
	}
	else		//�����ɿ�
	{
		if (factor > 0)
		{
			*time -= dec;

			if (*time < 0)
			{
				*time = 0;
			}
		}
	}

	factor = limitVal<float>( factor, 0, 1 );//ע��һ����float�����޷�

	return factor;  //ע�ⷽ��
}

/**
  * @brief  ��ȡ�����ƶ�ģʽ
  * @param  void
  * @retval TRUE:��еģʽ    false:������ģʽ
  * @attention  
  */
bool CHASSIS_IfActiveMode(void)
{
	if (modeChassis == CHASSIS_MECH_MODE)
	{
		return TRUE;//��е
	}
	else
	{
		return FALSE;//������
	}
}

/**
  * @brief  �����Ƿ�������ģʽ
  * @param  void
  * @retval TRUE FALSE
  * @attention  
  */
bool Chassis_IfSZUPUP(void)
{
	if(actChassis == CHASSIS_SZUPUP)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  �����Ƿ���Ťƨ��ģʽ
  * @param  void
  * @retval TRUE FALSE
  * @attention  
  */
bool Chassis_IfCORGI(void)
{
	if(actChassis == CHASSIS_CORGI)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  �����Ƿ���45��ģʽ
  * @param  void
  * @retval TRUE FALSE
  * @attention  
  */
bool Chassis_IfPISA(void)
{
	if(actChassis == CHASSIS_PISA)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


/*********************�������ݽӿ�*************************/

/**
  * @brief  ��ȡ����ʵ��ת��
  * @param  void
  * @retval ������ת��
  * @attention  
  */
float Chassis_All_Speed(void)
{
	return ( abs(Chassis_Speed_Measure[0]) + abs(Chassis_Speed_Measure[1])
				 + abs(Chassis_Speed_Measure[2]) + abs(Chassis_Speed_Measure[3]) );
}

/**
  * @brief  ��ȡ����Ŀ��ת��
  * @param  void
  * @retval ������ת��
  * @attention  
  */
float Chassis_All_Speed_Target(void)
{
	return ( abs(Chassis_Speed_Target[0]) + abs(Chassis_Speed_Target[1])
				 + abs(Chassis_Speed_Target[2]) + abs(Chassis_Speed_Target[3]) );
}


/**
  * @brief  ��ȡ�����ƶ�Ŀ��ֵ
  * @param  void
  * @retval ����Ŀ���ƶ���
  * @attention  �����жϵ����Ƿ������ƶ��仯����,�������ݳ����
  */
float Get_Chass_X(void)
{
	return Chassis_Move_X;
}

float Get_Chass_Y(void)
{
	return Chassis_Move_Y;
}

float Get_Chass_Z(void)
{
	return Chassis_Move_Z;
}

/**
  * @brief  Ťƨ��ר�ã�ʱ�䲻��棩
  * @param  ��ǰ��̨ƫ������Ŀ��ƫ����
  * @retval ��ת�ٶ�
  * @attention 
  */
float Chassis_Z_Corgi(float get, float set)
{
	float error[2];
	float z = 0;
	
	error[NOW] = set - get;
	
	z = 15*((error[NOW]*8) + 10*(error[NOW] - error[LAST]));//PD����
	z = limitVal<float>(z, -5000, 5000);

	error[LAST] = error[NOW];
	
	return z;
}

/**
  * @brief  ��ת�ٶ�PID����
  * @param  ��ǰ��̨ƫ������Ŀ��ƫ����
  * @retval ��ת�ٶ�
  * @attention 
  */
float speed_z_pterm = 0;
float speed_z_iterm = 0;
float speed_z_dterm = 0;


float Chassis_SpeedZ_PID(int16_t ErrorReal, float kp)
{
	static int16_t ErrorPrev = 0;
	static int32_t ErrorSum = 0;
	static int32_t ErrorPR = 0;
	static int32_t ErrorPR_KF = 0;
	
	float speed_z = 0;

	ErrorPR_KF = KalmanFilter(&Chassis_Error_Kalman, ErrorReal);
	
	//P
	speed_z_pterm = ErrorReal * kp;
	speed_z_pterm = limitVal<float>(speed_z_pterm, -REVOLVE_MAX_NORMAL, REVOLVE_MAX_NORMAL);
	
	//I
	ErrorSum -= ErrorPR_KF;
	speed_z_iterm = ErrorSum*3*0.002f;
	if( abs(ErrorReal) <= 10)
	{
		ErrorSum = 0;
	}

	speed_z_iterm = limitVal<float>(speed_z_iterm,-5000,5000);
	
	//D
	ErrorPR = ErrorPR_KF - ErrorPrev;
	
	if( abs(ErrorPR_KF) > REVOLVE_ANGLE )
	{
		speed_z_dterm = -(ErrorPR) * REVOLVE_KD;
	}
	else
	{
		speed_z_dterm = 0;
	}

	speed_z = speed_z_pterm + speed_z_dterm;// + speed_z_iterm;//// + ;
	speed_z = limitVal<float>(speed_z, -Chassis_Revolve_Move_Max, +Chassis_Revolve_Move_Max);

	ErrorPrev = ErrorPR_KF;
	
	return speed_z;
}
