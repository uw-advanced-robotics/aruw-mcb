#ifndef _TASK_CHASSIS_H
#define _TASK_CHASSIS_H

#include <rm-dev-board-a/board.hpp>

#define KP 0
#define KI 1
#define KD 2

//����ģʽѡ��
typedef enum
{
	CHASSIS_MECH_MODE = 0,//��е
	CHASSIS_GYRO_MODE = 1,//������,���̸�����̨
	
} eChassisCtrlMode;


//����ģʽʱ,����״̬ѡ��
typedef enum
{
	CHASSIS_NORMAL   = 0,//��ͨģʽ,����ǰ��
	CHASSIS_CORGI    = 1,//Ťƨ��ģʽ
	CHASSIS_ROSHAN   = 2,//���ģʽ
	CHASSIS_SLOW     = 3,//��������ģʽ
	CHASSIS_SZUPUP   = 4,//����ģʽ
	CHASSIS_MISS     = 5,//�Զ�����ģʽ
	CHASSIS_PISA     = 6,//45��ģʽ
	
} eChassisAction;

//���̵��ID
typedef enum
{
	LEFT_FRON_201 = 0,  // ��ǰ
	RIGH_FRON_202 = 1,  // ��ǰ
	LEFT_BACK_203 = 2,  // ���
	RIGH_BACK_204 = 3,  // �Һ�
	
}eChassisWheel;

//ת�ٲ���
typedef enum
{
	Chassis_ANGLE = 0,  // ��е�Ƕ�
	Chassis_SPEED = 1,  // ת��
	
}eChassis_MotorReturnValueType;


void CHASSIS_InitArgument(void);
void CHASSIS_StopMotor(void);
void CHASSIS_REST(void);

/************************�����ܿ���,loop�е���***************/
void Task_Chassis(void *pvParameters);
void CHAS_Rc_Ctrl(void);
void CHAS_Key_Ctrl(void);

/*******************���̼���ģʽ����ģʽС����*******************/
void Chassis_Keyboard_Move_Calculate( int16_t sMoveMax, int16_t sMoveRamp );
void Chassis_Mouse_Move_Calculate( int16_t sRevolMax );
void Chassis_NORMAL_Mode_Ctrl(void);
void CHASSIS_CORGI_Mode_Ctrl(int16_t sRevolMax, int16_t sRevolRamp);
void CHASSIS_CORGI_Mode_Ctrl_Time(int16_t sRevolMax, int16_t sRevolRamp);
void CHASSIS_SZUPUP_Mode_Ctrl(void);
void CHASSIS_MISS_Mode_Ctrl(void);
void CHASSIS_PISA_Mode_Ctrl(void);

/********************���̵�����ݸ��¼�����*****************************/
void CHASSIS_UpdateMotorAngle( eChassisWheel eWheel, int16_t angle );
void CHASSIS_UpdateMotorSpeed( eChassisWheel eWheel, int16_t speed );
void CHASSIS_UpdateMotorCur( eChassisWheel eWheel, int16_t current );

void CHASSIS_CANbusCtrlMotor(void);

/**********************�����������**************************/
void Chassis_Omni_Move_Calculate(void);
void Chassis_Motor_Speed_PID( eChassisWheel eWheel );
void Chassis_MotorOutput(void);
float Chassis_Z_Speed_PID(void);

/*****************���̹���*************************/
void Chassis_Power_Limit(void);

/**************����ģʽ��������********************/
float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec );
bool CHASSIS_IfActiveMode(void);

bool Chassis_IfSZUPUP(void);
bool Chassis_IfCORGI(void);
bool Chassis_IfPISA(void);

/************�������ݽӿ�***************/

float Chassis_All_Speed(void);
float Chassis_All_Speed_Target(void);

float Get_Chass_X(void);
float Get_Chass_Y(void);
float Get_Chass_Z(void);

float Chassis_Z_Corgi(float get, float set);
float Chassis_SpeedZ_PID(int16_t ErrorReal, float kp);

#endif