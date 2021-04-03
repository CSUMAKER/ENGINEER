#ifndef _TASK_ENGINEER_CONTROL_H
#define _TASK_ENGINEER_CONTROL_H

#include "makos_includes.h"
#include "mak_pid.h"

#define RPC_ZERO(IN,RANGE)	((IN < RANGE && IN > -RANGE) ? 0 : IN)

typedef struct
{
	float angle_T;
	
	S32 position_T[4];//Ŀ��λ��
	S32 position_C[4];//ʵ��λ��
	
	float speed_T[4];//Ŀ���ٶ�,���ζ�ӦHL,HR,BL,BR
	float speed_C[4];//ʵ���ٶ�,���ζ�ӦHL,HR,BL,BR
	
	float current_T[4];//Ŀ�����,���ζ�ӦHL,HR,BL,BR
	float current_C[4];//ʵ�ʵ���,���ζ�ӦHL,HR,BL,BR
	
	int16_t CAN_data[4];//CAN��������,���ζ�ӦHL,HR,BL,BR
	
	PID_Absolute_Type pid_position[4];//λ�û�
	PID_Absolute_Type pid_speed[4];//�ٶȻ�
	PID_Increment_Type pid_current[4];//������
	PID_Absolute_Type pid_direction_correct;//·����������ת������ȷ�������õ�pid
	
}chassis_control_t;

typedef struct
{
	S32 position_T[2];//Ŀ��λ��
	S32 position_C[2];//ʵ��λ��
	
	float speed_T[2];//Ŀ���ٶ�
	float speed_C[2];//ʵ���ٶ�
	
	float current_T[2];//Ŀ�����
	float current_C[2];//ʵ�ʵ���
	
	int16_t CAN_data[2];//CAN��������
	
	PID_Absolute_Type pid_position[2];//λ�û�
	PID_Absolute_Type pid_speed[2];//�ٶȻ�
	PID_Increment_Type pid_current[2];//������
	
}lift_control_t;

typedef struct
{
	S32 position_T;//Ŀ��λ��
	S32 position_C;//ʵ��λ��
	
	float speed_T;//Ŀ���ٶ�
	float speed_C;//ʵ���ٶ�
	
	float current_T;//Ŀ�����
	float current_C;//ʵ�ʵ���
	
	int16_t CAN_data;//CAN��������
	
	
	PID_Absolute_Type pid_position;//λ�û�
	PID_Absolute_Type pid_speed;//�ٶȻ�
	PID_Increment_Type pid_current;//������
}lift_x_control_t;

typedef struct
{
	S32 position_T[2];//Ŀ��λ��
	S32 position_C[2];//ʵ��λ��
	
	float speed_T[2];//Ŀ���ٶ�
	float speed_C[2];//ʵ���ٶ�
	
	float current_T[2];//Ŀ�����
	float current_C[2];//ʵ�ʵ���
	
	int16_t CAN_data[2];//CAN��������
	
	PID_Absolute_Type pid_position[2];	//λ�û�
	PID_Absolute_Type pid_speed[2];		//�ٶȻ�
	PID_Increment_Type pid_current[2];	//������
	
}claw_control_t;

typedef struct
{
	void(*clamp_claw)(void);
	void(*loose_calw)(void);
	
	void(*popup_claw)(void);
	void(*back_calw)(void);
	
	void(*popup_supply)(void);
	void(*back_supply)(void);
	
	void(*popup_relief)(void);
	void(*back_relief)(void);
	
}gas_control_t;

typedef struct
{
	S32 position_T[2];//Ŀ��λ��
	S32 position_C[2];//ʵ��λ��
	
	float speed_T[2];//Ŀ���ٶ�
	float speed_C[2];//ʵ���ٶ�
	
	float current_T[2];//Ŀ�����
	float current_C[2];//ʵ�ʵ���
	
	int16_t CAN_data[2];//CAN��������
	
	PID_Absolute_Type pid_position[2];//λ�û�
	PID_Absolute_Type pid_speed[2];//�ٶȻ�
	PID_Increment_Type pid_current[2];//������
	
}relief_control_t;

typedef struct
{
	int angle_T;
	
	S32 position_T;//Ŀ��λ��
	S32 position_C;//ʵ��λ��
	
	float speed_T;//Ŀ���ٶ�
	float speed_C;//ʵ���ٶ�
	
	float current_T;//Ŀ�����
	float current_C;//ʵ�ʵ���
	
	int16_t CAN_data;//CAN��������
	
	PID_Absolute_Type pid_position;//λ�û�
	PID_Increment_Type pid_speed;//�ٶȻ�
	PID_Increment_Type pid_current;//������
	
	bool holder_control_status;
}holder_control_t;

typedef struct
{
	S32 position_T[2];//Ŀ��λ��
	S32 position_C[2];//ʵ��λ��
	
	float speed_T[2];//Ŀ���ٶ�
	float speed_C[2];//ʵ���ٶ�
	
	float current_T[2];//Ŀ�����
	float current_C[2];//ʵ�ʵ���
	
	int16_t CAN_data[2];//CAN��������
	
	PID_Absolute_Type pid_position[2];//λ�û�
	PID_Absolute_Type pid_speed[2];//�ٶȻ�
	PID_Increment_Type pid_current[2];//������
	
}arm_control_t;

typedef struct
{
	S32 position_T;//Ŀ��λ��
	S32 position_C;//ʵ��λ��
	
	float speed_T;//Ŀ���ٶ�
	float speed_C;//ʵ���ٶ�
	
	float current_T;//Ŀ�����
	float current_C;//ʵ�ʵ���
	
	int16_t CAN_data;//CAN��������
	
	PID_Absolute_Type pid_position;//λ�û�
	PID_Absolute_Type pid_speed;//�ٶȻ�
	PID_Increment_Type pid_current;//������
	
}arm_x_control_t;


typedef struct
{
	chassis_control_t chassis;   //����
	lift_control_t lift;         //����̧��
	claw_control_t claw;         //һ��̧��
	lift_x_control_t lift_x;     //Ƥ��
	relief_control_t relief;     //��Ԯ
	holder_control_t holder;		 //��̨
	arm_control_t arm;					 //һ����е�ۣ��������
	arm_x_control_t arm_x;       //������е�ۣ�һ�����
	gas_control_t gas;           //
	
}engineer_control_t;
extern engineer_control_t engineer_control;

void engineer_chassis_speed_T(float	vy,float vx,float vr);
void air_pump_on(void);
void air_pump_off(void);
void back_arm(void);
void out_arm(void);
void task_engineer_control(void* param);

#endif

