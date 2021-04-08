#ifndef _TASK_ENGINEER_STATE_H
#define _TASK_ENGINEER_STATE_H

#include "makos_includes.h"

typedef enum
{
	stop=1,	//ֹͣ
	follow,	//����
	catwalk	//è��
	
}chassis_mode_t;

typedef enum
{
	handle=1,	//�ֱ�
	keyboard	//����
	
}remote_mode_t;

typedef enum
{
	RM3508=1,
	RM3510     //����ʱʹ��3510������ʹ��3508�����ڿ���ʱ�����ȼ�����������ߴ����������
	
}motor_type_t;

typedef struct
{
	S16 remote_JL_UD;
	S16 remote_JL_LR;
	S16 remote_JR_UD;
	S16 remote_JR_LR;
	
	s16 remote_LL;
	
	u8 remote_SL;
	u8 remote_SR;
	
}engineer_remote_t;

typedef struct
{
	chassis_mode_t mode;
	
	motor_type_t motor;  //�������
	
	float move_X_SPD;//�����ٶ� ��Ϊ��
	float move_Y_SPD;//ǰ���ٶ� ǰΪ��
	float move_Z_SPD;//��ת�ٶ� ˳ʱ��Ϊ��
	
	S32 YAW;//·������ʹ�õ�ƫ���Ǽ�¼��
	
	bool is_direction_control; //�Ƿ���·��������0 ������1 �ر�
	
}engineer_chassis_t;

typedef struct
{
	float claw_angle;	//���ӽǶ�
	float holder;   //��̨
	bool is_claw_clamp;//�����Ƿ�ӽ�
	
	float lift_height;	//̧���ܸ߶�
	float lift_x_dis;	//��������
	bool lift_Y_stick;	//�����Ƿ�ǰ��
	
	float arm_angle;		//
	float arm_x_angle;	//
	float temp_arm_angle;		//
	float temp_arm_x_angle;
}engineer_take_bullet_t;

typedef struct
{
	bool relief_finger;	//��Ԯ��ָ
	float relief_frame;	//��Ԯ��
	short AddHPCard;	//��Ѫ��
	
}engineer_relief_t;

typedef struct
{
	bool laser_switch_left;		//��༤�⿪��
	bool laser_switch_right;	//�Ҳ༤�⿪��
	bool laser_switch_mid;	//�м伤�⿪��
	S8 catch_ultrasound;		//צ�ӳ�����
	S8 chassis_ultrasound[4];	//���̳�����
	
}engineer_sensors_t;

typedef struct
{
	remote_mode_t remote_mode;		//ң�ؿ���ģʽ
	
	engineer_take_bullet_t bullet;	//ȡ��
	
	engineer_relief_t relief;		//��Ԯ
	
	engineer_chassis_t chassis;		//����
	
	engineer_sensors_t sensors;		//������
	
	bool is_supplement;				//��������
	
}engineer_state_t;
extern engineer_state_t engineer;
extern engineer_remote_t remote_origin;//�ݴ�ң��������

extern bool badass;					//�������жϵ�ҩ���Ƿ�����ǰ���ı���
extern int start_location;	//�������ƶ��Ŀ�ʼֵ���˳�һ��ץ����Ļظ���
extern int left_limits;		//���������˳���ʱ�����޷�ֵ,������˵�Ǹ���
extern int right_limits;	//���������˳���ʱ�����޷�ֵ

bool ammunition_box_towards(void);	//�������жϵ�ҩ���Ƿ�����ǰ���ĺ�������⶯����
void task_engineer_state(void* param);

#endif

