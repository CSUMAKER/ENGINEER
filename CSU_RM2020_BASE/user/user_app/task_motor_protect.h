#ifndef _TASK_MOTOR_PROTECT_H
#define _TASK_MOTOR_PROTECT_H

#include "headfile.h"

//���������ڷ�ֹ�����ת�����𻵣�2020��1��12��//

//���̵����ֹ��ת���֣��ĸ����ͳһ����
extern bool chassis_motors;								//Ĭ��ʹ�ܣ�������ת��,������ͬ����׸��
//����̧�����ͳһ����
extern bool lift_motors;
//һ��ץ�����Ƶ��
extern bool lift_x_motors;
//����צ�ӵ��ͳһ����
extern bool claw_motors;
//��Ԯ���ͳһ����
extern bool relief_motors;
//��̨���
extern bool holder_motors;
//������е�۵��ͳһ����
extern bool arm_motors;

typedef struct
{
	u8 cur_claw_value[2];								//��¼צ�ӵ�ǰֵ��������0.5����ֵ�Ƚϣ�����צ����Ҫ�ƶ�����ֵȴ��δ�ı����Ƕ�ת������˼·��ͬ

	u8 cur_lift_x_value;								//��¼���Һ��Ƶ����ǰֵ

	u8 cur_lift_value[2];								//��¼̧�������ǰֵ

	u8 cur_chassis_value[4];							//��¼�����ĸ������ǰֵ
	
}motor_protect_cur_val_t;

extern u8 jam_time;
void task_motor_protect(void *pram);
#endif	
