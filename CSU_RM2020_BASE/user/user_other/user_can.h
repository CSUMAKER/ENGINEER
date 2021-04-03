#ifndef _USER_CAN_H
#define _USER_CAN_H

#include "makos_includes.h"
#include "timer.h"

extern int16_t mpu_yaw_speed;		//��ͨmpu6050��ԭʼ���ٶ�
extern float imu_yaw;				//hi216��ԭʼ�Ƕ�
extern float real_mpu_hi216_yaw;	//���ת�������������hi216�Ƕ�

typedef	struct	_mcircle_t
{
	S32	circle;
	U32	angle;
}mcircle_t, *p_mcircle;

extern mcircle_t	motor_circle[2][8];
extern mcircle_t	can_mpu_yaw;	//���ת�����hi216�Ƕ���Ȧ��

#endif

