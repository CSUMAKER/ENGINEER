#ifndef _USER_CAN_H
#define _USER_CAN_H

#include "makos_includes.h"
#include "timer.h"

extern int16_t mpu_yaw_speed;		//普通mpu6050的原始角速度
extern float imu_yaw;				//hi216的原始角度
extern float real_mpu_hi216_yaw;	//标度转换加连续处理的hi216角度

typedef	struct	_mcircle_t
{
	S32	circle;
	U32	angle;
}mcircle_t, *p_mcircle;

extern mcircle_t	motor_circle[2][8];
extern mcircle_t	can_mpu_yaw;	//标度转换后的hi216角度与圈数

#endif

