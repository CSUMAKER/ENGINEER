#ifndef _TASK_MPU_H
#define _TASK_MPU_H

#include "headfile.h"

typedef	struct	_atmosphere_data_t
{
	float gyro_z;
	float yaw;
}imu_data_t, *p_imu_data;

extern imu_data_t IMU_data;

void	task_mpu(void* param);

#endif
