#include "task_mpu.h"
#include "mpu.h"
#include "can.h"
#include "imu_packet.h"
#include "imu_data_decode.h"
#include "PWM.h"

//全局变量用户接口
imu_data_t IMU_data;

static float mpu_gyro_z_offest;	//陀螺仪零飘补偿
void gyro_offest(void)
{
	int i;
	static float last_offest_gyro;
	retry:
	for(i=0;i<300;i++)
	{
		READ_MPU6500_GYRO();
		if(fabs(last_offest_gyro-mpu_value.Gyro[2])<1500)
		{
			mpu_gyro_z_offest+=mpu_value.Gyro[2];
			last_offest_gyro=mpu_value.Gyro[2];
		}
		else
		{
			mpu_gyro_z_offest=0;
			last_offest_gyro=0;
			goto retry;
		}
		task_delay_ms(1);
	}
	mpu_gyro_z_offest/=300;
}

//void	task_test(void* param);
//u8 test_can[8];

float Eular[3];		//串口陀螺仪反馈的欧拉角
void	task_mpu(void* param)
{
	
//	task_insert(task_test, NULL, 1);
	
	Init_MPU6500();
	gyro_offest();
	while(1)
	{
		//获取数据
		get_eular(Eular);
		READ_MPU6500_GYRO();
		mpu_value.Gyro[2]-=mpu_gyro_z_offest;
		
		IMU_data.gyro_z=mpu_value.Gyro[2];
		IMU_data.yaw=Eular[2];
		
//		print_wave(2,4,&IMU_data.gyro_z,&IMU_data.yaw);
		//延时
		task_delay_ms(5);
	}
}

//void	task_test(void* param)
//{
//	PWM_init();
//	while(1)
//	{
//		test_can[0]=0xff;
//		test_can[7]=0xff;
//		CAN2_SendMsg(0x201,test_can);
//
//		TIM1->CCR2 = 1540;
//		task_delay_ms(1000);
//		TIM1->CCR2 = 1740;
//		task_delay_ms(1000);
//	}
//}

