/**
  ****************************(C)                 ****************************
  * @file       task_holder.c/h
  * @brief      云台控制任务应用层，完成pitch，yaw两个自由度的控制
  *				硬件平台为STM32F405RGT6
  * @note       针对不同机器人种类，采用不同的控制策略
					步兵：两个6020	 英雄：两个6623   无人机：两个3510  哨兵：3510+6020
				1. 使用陀螺仪做速度反馈时，切记配置MPU6050的MPU_LOW_Pass为0不然滤波延迟会导致控制比较难做
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2019-1-26       L.Z.M           1.基本完成步兵云台控制（两个6020，暂时两个环都使用电调信息为反馈）
  *                                             2.基本完成3510俯仰控制（单独3510，位置环用电调角度，速度环用陀螺仪反馈）
  *	 V1.0.1		2019-3-3		L.Z.M			增加哨兵控制（6020偏航+3510俯仰）（陀螺仪的软件IIC暂时使用自己的）			
  *  
  *  V2.0.0		2019-9-11		L.Z.M			1.初步精简程序，此程序从此只用作步兵
  *												2.云台的yaw控制采用增量叠加
  *  V2.0.1		2019-9-12		L.Z.M			增加陀螺仪位置选择（云台或底盘）
  * @copyright    CSU RM. All rights reserved.
  ==============================================================================
  */
#include "headfile.h"
#include "task_holder.h"
#include "task_chassis.h"
#include "task_remote.h"
#include "holder_pid.h"
#include "pos_ctrl.h"
#include "task_vision.h"
#include "correct_ctrl.h"
#include "task_mpu.h"

U8 Soilder_ID;
//*********特殊点宏定义**********///
#define	SOLDIER1_PITCH_HIGH				25000		//PITCH电机限位参数
#define SOLDIER1_PITCH_LOW				22000
#define SOLDIER1_PITCH_MID				24188
#define	SOLDIER2_PITCH_HIGH				25800		//PITCH电机限位参数
#define SOLDIER2_PITCH_LOW				22700
#define SOLDIER2_PITCH_MID				25050
//******************************//

//电机电调反馈量
float	holder_yaw_angle,holder_pitch_angle;			//编码器反馈的偏航角度和俯仰角度（已经单位转换和连续化）
int16_t pitch_torque,yaw_torque,pitch_speed,yaw_speed;	//编码器反馈的扭矩电流和转速（带单位转换）

extern S16 camera_num;//视觉更新帧率（自增值）
u8 camera_max_num;//视觉更新帧率（检测值）
//控制用标志位和变量
u8 Vision_Flag = 0,MyScope_Flag = 0;
float	holder_x_target,holder_x_target2,holder_scope_target;	
float	holder_y_target,holder_y_target2;
S16 	holder_x_init,holder_y_init;
#define	ALPHA	0.2	//低通滤波参数
//pid输出值
float pitch_ex_v,yaw_ex_v;
int32_t yaw_output,pitch_output;
int32_t yaw_output_last,pitch_output_last;
//can消息
CanTxMsg	Holder_Motor_set_data;

u8 Send_Flag = 0;//是否发送波形
#define CMD_WAVE 3	//山外上位机对应的波形指令
/**
  * @brief  波形发送函数任务.
  * @param  none.
  * @note   对应山外上位机.
  * @retval 无.
  */
extern float Dir_angle_adj;
extern u8 Target_Flag,holder_fixed_flag;
extern float x_tar_base,x_tar_base_last,y_tar_base,a_moment_x;
extern float yaw_new2020,pitch_new2020,dist_new2020;

void 	task_holder_sendwave(void* param)
{
	uint8_t cmdf[2] = {CMD_WAVE, ~CMD_WAVE};
	uint8_t cmdr[2] = {~CMD_WAVE, CMD_WAVE};
	S32 DataInfo[8];
	while(1)
	{
		//控制
		DataInfo[0]	= (S32)(holder_x_target);  
		DataInfo[1] = (S32)0x00;
		DataInfo[2] = (S32)0x00;
		DataInfo[3]	= (S32)(holder_yaw_angle);
		DataInfo[4]	= (S32)(10 * imu_yaw);
		DataInfo[5]	= (S32)(real_mpu_hi216_yaw);
		DataInfo[6] = (S32)yaw_ex_v;
		DataInfo[7]	= (S32)(yaw_speed);
		
		//视觉 - x
//		DataInfo[0]	= (S32)(100 * yaw_new2020);  //像素偏移
//		DataInfo[1] = (S32)(100 * pitch_new2020);			//原始模型值
//		DataInfo[2] = (S32)(dist_new2020);		//视觉预测速度
//		DataInfo[3]	= (S32)(rm_mpu_real_yaw);	//矫正模型值
//		DataInfo[4]	= (S32)camera_max_num;	//实际位置
//		DataInfo[5]	= (S32)Dir_angle;
//		DataInfo[6] = (S32)CAN_data[0];
//		DataInfo[7]	= (S32)CAN_data[1];    //丢帧信号
		
//		//视觉 - y
//		DataInfo[0]	= (S16)(Camera_Y_offset);  //像素偏移
//		DataInfo[1] = (S16)y_tar_base;			//原始模型值
//		DataInfo[2] = (S16)vision_y_speed;		//视觉预测速度
//		DataInfo[3]	= (S16)(holder_y_target);	//矫正模型值
//		DataInfo[4]	= (S16)holder_pitch_angle;	//实际位置
//		DataInfo[5]	= (S16)(holder_pitch_angle);
//		DataInfo[6] = (S16)holder_y_target;
//		DataInfo[7]	= (S16)(Target_Flag);    //丢帧信号		
//						
		usart2_send_string(cmdf, sizeof(cmdf));    
		
		usart2_send_string((uint8_t *)&DataInfo[0],4);
		usart2_send_string((uint8_t *)&DataInfo[1],4);
		usart2_send_string((uint8_t *)&DataInfo[2],4);
		usart2_send_string((uint8_t *)&DataInfo[3],4);
		usart2_send_string((uint8_t *)&DataInfo[4],4);
		usart2_send_string((uint8_t *)&DataInfo[5],4);
		usart2_send_string((uint8_t *)&DataInfo[6],4);
		usart2_send_string((uint8_t *)&DataInfo[7],4);
		
		usart2_send_string(cmdr, sizeof(cmdr)); 
		
		task_delay_ms(10);
	}
}
/**
  * @brief  云台部分数据的初始化
  * @param  yaw，pitch方向的比例系数
  * @note   .
  * @retval none.
  */
void	holder_data_init(float* Remote_x,float* Remote_y)
{
	Holder_Motor_set_data_init(&Holder_Motor_set_data);

	Dir_angle_adj = 24;
	holder_y_init = SOLDIER1_PITCH_MID + 800;
	
	holder_x_init = real_mpu_hi216_yaw;
	holder_y_target  = holder_y_init;
	holder_x_target  = holder_x_init;
	*Remote_y = -1.85;
	*Remote_x = -0.18;
	
}

/**
  * @brief  目标值保护，防止发生意外
  * @param  
  * @note   .
  * @retval none.
  */
void	holder_target_protect(void)
{
	if(Soilder_ID == 1)
		holder_y_target = SOLDIER_LIMIT(holder_y_target,SOLDIER1_PITCH_LOW,SOLDIER1_PITCH_HIGH);
	else
		holder_y_target = SOLDIER_LIMIT(holder_y_target,SOLDIER2_PITCH_LOW,SOLDIER2_PITCH_HIGH);
			
	if(holder_pitch_angle < 100)	//电机还没有供电
		holder_y_target = holder_pitch_angle;		//防止没供电的时候PID计算
	
//	if(!MyScope_Flag)
//	{
//		if(holder_x_target - holder_yaw_angle > 419)
//			while(holder_x_target - holder_yaw_angle > 419)
//				holder_x_target -= 819;
//		else if	(holder_x_target - holder_yaw_angle < -419)
//			while(holder_x_target - holder_yaw_angle < -419)
//				holder_x_target += 819;
//	}
}

void	task_holder_control(void* param);

void	task_holder(void* param)
{
	float	 Remote_PY,Remote_PX;			//修正遥控器的范围到云台实际可以转动的范围
	p_remote_data data = NULL;
	p_holder_ctrl holder_ctrl_data = mymalloc(SRAMIN,sizeof(holder_ctrl_t));
	
	task_delay_ms(1500);

	holder_data_init(&Remote_PX,&Remote_PY);
	
	task_delay_ms(1500);

	task_insert(task_holder_control, NULL, 1);
	float last_x_tar = holder_x_init;
	
	while(1)
	{
		data = msg_get_read_some(&remote_msg);
		if(data && !Vision_Flag)
		{
			if(data->SL == RP_S_DOWN)		//表示是电脑控制状态
			{
				s32 temp_mouse_x = data->MX * 5.5f;
				s32 temp_mouse_y = data->MY * 3.2f;

				holder_ctrl_data->mouse_x = SOLDIER_LIMIT(temp_mouse_x,-1000,1000);
				holder_ctrl_data->mouse_y = SOLDIER_LIMIT(temp_mouse_y,-1000,1000);
			}
			else							//遥控器是数据
			{
				s32 temp = (data->JR_LR - 1024)*0.9f;
				holder_ctrl_data->mouse_x = SOLDIER_RPC_ZERO(temp,5);
				
				temp = (data->JR_UD - 1024) * 1.5f;
				holder_ctrl_data->mouse_y = SOLDIER_RPC_ZERO(temp,5);
			}

			holder_y_target = holder_y_target - 0.13f * holder_ctrl_data->mouse_y;
			holder_x_target = holder_x_target - 0.03f * holder_ctrl_data->mouse_x;
			
			//进行一次简单的低通滤波
			holder_x_target = 0.5f * holder_x_target + 0.5f * last_x_tar;
			last_x_tar = holder_x_target;
			
			msg_read_finish_some(&remote_msg);
		}
		task_delay_ms(5);
	}
}

PID_AbsoluteType_holder	pid_holder_x_speed;
PID_IncrementType	pid_holder_y_speed;
PID_AbsoluteType_section	pid_holder_y_position_section;
PID_AbsoluteType_section	pid_holder_x_position_section;
float yaw_speed_mpu;

void	task_holder_control(void* param)
{
	if(Send_Flag)
		task_insert((task_f)task_holder_sendwave,NULL,3);
	task_delay_ms(20);
	
	holder_pid_init_absolute_section(&pid_holder_x_position_section, 28,28, 28, 0.00, 10, 0,\
																				20, 50, 100, 3, 160, 2800 );
	holder_pid_init_absolute_section(&pid_holder_y_position_section, 1.4, 1.15, 0.8 , 0.089, 0.3, 300,\
																				50, 120, 300, 10,30000 , 600 );
	holder_pid_init_absolute(&pid_holder_x_speed, 1,  0.2,  1,  1500);		//绝对式 36 0.36
	holder_pid_init_increment(&pid_holder_y_speed, 16, 0.2, 0.0, 1800, 8000);
	
	while(1)
	{			
		holder_target_protect();
		static int count_ticks = 0,camera_count_ticks = 0;
		count_ticks ++;		
		camera_count_ticks ++;
		if(camera_count_ticks == 500)//1s检测一次视觉帧率
		{
			camera_count_ticks = 0;
			camera_max_num = camera_num;
			camera_num = 0;
		}
		if(count_ticks == 5)		//位置环控制周期10ms,速度环2ms
		{ 
			count_ticks = 0;

			yaw_ex_v = PID_Update_Yaw_index(holder_x_target,real_mpu_hi216_yaw,&pid_holder_x_position_section);
			pitch_ex_v = PID_Update_Yaw_index(holder_y_target, (float)holder_pitch_angle, &pid_holder_y_position_section);
		}
		pitch_output = (S32)PID_IncrementMode_Yaw((float)pitch_ex_v,pitch_speed, &pid_holder_y_speed);		
		long int yaw_out_temp;
		yaw_speed_mpu = -(float)mpu_yaw_speed/10;
		yaw_out_temp = PID_Update_Yaw(yaw_ex_v,yaw_speed_mpu,&pid_holder_x_speed);	
		yaw_out_temp *= 35;
		yaw_output = SOLDIER_LIMIT(yaw_out_temp,-32767,32767);
		
		//输出的低通滤波
		pitch_output = pitch_output*ALPHA + pitch_output_last * (1-ALPHA);
		yaw_output = yaw_output*ALPHA + yaw_output_last * (1-ALPHA);
		pitch_output_last = pitch_output;
		yaw_output_last = yaw_output;
		
		yaw_output = 0;
		pitch_output = 0;   
		Holder_Motor_set_yaw_pitch(yaw_output , pitch_output);

		task_delay_ms(2);
	}
}

/**
  * @brief  云台电机CAN结构体初始化.
  * @param  结构体的地址
  * @note   
  * @retval void
  */
void	Holder_Motor_set_data_init(CanTxMsg* data)
{
        data->StdId	= 0x1ff;
        data->IDE	= CAN_ID_STD;
        data->RTR	= CAN_RTR_DATA;
        data->DLC	= 8;
        data->Data[0]	= 0x00;
        data->Data[1]	= 0x00;
        data->Data[2]	= 0x00;
        data->Data[3]	= 0x00;
        data->Data[4]	= 0x00;
        data->Data[5]	= 0x00;
        data->Data[6]	= 0x00;
        data->Data[7]	= 0x00;
}

/**
  * @brief  向电调ID为205和206的电机发送信息
  * @param  ID205信息，ID206信息
  * @note   注意两个信息的顺序
  * @retval 成功则返回1
  */
U32	Holder_Motor_set_yaw_pitch(S16 yaw, S16 pitch)
{
	pitch = SOLDIER_LIMIT(pitch,-32767,32767);
	yaw = SOLDIER_LIMIT(yaw,-32767,32767);
	Holder_Motor_set_data.Data[0] = (U8)(yaw>>8);
	Holder_Motor_set_data.Data[1] = (U8)(yaw&0xff);
	Holder_Motor_set_data.Data[2] = (U8)(pitch>>8);
	Holder_Motor_set_data.Data[3] = (U8)(pitch&0xff);
	
	CAN_Transmit(CAN2, &Holder_Motor_set_data);
	return	1;
}
