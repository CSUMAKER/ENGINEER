#include "task_engineer_state.h"
#include "task_engineer_control.h"
#include "headfile.h"
#include "task_sensors.h"
#include "task_motor_protect.h"
#include "task_cancel_auto_collect.h"

//传感器变量
//engineer.sensors.laser_switch_left	//左激光
//engineer.sensors.laser_switch_right	//右激光
//catch_ultrasound						//中间超声波

engineer_state_t engineer;		 //工程车目标值
engineer_remote_t remote_origin; //暂存遥控器数据
								 //添加堵转保护任务（建立控制变量）
void task_print(void *param);
void chassis_keybord_mode(engineer_chassis_t *robot);
void chassis_handle_mode(engineer_chassis_t *robot);
void lift_handle_mode(void);
void lift_keybord_mode(void);
bool ammunition_box_towards(void);
bool badass;

void task_engineer_state(void *param)
{
	task_insert_CCM(task_print, NULL, 1);
	//	task_insert_CCM(task_sensors, NULL, 2);
	p_remote_data data = NULL;
	engineer.chassis.motor = RM3508; //先指定电机种类以方便发挥接下来的代码功能

	while (1)
	{
		data = msg_get_read_some(&remote_msg);
		if (data != NULL)
		{
			//保存遥控器原始控制信号
			remote_origin.remote_JL_LR = RPC_ZERO(((s16)data->JL_LR - 1024), 10);
			remote_origin.remote_JL_UD = RPC_ZERO(((s16)data->JL_UD - 1024), 10);
			remote_origin.remote_JR_LR = RPC_ZERO(((s16)data->JR_LR - 1024), 10);
			remote_origin.remote_JR_UD = RPC_ZERO(((s16)data->JR_UD - 1024), 10);
			remote_origin.remote_LL = (s16)data->LL - 1024;
			remote_origin.remote_SR = data->SR;
			remote_origin.remote_SL = data->SL;
			//判断控制模式————遥控器/键鼠
			switch (data->SL)
			{
			case RP_S_UP:
				engineer.remote_mode = handle;
				engineer.chassis.mode = catwalk;
				break;
			case RP_S_MID:
			{
				engineer.remote_mode = handle;
				engineer.chassis.mode = follow;
				if (lift_motors == 0)
				{
					chassis_motors = 1;
					lift_motors = 1;
					lift_x_motors = 1;
					claw_motors = 1;
					relief_motors = 1;
					holder_motors = 1;
					arm_motors = 1;
				}
			}
			break;
			case RP_S_DOWN:
				engineer.remote_mode = keyboard;
				chassis_motors = 0;
				lift_motors = 0;
				lift_x_motors = 0;
				claw_motors = 0;
				relief_motors = 0;
				holder_motors = 0;
				arm_motors = 0;
				break;
			default:
				engineer.remote_mode = handle;
				engineer.chassis.mode = stop;
				break;
			}
			//针对不同控制模式分别进行目标值给定
			//
			switch (engineer.remote_mode)
			{
			case handle:
				chassis_handle_mode(&engineer.chassis);
				lift_handle_mode();
				lift_keybord_mode();
				break;
			case keyboard:
				chassis_keybord_mode(&engineer.chassis);
				lift_keybord_mode();
				break;
			}
		}
		else
		{
			engineer.chassis.mode = stop;
		}
		task_delay_ms(1);
	}
}

void chassis_handle_mode(engineer_chassis_t *robot)
{
	switch (robot->mode)
	{
	case follow:
		engineer.chassis.move_X_SPD = remote_origin.remote_JL_LR;
		engineer.chassis.move_Y_SPD = remote_origin.remote_JL_UD;
		engineer.chassis.move_Z_SPD = remote_origin.remote_JR_LR;
		engineer.bullet.holder += remote_origin.remote_JR_UD * 1;
		break;
	case catwalk:
		engineer.chassis.move_X_SPD = 0;
		engineer.chassis.move_Y_SPD = 0;
		engineer.chassis.move_Z_SPD = 0;
		engineer.relief.relief_frame = 0;
		break;
	case stop:
		/*
			
			*/
		break;
	default:
		engineer.chassis.move_X_SPD = 0;
		engineer.chassis.move_Y_SPD = 0;
		engineer.chassis.move_Z_SPD = 0;

		break;
	}
	engineer_chassis_speed_T(engineer.chassis.move_Y_SPD, engineer.chassis.move_X_SPD, engineer.chassis.move_Z_SPD);
}

void chassis_keybord_mode(engineer_chassis_t *robot)
{
}

void lift_handle_mode(void)
{
	static u8 last_sr = 4;
	static S16 last_ll = 0;
	static u8 f_last_sr = 4;
	static S16 f_last_ll = 0;
	if (remote_origin.remote_LL > 250) //右开关由中间拨到上面控制爪子的抓取合放
	{
		TIM_SetCompare1(TIM4, 9600);
		TIM_SetCompare2(TIM4, 9600);
	}
	else
	{
		TIM_SetCompare1(TIM4, 10350);
		TIM_SetCompare2(TIM4, 10350);
	}
	if (engineer.chassis.mode == catwalk)
	{
		//以下控制为位置环控制
		engineer.bullet.lift_height += remote_origin.remote_JL_UD * 1; //左摇杆上下控制抬升高度（半自动
																	   //此处角度的最大值应该改成宏定义
		engineer.bullet.lift_x_dis += remote_origin.remote_JR_LR * 1;  //抬升左右横移（半自动
																	   //					//进一步测试再改正限幅，默认中间位置环为0
		engineer.bullet.claw_angle += remote_origin.remote_JR_UD * 1;  //右摇杆爪子的旋转角（半自动

		//		engineer.bullet.arm_angle+=remote_origin.remote_JL_UD*0.1;

		//		engineer.bullet.arm_x_angle+=remote_origin.remote_JL_UD*0.5;

		last_sr = remote_origin.remote_SR;
		last_ll = remote_origin.remote_LL;
	}
	if (engineer.chassis.mode == follow)
	{

		//		engineer.relief.relief_frame+=remote_origin.remote_JR_UD*1;

		//		if(remote_origin.remote_SR==RP_S_UP)
		//		{
		//
		//			for(int i=0;i<4;i++)
		//			{
		//				engineer_control.chassis.position_T[i]+=600;
		//			}
		//			engineer.bullet.holder=-1900;
		//		}

		if (remote_origin.remote_SR == RP_S_MID && f_last_sr == RP_S_UP) //右开关由中间拨到下面控制救援
		{
			//			engineer.bullet.holder+=8192-engineer.bullet.holder;
		}

		if (remote_origin.remote_SR == RP_S_DOWN && f_last_sr == RP_S_MID) //右开关由中间拨到下面控制救援
		{
			engineer.relief.relief_frame += 76800;
		}

		if (remote_origin.remote_SR == RP_S_MID && f_last_sr == RP_S_DOWN)
		{
			engineer.relief.relief_frame -= 76800;
		}
		f_last_sr = remote_origin.remote_SR;
		f_last_ll = remote_origin.remote_LL;
	}
}

//传感器变量
//engineer.sensors.laser_switch_left	//左激光
//engineer.sensors.laser_switch_right	//右激光
//engineer.sensors.catch_ultrasound		//中间超声波
//利用传感器实现的一将自动抓弹

int start_location = 110000;   //上升后移动的开始值，退出一键抓弹后的回复处
int left_limits = -540000;	   //贴着左铁杆出发时的左限幅值,正常来说是负的
int right_limits = 1120000;	   //贴着左铁杆出发时的右限幅值
#define lift_height_max 60000; //抬升最大值

bool ammunition_box_towards(void) //判断弹药箱是否在前方
{
	if (engineer.sensors.laser_switch_left == 1 && engineer.sensors.laser_switch_right == 1 && engineer.sensors.laser_switch_mid == 0)
		return true;
	else
		return false;
}

static bool flag_temp_catchok = false;

void lift_keybord_mode(void) //遥控器的一键抓弹
{

	static u8 last_sl = 4; //遥控器左拨杆从中间拨到下面开启一键抓弹
						   //遥控器右拨杆从中间拨到下面取消一键抓弹（立刻）(当右拨杆在下方的时候无法取弹）
	u16 i = 0;
	if (remote_origin.remote_SL == RP_S_DOWN && last_sl == RP_S_MID)
	{ //此处添加所有机构归零运动，注意先后次序号

		// 爪子左右横移归位   //向右为正，到时再具体测试正负，限幅（留下余量，避免左右碰到车）
		if (abs(engineer_control.lift_x.position_C) > 500)
		{
			engineer.bullet.lift_x_dis = 0;
			while (abs(engineer_control.lift_x.position_C) > 500)
				task_delay_ms(1);
		}
		if (abs(engineer_control.claw.position_C[0]) > 500 || abs(engineer_control.claw.position_C[1]) > 500)
		{
			engineer.bullet.claw_angle = 0; //爪子归位
			while (abs(engineer_control.claw.position_C[0]) > 500 || abs(engineer_control.claw.position_C[1]) > 500)
				task_delay_ms(1);
		}

		//以下确认上述机构以及归位完毕↓
		if (abs(engineer_control.claw.position_C[1]) < 500 && abs(engineer_control.lift_x.position_C) < 500)
		{
			engineer.bullet.lift_height = 0; //抬升归位
			while (abs(engineer_control.lift.position_C[0]) > 5000 || abs(engineer_control.lift.position_C[1]) > 5000)
				task_delay_ms(1);
		}

		engineer.bullet.lift_x_dis = start_location; //回到上升起始位置
		while (abs(engineer.bullet.lift_x_dis - start_location) > 600)
			task_delay_ms(1);

		for (i = 0; i < 350; i++)
		{
			engineer.bullet.lift_height += 1000;
			task_delay_ms(5); //此后使用键盘操作应当添加一件撤销操作功能（在for里面break），把所有操作后面的都跳过
		}
		task_delay_ms(1000);

		engineer.bullet.lift_x_dis = left_limits; //走到最左边开始向右扫描
		while (abs(engineer_control.lift_x.position_C - left_limits) > 600)
			task_delay_ms(1);

		for (; abs(engineer.bullet.lift_x_dis - right_limits) >= 600;) //设定右边目标
		{
			engineer.bullet.lift_x_dis += 2000;
			task_delay_ms(10);
			flag_temp_catchok = ammunition_box_towards();

			if (auto_collect_mode) //一键退出
			{
				break;
			}

			if (flag_temp_catchok)
			{
				engineer.bullet.lift_x_dis = engineer_control.lift_x.position_C;
				for (i = 0; i < 76; i++)
				{
					if (auto_collect_mode) //一键退出
					{
						i = 100; //失能气路
						break;
					}
					engineer.bullet.claw_angle += 1000;
					task_delay_ms(5);
				}

				if (i == 76)
					engineer_control.gas.clamp_claw();
				task_delay_ms(500);

				for (i = 0; i < 60; i++)
				{
					if (auto_collect_mode) //一键退出
					{
						i = 100;
						break;
					}
					engineer.bullet.claw_angle -= 1000;
					task_delay_ms(5);
				}
				task_delay_ms(1500);

				for (i = 0; i < 50; i++)
				{
					if (auto_collect_mode) //一键退出
					{
						i = 100;
						break;
					}
					engineer.bullet.claw_angle += 1000;
					task_delay_ms(5);
				}

				if (i == 50)
					engineer_control.gas.loose_calw();
				task_delay_ms(100);

				for (i = 0; i < 66; i++)
				{
					if (auto_collect_mode) //一键退出
					{
						i = 100;
						break;
					}

					engineer.bullet.claw_angle -= 1000;
					task_delay_ms(5);
				}

				engineer.bullet.claw_angle = 0;

				if (remote_origin.remote_SL == RP_S_DOWN && remote_origin.remote_SR == RP_S_DOWN)
				{
					break;
				}
			}
		}

		engineer.bullet.lift_x_dis = start_location; //回到上升起始位置
		while (abs(engineer_control.lift_x.position_C - start_location) > 600)
			task_delay_ms(1);
		task_delay_ms(700);

		for (i = 0; i < 350; i++) //下降到底
		{

			engineer.bullet.lift_height -= 1000;
			task_delay_ms(5);
		}
		engineer.bullet.lift_height = 0;
	}

	else
	{
	}
	last_sl = remote_origin.remote_SL;

	//此处应当添加撤销操作后的归零
}
void task_print(void *param)
{
	float temp_send[8];

	while (1)
	{
		temp_send[0] = engineer_control.claw.position_C[1];
		temp_send[1] = engineer_control.claw.position_T[1];
		temp_send[2] = engineer_control.claw.speed_C[0];
		temp_send[3] = engineer_control.claw.speed_T[0];
		temp_send[4] = engineer_control.claw.position_T[0];
		temp_send[5] = engineer_control.claw.position_C[0];
		temp_send[6] = engineer_control.holder.angle_T;
		temp_send[7] = engineer_control.claw.current_T[0];

		print_wave(8, 4, &temp_send[0], &temp_send[1], &temp_send[2], &temp_send[3], &temp_send[4], &temp_send[5], &temp_send[6], &temp_send[7]);

		task_delay_ms(10);
	}
}
