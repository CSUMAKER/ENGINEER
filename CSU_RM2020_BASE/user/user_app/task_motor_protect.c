#include "task_engineer_state.h"
#include "task_engineer_control.h"
#include "headfile.h"
#include "task_sensors.h"
#include "task_motor_protect.h"

////底盘电机防止堵转部分，四个电机统一控制
bool chassis_motors=1;							//默认使能，即可以转动,以下相同不再赘述

//两个抬升电机统一控制
bool lift_motors=1 ;

//一个抓弹横移电机
bool lift_x_motors=1 ;

//两个爪子电机统一控制
bool claw_motors=1 ;

//救援电机统一控制
bool relief_motors=1;

//云台电机
bool holder_motors=1;//暂时还没有写保护程序

//三个机械臂电机统一控制
bool arm_motors=1;

u8 claw_jam_time;//定义爪子电机阻塞时间变量
u8 lift_x_jam_time;//定义横移电机阻塞时间变量
u8 lift_jam_time;// 定义抬升电机阻塞时间变量


void task_motor_protect(void *pram)
{
	while(1)
		{
		//爪子电机防止堵转部分
		if(engineer_control.claw.speed_T[0]>150||engineer_control.claw.speed_T[0]<-150)
		{
			
			if(abs(engineer_control.claw.position_T[0]-engineer_control.claw.position_C[0])>3000 || abs(engineer_control.claw.position_T[1]-engineer_control.claw.position_C[1])>3000)
				if(abs(engineer_control.claw.speed_T[0]-engineer_control.claw.speed_C[0])>700 || abs(engineer_control.claw.speed_T[1]-engineer_control.claw.speed_C[1])>700)
					claw_jam_time++;
			
			if(claw_jam_time>=400)
			claw_motors=0;
		}
		task_delay_ms(5);
		
		
		//抓弹横移电机
		if(engineer_control.lift_x.speed_T>300 || engineer_control.lift_x.speed_T<-300)
		{
			
			if(abs(engineer_control.lift_x.position_T-engineer_control.lift_x.position_C)>3000)
				if(abs(engineer_control.lift_x.speed_T-engineer_control.lift_x.speed_C)>600)
					lift_x_jam_time++;
			
			if(lift_x_jam_time>=200)
				lift_x_motors=0;
		}
		//以下堵转代码暂未测试
//		
//		//抬升电机
//		if(engineer_control.lift.speed_T[0]>150 || engineer_control.lift.speed_T[0]<-150)
//		{
//			
//			if(abs(engineer_control.lift.position_T[0]-engineer_control.lift.position_C[0])>3000 || abs(engineer_control.lift.position_T[1]-engineer_control.lift.position_C[1])>3000)
//				if(abs(engineer_control.lift.speed_T[0]-engineer_control.lift.speed_C[0])>600 || abs(engineer_control.lift.speed_T[1]-engineer_control.lift.speed_C[1])>600)
//					lift_jam_time++;
//			
//			if(lift_jam_time>=200)
//				lift_motors=0;
//		}
//		
		}
}
