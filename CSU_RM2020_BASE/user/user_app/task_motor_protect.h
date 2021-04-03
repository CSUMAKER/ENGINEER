#ifndef _TASK_MOTOR_PROTECT_H
#define _TASK_MOTOR_PROTECT_H

#include "headfile.h"

//本程序用于防止电机堵转过流损坏，2020年1月12日//

//底盘电机防止堵转部分，四个电机统一控制
extern bool chassis_motors;								//默认使能，即可以转动,以下相同不再赘述
//两个抬升电机统一控制
extern bool lift_motors;
//一个抓弹横移电机
extern bool lift_x_motors;
//两个爪子电机统一控制
extern bool claw_motors;
//救援电机统一控制
extern bool relief_motors;
//云台电机
extern bool holder_motors;
//三个机械臂电机统一控制
extern bool arm_motors;

typedef struct
{
	u8 cur_claw_value[2];								//记录爪子当前值，用于与0.5秒后的值比较，倘若爪子需要移动但其值却并未改变则是堵转，以下思路相同

	u8 cur_lift_x_value;								//记录左右横移电机当前值

	u8 cur_lift_value[2];								//记录抬升电机当前值

	u8 cur_chassis_value[4];							//记录底盘四个电机当前值
	
}motor_protect_cur_val_t;

extern u8 jam_time;
void task_motor_protect(void *pram);
#endif	
