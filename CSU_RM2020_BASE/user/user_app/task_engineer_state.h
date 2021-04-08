#ifndef _TASK_ENGINEER_STATE_H
#define _TASK_ENGINEER_STATE_H

#include "makos_includes.h"

typedef enum
{
	stop=1,	//停止
	follow,	//跟随
	catwalk	//猫步
	
}chassis_mode_t;

typedef enum
{
	handle=1,	//手柄
	keyboard	//键鼠
	
}remote_mode_t;

typedef enum
{
	RM3508=1,
	RM3510     //样车时使用3510，赛车使用3508，故在控制时必须先加以区分以提高代码的重用性
	
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
	
	motor_type_t motor;  //电机种类
	
	float move_X_SPD;//横移速度 右为正
	float move_Y_SPD;//前进速度 前为正
	float move_Z_SPD;//旋转速度 顺时针为正
	
	S32 YAW;//路径修正使用的偏航角记录器
	
	bool is_direction_control; //是否开启路径修正，0 开启，1 关闭
	
}engineer_chassis_t;

typedef struct
{
	float claw_angle;	//夹子角度
	float holder;   //云台
	bool is_claw_clamp;//夹子是否加紧
	
	float lift_height;	//抬升架高度
	float lift_x_dis;	//悬架左右
	bool lift_Y_stick;	//悬架是否前进
	
	float arm_angle;		//
	float arm_x_angle;	//
	float temp_arm_angle;		//
	float temp_arm_x_angle;
}engineer_take_bullet_t;

typedef struct
{
	bool relief_finger;	//救援手指
	float relief_frame;	//救援架
	short AddHPCard;	//加血卡
	
}engineer_relief_t;

typedef struct
{
	bool laser_switch_left;		//左侧激光开关
	bool laser_switch_right;	//右侧激光开关
	bool laser_switch_mid;	//中间激光开关
	S8 catch_ultrasound;		//爪子超声波
	S8 chassis_ultrasound[4];	//底盘超声波
	
}engineer_sensors_t;

typedef struct
{
	remote_mode_t remote_mode;		//遥控控制模式
	
	engineer_take_bullet_t bullet;	//取矿
	
	engineer_relief_t relief;		//救援
	
	engineer_chassis_t chassis;		//底盘
	
	engineer_sensors_t sensors;		//传感器
	
	bool is_supplement;				//补弹弹仓
	
}engineer_state_t;
extern engineer_state_t engineer;
extern engineer_remote_t remote_origin;//暂存遥控器数据

extern bool badass;					//传感器判断弹药箱是否在正前方的变量
extern int start_location;	//上升后移动的开始值，退出一键抓弹后的回复处
extern int left_limits;		//贴着左铁杆出发时的左限幅值,正常来说是负的
extern int right_limits;	//贴着左铁杆出发时的右限幅值

bool ammunition_box_towards(void);	//传感器判断弹药箱是否在正前方的函数（检测动作）
void task_engineer_state(void* param);

#endif

