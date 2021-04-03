#ifndef _TASK_ENGINEER_CONTROL_H
#define _TASK_ENGINEER_CONTROL_H

#include "makos_includes.h"
#include "mak_pid.h"

#define RPC_ZERO(IN,RANGE)	((IN < RANGE && IN > -RANGE) ? 0 : IN)

typedef struct
{
	float angle_T;
	
	S32 position_T[4];//目标位置
	S32 position_C[4];//实际位置
	
	float speed_T[4];//目标速度,依次对应HL,HR,BL,BR
	float speed_C[4];//实际速度,依次对应HL,HR,BL,BR
	
	float current_T[4];//目标电流,依次对应HL,HR,BL,BR
	float current_C[4];//实际电流,依次对应HL,HR,BL,BR
	
	int16_t CAN_data[4];//CAN发送数据,依次对应HL,HR,BL,BR
	
	PID_Absolute_Type pid_position[4];//位置环
	PID_Absolute_Type pid_speed[4];//速度环
	PID_Increment_Type pid_current[4];//电流环
	PID_Absolute_Type pid_direction_correct;//路径修正中旋转车至正确方向所用的pid
	
}chassis_control_t;

typedef struct
{
	S32 position_T[2];//目标位置
	S32 position_C[2];//实际位置
	
	float speed_T[2];//目标速度
	float speed_C[2];//实际速度
	
	float current_T[2];//目标电流
	float current_C[2];//实际电流
	
	int16_t CAN_data[2];//CAN发送数据
	
	PID_Absolute_Type pid_position[2];//位置环
	PID_Absolute_Type pid_speed[2];//速度环
	PID_Increment_Type pid_current[2];//电流环
	
}lift_control_t;

typedef struct
{
	S32 position_T;//目标位置
	S32 position_C;//实际位置
	
	float speed_T;//目标速度
	float speed_C;//实际速度
	
	float current_T;//目标电流
	float current_C;//实际电流
	
	int16_t CAN_data;//CAN发送数据
	
	
	PID_Absolute_Type pid_position;//位置环
	PID_Absolute_Type pid_speed;//速度环
	PID_Increment_Type pid_current;//电流环
}lift_x_control_t;

typedef struct
{
	S32 position_T[2];//目标位置
	S32 position_C[2];//实际位置
	
	float speed_T[2];//目标速度
	float speed_C[2];//实际速度
	
	float current_T[2];//目标电流
	float current_C[2];//实际电流
	
	int16_t CAN_data[2];//CAN发送数据
	
	PID_Absolute_Type pid_position[2];	//位置环
	PID_Absolute_Type pid_speed[2];		//速度环
	PID_Increment_Type pid_current[2];	//电流环
	
}claw_control_t;

typedef struct
{
	void(*clamp_claw)(void);
	void(*loose_calw)(void);
	
	void(*popup_claw)(void);
	void(*back_calw)(void);
	
	void(*popup_supply)(void);
	void(*back_supply)(void);
	
	void(*popup_relief)(void);
	void(*back_relief)(void);
	
}gas_control_t;

typedef struct
{
	S32 position_T[2];//目标位置
	S32 position_C[2];//实际位置
	
	float speed_T[2];//目标速度
	float speed_C[2];//实际速度
	
	float current_T[2];//目标电流
	float current_C[2];//实际电流
	
	int16_t CAN_data[2];//CAN发送数据
	
	PID_Absolute_Type pid_position[2];//位置环
	PID_Absolute_Type pid_speed[2];//速度环
	PID_Increment_Type pid_current[2];//电流环
	
}relief_control_t;

typedef struct
{
	int angle_T;
	
	S32 position_T;//目标位置
	S32 position_C;//实际位置
	
	float speed_T;//目标速度
	float speed_C;//实际速度
	
	float current_T;//目标电流
	float current_C;//实际电流
	
	int16_t CAN_data;//CAN发送数据
	
	PID_Absolute_Type pid_position;//位置环
	PID_Increment_Type pid_speed;//速度环
	PID_Increment_Type pid_current;//电流环
	
	bool holder_control_status;
}holder_control_t;

typedef struct
{
	S32 position_T[2];//目标位置
	S32 position_C[2];//实际位置
	
	float speed_T[2];//目标速度
	float speed_C[2];//实际速度
	
	float current_T[2];//目标电流
	float current_C[2];//实际电流
	
	int16_t CAN_data[2];//CAN发送数据
	
	PID_Absolute_Type pid_position[2];//位置环
	PID_Absolute_Type pid_speed[2];//速度环
	PID_Increment_Type pid_current[2];//电流环
	
}arm_control_t;

typedef struct
{
	S32 position_T;//目标位置
	S32 position_C;//实际位置
	
	float speed_T;//目标速度
	float speed_C;//实际速度
	
	float current_T;//目标电流
	float current_C;//实际电流
	
	int16_t CAN_data;//CAN发送数据
	
	PID_Absolute_Type pid_position;//位置环
	PID_Absolute_Type pid_speed;//速度环
	PID_Increment_Type pid_current;//电流环
	
}arm_x_control_t;


typedef struct
{
	chassis_control_t chassis;   //底盘
	lift_control_t lift;         //二级抬升
	claw_control_t claw;         //一级抬升
	lift_x_control_t lift_x;     //皮带
	relief_control_t relief;     //救援
	holder_control_t holder;		 //云台
	arm_control_t arm;					 //一级机械臂（两电机）
	arm_x_control_t arm_x;       //二级机械臂（一电机）
	gas_control_t gas;           //
	
}engineer_control_t;
extern engineer_control_t engineer_control;

void engineer_chassis_speed_T(float	vy,float vx,float vr);
void air_pump_on(void);
void air_pump_off(void);
void back_arm(void);
void out_arm(void);
void task_engineer_control(void* param);

#endif

