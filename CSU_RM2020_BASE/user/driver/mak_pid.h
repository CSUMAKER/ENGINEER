#ifndef _MAK_PID_H
#define _MAK_PID_H

#include "makos_type.h"

typedef struct 
{
	float kp;
	float ki;
	float kd;
	float errILim_up;//积分上限
	float errILim_down;//积分上限
	float errLim;
	float outLim;
	float errNow;
	float errOld;
	float errP;
	float errI;
	float errD;
	float ctrOut;
} PID_Absolute_Type;

typedef struct 
{
 /*PID算法接口变量，用于给用户获取或修改PID算法的特性*/
 float kp;     //比例系数
 float ki;     //积分系数
 float kd;     //微分系数
 
 float errNow; //当前的误差
 float dCtrOut;//控制增量输出
 float ctrOut;//控制输出
 
 float OutLim;//输出限幅
 float IncLim;//增量限幅

 /*PID算法内部变量，其值不能修改*/
 float errOld1;
 float errOld2;
 
}PID_Increment_Type;

void pid_init_absolute(PID_Absolute_Type* PID,float kp, float ki, float kd, float outLimit, float errlimit);
float PID_Update(PID_Absolute_Type* PID,float Target,float Current);
void pid_zero_absolute(PID_Absolute_Type* PID);

void	pid_init_increment(PID_Increment_Type* PID,float kp, float ki, float kd, float OutLim,float IncLim);
float PID_IncrementMode(PID_Increment_Type* PID,float Target,float Current);
void pid_zero_increment(PID_Increment_Type* PID);
#endif
