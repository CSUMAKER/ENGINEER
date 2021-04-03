#ifndef _MAK_PID_H
#define _MAK_PID_H

#include "makos_type.h"

typedef struct 
{
	float kp;
	float ki;
	float kd;
	float errILim_up;//��������
	float errILim_down;//��������
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
 /*PID�㷨�ӿڱ��������ڸ��û���ȡ���޸�PID�㷨������*/
 float kp;     //����ϵ��
 float ki;     //����ϵ��
 float kd;     //΢��ϵ��
 
 float errNow; //��ǰ�����
 float dCtrOut;//�����������
 float ctrOut;//�������
 
 float OutLim;//����޷�
 float IncLim;//�����޷�

 /*PID�㷨�ڲ���������ֵ�����޸�*/
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
