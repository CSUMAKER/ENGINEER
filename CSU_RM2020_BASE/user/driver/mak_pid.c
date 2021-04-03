#include "mak_pid.h"
/**
  * @brief  ����ʽPID��ʼ��.
  * @param  PID�ṹ���ַ��P,I,D,�����޷�.
  * @note   .
  * @retval none.
  */
void pid_init_absolute(PID_Absolute_Type* PID,float kp, float ki, float kd, float outLimit, float errlimit)
{
	PID->kp		= kp;
	PID->ki		= ki;
	PID->kd		= kd;
	PID->errLim = errlimit;
	PID->outLim = outLimit;
	PID->errNow= 0;
	PID->errP= 0;
	PID->errI= 0;
	PID->errD= 0;
	PID->errOld= 0;
	PID->ctrOut= 0;
}
/**
  * @brief  ����ʽPID״̬����
  * @param  PID�ṹ���ַ
  * @note   .
  * @retval .
  */
void pid_zero_absolute(PID_Absolute_Type* PID)
{
	PID->errNow= 0;
	PID->errP= 0;
	PID->errI= 0;
	PID->errD= 0;
	PID->errOld= 0;
	PID->ctrOut= 0;
}
/**
  * @brief  ����ʽPID.
  * @param  Ŀ��ֵ��ʵ��ֵ��PID�ṹ���ַ.
  * @note   .
  * @retval ��Ҫ�����ֵ.
  */
float PID_Update(PID_Absolute_Type* PID,float Target,float Current)
{
	PID->errNow = Target - Current;

	PID->errP = PID->errNow;  //��ȡ���ڵ�������kp����
	
	PID->errI += PID->errNow; //�����֣�����ki����

	if(PID->errLim != 0)	   //�������޺�����
	{
		LIMIT(PID->errI,PID->errLim,-PID->errLim);
	}
 
	PID->errD = PID->errNow - PID->errOld;//���΢�֣�����kd����

	PID->errOld = PID->errNow;	//�������ڵ����
 
	PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD;//�������ʽPID���
	
	if(PID->outLim!=0)
	{
		LIMIT(PID->ctrOut,PID->outLim,-PID->outLim);
	}
	
	return PID->ctrOut;
}

/***********************************************************************************************************************************************/

/**
  * @brief  ����ʽPID��ʼ��.
  * @param  PID�ṹ���ַ��P,I,D,�����޷��������޷�.
  * @note   .
  * @retval none.
  */
void	pid_init_increment(PID_Increment_Type* PID,float kp, float ki, float kd, float OutLim,float IncLim)
{
	PID->kp		= kp;
	PID->ki		= ki;
	PID->kd		= kd;
	PID->OutLim	= OutLim;
	PID->IncLim = IncLim;
	PID->errNow = 0;
	PID->errOld1 = 0;
	PID->errOld2 = 0;
	PID->dCtrOut = 0;
	PID->ctrOut = 0;
}
/**
  * @brief  ����ʽPID����.
  * @param  PID�ṹ���ַ.
  * @note   .
  * @retval none.
  */
void pid_zero_increment(PID_Increment_Type* PID)
{
	PID->errNow = 0;
	PID->errOld1 = 0;
	PID->errOld2 = 0;
	PID->dCtrOut = 0;
	PID->ctrOut = 0;
}
/**
  * @brief  ����ʽPID�����Խ��������޷���.
  * @param  Ŀ��ֵ������ֵ��PID�ṹ���ַ.
  * @note   ���صĲ�������������ֱ������Ҫ�����ֵ.
  * @retval ��Ҫ�������.
  */
float PID_IncrementMode(PID_Increment_Type* PID,float Target,float Current)
{
	float dErrP, dErrI, dErrD;
	
	PID->errNow = Target - Current;

	dErrP = PID->errNow - PID->errOld1;

	dErrI = PID->errNow;

	dErrD = PID->errNow - 2 * PID->errOld1 + PID->errOld2;

	/*����ʽPID����*/
	PID->dCtrOut = PID->kp * dErrP + PID->ki * dErrI + PID->kd * dErrD;
	
	PID->errOld2 = PID->errOld1; //�������΢��
	PID->errOld1 = PID->errNow;  //һ�����΢��
	
	if (PID->dCtrOut < -PID->IncLim)//�����޷�
		PID->dCtrOut = -PID->IncLim;
	else if (PID->dCtrOut > PID->IncLim)
		PID->dCtrOut = PID->IncLim;
 
	PID->ctrOut += PID->dCtrOut;
	
	LIMIT(PID->ctrOut,PID->OutLim,-(PID->OutLim));//����޷�
	
	return PID->ctrOut;
}

