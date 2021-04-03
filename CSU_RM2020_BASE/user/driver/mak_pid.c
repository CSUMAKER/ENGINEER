#include "mak_pid.h"
/**
  * @brief  绝对式PID初始化.
  * @param  PID结构体地址，P,I,D,积分限幅.
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
  * @brief  绝对式PID状态清零
  * @param  PID结构体地址
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
  * @brief  绝对式PID.
  * @param  目标值，实际值，PID结构体地址.
  * @note   .
  * @retval 需要输出的值.
  */
float PID_Update(PID_Absolute_Type* PID,float Target,float Current)
{
	PID->errNow = Target - Current;

	PID->errP = PID->errNow;  //读取现在的误差，用于kp控制
	
	PID->errI += PID->errNow; //误差积分，用于ki控制

	if(PID->errLim != 0)	   //积分上限和下限
	{
		LIMIT(PID->errI,PID->errLim,-PID->errLim);
	}
 
	PID->errD = PID->errNow - PID->errOld;//误差微分，用于kd控制

	PID->errOld = PID->errNow;	//保存现在的误差
 
	PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD;//计算绝对式PID输出
	
	if(PID->outLim!=0)
	{
		LIMIT(PID->ctrOut,PID->outLim,-PID->outLim);
	}
	
	return PID->ctrOut;
}

/***********************************************************************************************************************************************/

/**
  * @brief  增量式PID初始化.
  * @param  PID结构体地址，P,I,D,积分限幅，增量限幅.
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
  * @brief  增量式PID清零.
  * @param  PID结构体地址.
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
  * @brief  增量式PID（可以进行增量限幅）.
  * @param  目标值，反馈值，PID结构体地址.
  * @note   返回的并非是增量，而直接是需要输出的值.
  * @retval 需要输出的量.
  */
float PID_IncrementMode(PID_Increment_Type* PID,float Target,float Current)
{
	float dErrP, dErrI, dErrD;
	
	PID->errNow = Target - Current;

	dErrP = PID->errNow - PID->errOld1;

	dErrI = PID->errNow;

	dErrD = PID->errNow - 2 * PID->errOld1 + PID->errOld2;

	/*增量式PID计算*/
	PID->dCtrOut = PID->kp * dErrP + PID->ki * dErrI + PID->kd * dErrD;
	
	PID->errOld2 = PID->errOld1; //二阶误差微分
	PID->errOld1 = PID->errNow;  //一阶误差微分
	
	if (PID->dCtrOut < -PID->IncLim)//增量限幅
		PID->dCtrOut = -PID->IncLim;
	else if (PID->dCtrOut > PID->IncLim)
		PID->dCtrOut = PID->IncLim;
 
	PID->ctrOut += PID->dCtrOut;
	
	LIMIT(PID->ctrOut,PID->OutLim,-(PID->OutLim));//输出限幅
	
	return PID->ctrOut;
}

