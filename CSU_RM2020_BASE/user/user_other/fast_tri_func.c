/**
  ******************************************************************************
  * @file   fast_tri_func.c 
  * @author Qj
  * @brief  快速计算三角函数
  * @date     
  ******************************************************************************
  * @attention
  *
  * Copyright (c) CSU_RM_FYT.
  * All rights reserved.
  *
  * This software component is licensed by SY under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
  
/* includes ------------------------------------------------------------------*/
#include "fast_tri_func.h"

/* variables -----------------------------------------------------------------*/
float _fast_cos[91] = { 1,
0.999848,0.999391,0.99863,0.997564,0.996195,0.994522,0.992546,0.990268,0.987688,0.984808,
0.981627,0.978148,0.97437,0.970296,0.965926,0.961262,0.956305,0.951057,0.945519,0.939693,
0.93358,0.927184,0.920505,0.913545,0.906308,0.898794,0.891007,0.882948,0.87462,0.866025,
0.857167,0.848048,0.838671,0.829038,0.819152,0.809017,0.798635,0.788011,0.777146,0.766044,
0.75471,0.743145,0.731354,0.71934,0.707107,0.694658,0.681998,0.669131,0.656059,0.642788,
0.62932,0.615661,0.601815,0.587785,0.573576,0.559193,0.544639,0.529919,0.515038,0.5,
0.48481,0.469471,0.45399,0.438371,0.422618,0.406737,0.390731,0.374606,0.358368,0.34202,
0.325568,0.309017,0.292372,0.275637,0.258819,0.241922,0.224951,0.207912,0.190809,0.173648,
0.156434,0.139173,0.121869,0.104528,0.0871556,0.0697563,0.0523358,0.0348993,0.0174522,-1.73205e-07};

/**
  * @brief  快速计算cos
  * @param  整数角度值
  * @retval 计算值
  * @attention 根据三角函数周期性和对称性，只需取90个已知值就能计算出所有角度值
  */
float fast_cos(int16_t angle)
{
	if (angle>=0 && angle <= 90)
	{
		return _fast_cos[angle];
	}
	else if (angle > 90 && angle <=180)
	{
		return -(_fast_cos[180-angle]);
	}
	else if (angle > 180 && angle <=360)
	{
		return fast_cos(360-angle);
	}
	else if (angle > 360)
	{
		return fast_cos(angle - 360);
	}
	else if (angle < 0)
	{
		return (fast_cos(-angle));
	}
	return 0;
}

/**
  * @brief  快速计算sin
  * @param  整数角度值
  * @retval 计算值
  * @attention 根据三角函数周期性和对称性，只需取90个已知值就能计算出所有角度值
  */
float fast_sin(int16_t angle)
{
	return fast_cos(angle - 90);
	return 0;
}

///**
//  * @brief  快速计算arctan
//  * @param  整数值
//  * @retval 计算值
//  * @attention 
//  */
//float fast_atan(int16_t num)
//{
//    
//}
