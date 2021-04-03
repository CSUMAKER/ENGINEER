/**
  * @file 		correct_ctrl.h
  * @brief		利用matlab系统辨识工具箱得到目标传函,然后利用siso工具箱得到的矫正环节.(功能说明，注意事项，硬件平台，待解决事项等
  * @author		biu123biu
  * @date		    10.8
  * @copyright    CSU RM. All rights reserved.
  */
/*
record:
	10.8-1.完成6623的识别，控制器的设计与C语言部分编写
		 2.效果:跟随效果很好，但稳态精度不够，原因应该是识别确定度
		  不够高(94%)
		 3.效果比pid好,突出表现在:阶跃回落无超调;正弦响应拐点处滞后小。
	        但不是碾压，pid也够用。
		 4.目标传函:
			...
		 5.校正器:
			...
*/

#include "correct_ctrl.h"
/*
@tips:
目标传函(6623,)
num = [0,1.38933316121691,0.162532020890477]
den = [1,5.75969815456932,0.743885242971913]
*/ 

/*
@param:
input:目标值-反馈值
*/
float correct_ctrl(float input){
	uint8_t i;
	/*n为传递函数分子最高阶次*/
	const u8	n = 3;
	const	float b[n+1] = {0,45.5932,-90.5379,44.9449};//分子
	const	float a[n+1] = {1.0000,-1.9997,0.9998,-0.0000};//分母
	static	float	x[n+1] = {0};
	static	float	y[n+1] = {0};
	for(i = n;i>0;i--){
		y[i] = y[i-1];
		x[i] = x[i-1];
	}
	x[0] = input;
	y[0] = 0;
	for(i = n;i > 0;i--){
		y[0] += b[i]*x[i];
		y[0] -= a[i]*y[i];
	}
	return	(y[0]+b[0]*x[0]);
}

