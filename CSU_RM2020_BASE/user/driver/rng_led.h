#ifndef __RNG_LED_H
#define __RNG_LED_H	 
#include "makos_includes.h"
	
u8  RNG_Init(void);			//RNG��ʼ�� 
u32 RNG_Get_RandomNum(void);//�õ������
int RNG_Get_RandomRange(int min,int max);//����[min,max]��Χ�������
#endif

















