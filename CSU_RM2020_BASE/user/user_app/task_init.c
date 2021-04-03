/*
	FILE NAME:	task_init.c
	DESCRIPTION:

	EDIT RECORDS:
	---------
	AUTHOR:		FAN YAOWEN
	EDIT TIME:	2018/3/10
	MODIFICATION:
	---------
*/
#include "task_init.h"
#include "headfile.h"
#include "task_led.h"
#include "task_judge.h"
#include "task_remote.h"
#include "task_UI.h"
#include "UI_lib.h"
#include "task_judge.h"
#include "task_chassis.h"
#include "task_holder.h"
#include "timer.h"
#include "task_vision.h"
#include "task_Ano.h"
#include "task_mpu.h"
#include "rng_led.h"
#include "imu_packet.h"
#include "imu_data_decode.h"
#include "task_engineer_state.h"
#include "task_engineer_control.h"
#include "PWM.h"
#include "task_motor_protect.h"
#include "task_cancel_auto_collect.h"
void base_init(void);

void	task_init(void* param)
{
	base_init();//基础功能硬件初始化
	
	/**********机器人通用任务**************/
	
	task_insert_CCM(task_led_sys, NULL, 4);	//led信号灯
	task_insert_CCM(task_ANO,NULL,3);		//匿名上位机
	task_insert_CCM(task_UI, NULL, 3);		//oled用户交互界面
	task_insert(task_judge, NULL, 2);		//裁判系统
	task_insert(task_remote, NULL, 2);		//遥控器
//	task_insert_CCM(task_mpu, NULL, 2);		//陀螺仪
	
	/*************************************/

	/************用户控制任务*************/

//	task_insert(task_vision_get_old_data,NULL,1);
//	task_insert(task_chassis_control,NULL,1);
//	task_insert(task_holder,NULL,1);
	task_insert(task_engineer_state,NULL,1);
	task_insert(task_engineer_control,NULL,1);
//	task_insert(task_motor_protect,NULL,1);
//	task_insert(task_cancel_auto_collect,NULL,1);//尚未测试功能
	/*************************************/
	
    task_suspend(ptcb_curr);

    while(1)
    {
        task_delay_ms(1000);
    }
}

void base_init(void)
{
	PWM_init();
	uasrt6_init(115200);			//装甲板视觉
//	uasrt4_init(115200);			//imu通讯
	Delay_Timer_Init();				//微妙延时初始化
//	imu_data_decode_init();			//imu解码初始化
	RNG_Init();						//硬件随机数初始化
	TIM12_Int_Init(10000,8400-1);	//can中断速率检测初始化

    CAN1_Configuration((((u32)Control_ID << 3) & 0xffff0000) >> 16, (((u32)Control_ID << 3) | CAN_ID_EXT | CAN_RTR_DATA) & 0xffff,CAN_Mode_Normal);
    can2_Configuration((((u32)Control_ID << 3) & 0xffff0000) >> 16, (((u32)Control_ID << 3) | CAN_ID_EXT | CAN_RTR_DATA) & 0xffff,CAN_Mode_Normal);

    remote_send_msg_init();			//遥控器消息池初始化

//	UI_init();						//oled用户交互初始化
    task_delay_ms(2000);			//等待电机上电
}
