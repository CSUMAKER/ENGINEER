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
	base_init();//��������Ӳ����ʼ��
	
	/**********������ͨ������**************/
	
	task_insert_CCM(task_led_sys, NULL, 4);	//led�źŵ�
	task_insert_CCM(task_ANO,NULL,3);		//������λ��
	task_insert_CCM(task_UI, NULL, 3);		//oled�û���������
	task_insert(task_judge, NULL, 2);		//����ϵͳ
	task_insert(task_remote, NULL, 2);		//ң����
//	task_insert_CCM(task_mpu, NULL, 2);		//������
	
	/*************************************/

	/************�û���������*************/

//	task_insert(task_vision_get_old_data,NULL,1);
//	task_insert(task_chassis_control,NULL,1);
//	task_insert(task_holder,NULL,1);
	task_insert(task_engineer_state,NULL,1);
	task_insert(task_engineer_control,NULL,1);
//	task_insert(task_motor_protect,NULL,1);
//	task_insert(task_cancel_auto_collect,NULL,1);//��δ���Թ���
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
	uasrt6_init(115200);			//װ�װ��Ӿ�
//	uasrt4_init(115200);			//imuͨѶ
	Delay_Timer_Init();				//΢����ʱ��ʼ��
//	imu_data_decode_init();			//imu�����ʼ��
	RNG_Init();						//Ӳ���������ʼ��
	TIM12_Int_Init(10000,8400-1);	//can�ж����ʼ���ʼ��

    CAN1_Configuration((((u32)Control_ID << 3) & 0xffff0000) >> 16, (((u32)Control_ID << 3) | CAN_ID_EXT | CAN_RTR_DATA) & 0xffff,CAN_Mode_Normal);
    can2_Configuration((((u32)Control_ID << 3) & 0xffff0000) >> 16, (((u32)Control_ID << 3) | CAN_ID_EXT | CAN_RTR_DATA) & 0xffff,CAN_Mode_Normal);

    remote_send_msg_init();			//ң������Ϣ�س�ʼ��

//	UI_init();						//oled�û�������ʼ��
    task_delay_ms(2000);			//�ȴ�����ϵ�
}
