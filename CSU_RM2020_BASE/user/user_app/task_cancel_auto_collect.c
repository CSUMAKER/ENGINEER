#include "task_engineer_state.h"
#include "task_engineer_control.h"
#include "headfile.h"
#include "task_sensors.h"
#include "task_motor_protect.h"
#include "task_cancel_auto_collect.h"

bool auto_collect_mode=true;										//�˳��Զ�ȡ��

void task_cancel_auto_collect(void)									//�˳�һ��ץ��ģʽ���ص���ʼλ��
{
	
	
	while(1)
	{
		if(remote_origin.remote_SL==RP_S_DOWN && remote_origin.remote_SR==RP_S_DOWN)
		{
			auto_collect_mode=false;
			u16 i=0;
			engineer.bullet.claw_angle=0;								//צ�ӹ�λ
			while(abs(engineer_control.claw.position_C[0])>500||abs(engineer_control.claw.position_C[1])>500)
				task_delay_ms(1);
			task_delay_ms(800);
			
			
			engineer.bullet.lift_x_dis=start_location;
			while(abs(engineer_control.lift_x.position_C-start_location)>500)	//�Ȼص�������Ϻ����ʼλ�÷����½�
			{
				task_delay_ms(5);
			}
			task_delay_ms(1000);
			
			
			for(i=0;i<350;i++)													//�½�����
				{	
					engineer.bullet.lift_height-=1000;
					task_delay_ms(5);
				}
			engineer.bullet.lift_height=0;	
			task_delay_ms(500);
		}
	}
}

