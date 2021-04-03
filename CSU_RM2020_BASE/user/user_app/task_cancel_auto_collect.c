#include "task_engineer_state.h"
#include "task_engineer_control.h"
#include "headfile.h"
#include "task_sensors.h"
#include "task_motor_protect.h"
#include "task_cancel_auto_collect.h"

bool auto_collect_mode=true;										//退出自动取弹

void task_cancel_auto_collect(void)									//退出一键抓弹模式并回到初始位置
{
	
	
	while(1)
	{
		if(remote_origin.remote_SL==RP_S_DOWN && remote_origin.remote_SR==RP_S_DOWN)
		{
			auto_collect_mode=false;
			u16 i=0;
			engineer.bullet.claw_angle=0;								//爪子归位
			while(abs(engineer_control.claw.position_C[0])>500||abs(engineer_control.claw.position_C[1])>500)
				task_delay_ms(1);
			task_delay_ms(800);
			
			
			engineer.bullet.lift_x_dis=start_location;
			while(abs(engineer_control.lift_x.position_C-start_location)>500)	//先回到上升完毕后的起始位置方便下降
			{
				task_delay_ms(5);
			}
			task_delay_ms(1000);
			
			
			for(i=0;i<350;i++)													//下降到底
				{	
					engineer.bullet.lift_height-=1000;
					task_delay_ms(5);
				}
			engineer.bullet.lift_height=0;	
			task_delay_ms(500);
		}
	}
}

