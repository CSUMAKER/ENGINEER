#include "task_engineer_state.h"
#include "task_engineer_control.h"
#include "headfile.h"
#include "mak_pid.h"
#include "task_motor_protect.h"

engineer_control_t engineer_control;

void task_engineer_chassis_control(void* param);	//底盘控制
void task_engineer_lift_control(void* param);		//抬升控制
void task_engineer_lift_x_control(void* param);		//抬升横移
void task_engineer_claw_control(void* param);		//爪子控制
void task_engineer_relief_control(void* param);  //救援控制
void task_engineer_holder_control(void* param);  //云台控制

void task_engineer_arm_control(void* param);			//
void task_engineer_arm_x_control(void* param);			//
void task_engineer_can1_send(void* param);			//统一can1发送
void task_engineer_can2_send(void* param);			//统一can2发送

void task_engineer_gas(void* param);				//气动装置

void task_engineer_control(void* param)
{
	task_insert_CCM(task_engineer_chassis_control, NULL, 1);
//	task_insert_CCM(task_engineer_lift_control, NULL, 1);
//	task_insert_CCM(task_engineer_lift_x_control, NULL, 1);
//	task_insert_CCM(task_engineer_claw_control, NULL, 1);
	task_insert_CCM(task_engineer_can1_send, NULL, 1);
	task_insert_CCM(task_engineer_can2_send, NULL, 1);
//	task_insert_CCM(task_engineer_relief_control,NULL,1);
//	task_insert_CCM(task_engineer_arm_control, NULL, 1);
//	task_insert_CCM(task_engineer_arm_x_control, NULL, 1);
//	task_insert_CCM(task_engineer_gas, NULL, 2);
	 task_insert_CCM(task_engineer_holder_control,NULL,1);
	while(1)
	{
		task_delay_ms(100);
	}
}
/*************************CAN发送***************************/
static u8 can1_send_data_1[8];//0x200
static u8 can1_send_data_2[8];//0x1ff
void task_engineer_can1_send(void* param)
{
	while(1)
	{
		CAN1_SendMsg(0x200,can1_send_data_1);
		CAN1_SendMsg(0x1ff,can1_send_data_2);
		task_delay_ms(1);
	}
}
static u8 can2_send_data_1[8];//0x200
static u8 can2_send_data_2[8];//0x1ff
void task_engineer_can2_send(void* param)
{
	while(1)
	{
		CAN2_SendMsg(0x200,can2_send_data_1);
		CAN2_SendMsg(0x1ff,can2_send_data_2);
		task_delay_ms(1);
	}
}

/*************************底盘***************************/
/**
  * @brief  底盘速度计算函数.
  * @param  三方向矢量.
  * @note   无.
  * @retval 无.
  */
#define   	GAIN_I		0.5f
#define		GAIN_J		0.5f
#define		GAIN_K		0.9f
void engineer_chassis_speed_T(float	vy,float vx,float vr)
{    //路径修正
	
	float	temp_vy = vy, temp_vx = vx;
	if( /*vy!=0 &&*/ vr==0 && engineer.chassis.mode==follow/*&&remote_origin.remote_SR!=RP_S_UP*/)
	{
		if(engineer.chassis.is_direction_control==0)
		{
      engineer_control.chassis.angle_T=engineer.chassis.YAW;		
		}
		
		engineer.chassis.is_direction_control=1;
		
		if(engineer_control.chassis.angle_T-engineer.chassis.YAW<=5&&engineer_control.chassis.angle_T-engineer.chassis.YAW>=-5)
		{
			
		}
		else if(engineer_control.chassis.angle_T-engineer.chassis.YAW>=5)
		{
			vr=-PID_Update(&engineer_control.chassis.pid_direction_correct,engineer_control.chassis.angle_T,engineer.chassis.YAW+5);
		}
		else if(engineer_control.chassis.angle_T-engineer.chassis.YAW<=-5)
		{
			vr=-PID_Update(&engineer_control.chassis.pid_direction_correct,engineer_control.chassis.angle_T,engineer.chassis.YAW-5);	
		}
	}
	else
	{
		engineer.chassis.is_direction_control=0;
	}
	
		
//	速度解算

	vy = temp_vy * (fast_cos((int16_t)engineer_control.holder.angle_T)) 
	    + temp_vx * (fast_sin((int16_t)engineer_control.holder.angle_T));

	vx = -temp_vy * (fast_sin((int16_t)engineer_control.holder.angle_T)) 
	    + temp_vx * (fast_cos((int16_t)engineer_control.holder.angle_T));
		
	engineer_control.chassis.position_T[0] += (+vx / GAIN_I + vy / GAIN_J + vr / GAIN_K);
	engineer_control.chassis.position_T[1] += (+vx / GAIN_I - vy / GAIN_J + vr / GAIN_K);
	engineer_control.chassis.position_T[2] += (-vx / GAIN_I - vy / GAIN_J + vr / GAIN_K);
	engineer_control.chassis.position_T[3] += (-vx / GAIN_I + vy / GAIN_J + vr / GAIN_K);
}



void chassis_pid_init(void)
{
	u8 i;

	pid_init_absolute(&engineer_control.chassis.pid_direction_correct , 1.2 , 0.000 ,00 , 5000 ,500);// 路径修正pid

//	if(engineer.chassis.motor==RM3508)                         //倘若使用的电机是3510则只需使用一组pid（速度环即可）,若是3508则需要串级pid
		for(i=0;i<4;i++)
		{
			pid_init_absolute(&engineer_control.chassis.pid_position[i] , 0.54 , 00.01 , 0.02 , 0 ,500);//参数需要重调完成
			pid_init_absolute(&engineer_control.chassis.pid_speed[i]    , 7.5 ,0.01 , 3.125 , 0 ,500);	
			pid_init_increment(&engineer_control.chassis.pid_current[i] , 0.95 , 0.15 , 0.004, 10000 , 10000 );
			
		}

		
}

void chassis_pid_control(void)
{
	u8 i;
	static u8 tick_count_p = 0,tick_count_v = 0,tick_count_i = 0;
	tick_count_p++;
	tick_count_v ++;
	tick_count_i++;
	if(engineer.chassis.mode==stop)
	{
		for(i=0;i<4;i++)           
		{
			engineer_control.chassis.CAN_data[i] = 0 ;
			pid_zero_absolute(&engineer_control.chassis.pid_speed[i]);
			pid_zero_increment(&engineer_control.chassis.pid_current[i]);
		}
//		engineer_chassis_send_can();
		for(i = 0; i <4;i++)
		{
			can2_send_data_1[2*i] = 0;
			can2_send_data_1[(2*i) + 1] = 0;
		}
	}
	else
	{
		
		if(tick_count_p % 5==0)//位置环5ms
		{
			tick_count_p = 0;
			for(i=0;i<4;i++)
			{
				engineer_control.chassis.speed_T[i] = PID_Update(&engineer_control.chassis.pid_position[i],engineer_control.chassis.position_T[i],engineer_control.chassis.position_C[i]);	
			}
		}
		if(tick_count_v % 2==0)//2ms
		{
			tick_count_v = 0;
			for(i=0;i<4;i++)           
			{
				engineer_control.chassis.speed_T[i]=RPC_ZERO(engineer_control.chassis.speed_T[i],30); 
				engineer_control.chassis.current_T[i] = PID_Update(&engineer_control.chassis.pid_speed[i],engineer_control.chassis.speed_T[i],engineer_control.chassis.speed_C[i]);	
			}
		}
		if(tick_count_i % 2==0)//2ms
		{
			tick_count_v = 0;
			
			if(engineer.chassis.motor==RM3508)//倘若使用的电机是3510则只需使用一组pid（速度环即可）,若是3508则需要串级pid(两组）
			{
				for(i=0;i<4;i++)
				{
					engineer_control.chassis.CAN_data[i] = (int16_t)PID_IncrementMode(&engineer_control.chassis.pid_current[i],engineer_control.chassis.current_T[i],engineer_control.chassis.current_C[i]);
				}
			}
			else
			{
				for(i=0;i<4;i++)
				{					
					engineer_control.chassis.CAN_data[i]=engineer_control.chassis.current_T[i];
				}
			}
//			engineer_chassis_send_can();
			if(chassis_motors==1)
			{
				for(i = 0; i < 4;i++)
				{
					can2_send_data_1[2*i] = engineer_control.chassis.CAN_data[i]>>8;
					can2_send_data_1[(2*i) + 1] = engineer_control.chassis.CAN_data[i]&0xff;
				}
			}
			else
			{
			//失能抬升电机
				for(i = 0; i <4;i++)
				{
					can2_send_data_1[2*i] = 0;
					can2_send_data_1[(2*i) + 1] = 0;
				}
			}
		}
	}
}
void task_engineer_chassis_control(void* param)
{
	chassis_pid_init();
	while(1)
	{
		chassis_pid_control();
		task_delay_ms(1);
	}
}
/*************************抬升***************************/


void task_engineer_lift_control(void* param)//二级抬升 can1 0x201 x0202，左摇杆上下
{
	int i;
	static u8 lift_v_count = 0,lift_i_count = 0,lift_p_count = 0;
	for(i=0;i<2;i++)
	{
		pid_init_absolute(&engineer_control.lift.pid_position[i] , 0.64 , 0.0001 , 0.25 , 0     ,500);//参数需要重调完成
		pid_init_absolute(&engineer_control.lift.pid_speed[i]    , 7.25 , 0.2 , 3.125   , 0     ,500);	
		pid_init_increment(&engineer_control.lift.pid_current[i] , 1.2 , 0.15 , 0.5 , 10000 , 10000 );
	}
	while(1)
	{
		LIMIT(engineer.bullet.lift_height,250000,0);	
		engineer_control.lift.position_T[0]=engineer.bullet.lift_height;//抓弹也是一正一负的电机转向
		engineer_control.lift.position_T[1]=-engineer.bullet.lift_height;
		
		lift_p_count++;lift_v_count++;lift_i_count++;
		if(lift_p_count % 5==0)//位置环5ms
		{
			lift_p_count = 0;
			for(i=0;i<2;i++)
			{
				engineer_control.lift.speed_T[i] = PID_Update(&engineer_control.lift.pid_position[i],engineer_control.lift.position_T[i],engineer_control.lift.position_C[i]);	
			}
		}
		if(lift_v_count % 2==0)//速度环2ms
		{
			lift_v_count = 0;
			for(i=0;i<2;i++)
			{
				engineer_control.lift.current_T[i] = PID_Update(&engineer_control.lift.pid_speed[i],engineer_control.lift.speed_T[i],engineer_control.lift.speed_C[i]);	
			}
		}
		if(lift_i_count % 2==0)//电流环2ms
		{
			lift_i_count = 0;
			for(i=0;i<2;i++)
			{
				engineer_control.lift.CAN_data[i] = (int16_t)PID_IncrementMode(&engineer_control.lift.pid_current[i],engineer_control.lift.current_T[i],engineer_control.lift.current_C[i]);	
			}
			//赋值给can消息
			if(lift_motors==1)
			{
				for(i = 0; i < 2;i++)
				{
					can1_send_data_1[2*i] = engineer_control.lift.CAN_data[i]>>8;
					can1_send_data_1[(2*i) + 1] = engineer_control.lift.CAN_data[i]&0xff;
				}
			}
			else
				{
			//失能抬升电机
					for(i = 0; i < 2;i++)
					{
						can1_send_data_1[2*i] = 0;
						can1_send_data_1[(2*i) + 1] = 0;
					}
				}
		}
	
		task_delay_ms(1);
	}
}

void task_engineer_lift_x_control(void* param)//pid参数未知，丝杆电机，前后，右摇杆左右
{
	static u8 lift_x_v_count = 0,lift_x_p_count = 0,life_x_i_count = 0;
	
	pid_init_absolute(&engineer_control.lift_x.pid_position , 0.32 , 0.05 , 0.2 , 3000 ,500);//7000
	pid_init_absolute(&engineer_control.lift_x.pid_speed, 0.7 , 0.4 , 1.2 , 0 ,500);//7000
	pid_init_increment(&engineer_control.lift_x.pid_current , 1.3 , 0.8 , 0.001 , 6000 ,5000);//10000
	
	while(1)
	{
//		LIMIT(engineer.bullet.lift_x_dis,1120000,-540000);	
		engineer_control.lift_x.position_T=engineer.bullet.lift_x_dis;
		lift_x_p_count++;lift_x_v_count++;life_x_i_count++;

		if(lift_x_p_count % 4==0)//位置环4ms
		{
			lift_x_p_count = 0;
			engineer_control.lift_x.speed_T = PID_Update(&engineer_control.lift_x.pid_position,engineer_control.lift_x.position_T,engineer_control.lift_x.position_C);
		}
		
		if(lift_x_v_count % 2==0)//速度环2ms
		{
			lift_x_v_count = 0;
			engineer_control.lift_x.current_T = PID_Update(&engineer_control.lift_x.pid_speed,engineer_control.lift_x.speed_T,engineer_control.lift_x.speed_C);
		}
		
		if(life_x_i_count % 2==0)//速度环2ms
		{
			life_x_i_count = 0;
			engineer_control.lift_x.CAN_data = (int16_t)PID_IncrementMode(&engineer_control.lift_x.pid_current,engineer_control.lift_x.current_T,engineer_control.lift_x.current_C);	
			
			if(lift_x_motors==1)
			{
				//赋值给can消息
				can1_send_data_1[4]=engineer_control.lift_x.CAN_data>>8;
				can1_send_data_1[5]=engineer_control.lift_x.CAN_data&0xff;
			}
			else
			{
			//失能横移电机
			can1_send_data_1[4]=0;
			can1_send_data_1[5]=0;
			}
		}
		
		task_delay_ms(1);
	}
}

void task_engineer_claw_control(void* param)//pid参数未知  		//一级抬升，右摇杆上下
{
	u8 i;
	static u8 claw_i_count = 0,claw_v_count = 0,claw_p_count = 0;
	
	for(i=0;i<2;i++)
	{
		pid_init_absolute(&engineer_control.claw.pid_position[i] , 0.64 , 0.001 , 0.22 , 3000 ,500);//参数需要重调完成
		pid_init_absolute(&engineer_control.claw.pid_speed[i]    ,   7.25 , 0.2 , 3.125  , 4000 ,500);	
		pid_init_increment(&engineer_control.claw.pid_current[i] , 1.2 , 0.15 , 0.5 , 10000 , 10000 );
	}
	
	
	while(1)
	{
		LIMIT(engineer.bullet.claw_angle,255000,0);   							//此处修改了限幅，65000->0
		engineer_control.claw.position_T[0]=engineer.bullet.claw_angle;
		engineer_control.claw.position_T[1]=-engineer.bullet.claw_angle;
		claw_p_count++;claw_v_count++;
		if(claw_p_count % 5==0)//位置环5ms
		{
			claw_p_count = 0;
			for(i=0;i<2;i++)
			{
				engineer_control.claw.speed_T[i] = PID_Update(&engineer_control.claw.pid_position[i],engineer_control.claw.position_T[i],engineer_control.claw.position_C[i]);
			}
		}
		if(claw_v_count % 2==0)//速度环2ms
		{
			claw_v_count = 0;
			for(i=0;i<2;i++)
			{
				engineer_control.claw.current_T[i] = (int16_t)PID_Update(&engineer_control.claw.pid_speed[i],engineer_control.claw.speed_T[i],engineer_control.claw.speed_C[i]);	
			}
		}
		if(claw_i_count % 2==0)//电流环2ms
		{
			claw_i_count = 0;
			for(i=0;i<2;i++)
			{
				engineer_control.claw.CAN_data[i] = (int16_t)PID_IncrementMode(&engineer_control.claw.pid_current[i],engineer_control.claw.current_T[i],engineer_control.claw.current_C[i]);	
			}
			//赋值给can消息
			if (claw_motors==1)
			{
				can1_send_data_1[6]=engineer_control.claw.CAN_data[0]>>8;
				can1_send_data_1[7]=engineer_control.claw.CAN_data[0]&0xff;
				
				can1_send_data_2[0]=-engineer_control.claw.CAN_data[0]>>8;
				can1_send_data_2[1]=-engineer_control.claw.CAN_data[0]&0xff;
			}
			else
			{
				can1_send_data_1[6]=0;
				can1_send_data_1[7]=0;
				can1_send_data_2[0]=0;
				can1_send_data_2[1]=0;
			}
		}
		task_delay_ms(1);
	}

}

/********************抓取弹药*********************/
void out_arm(void)					//右拨杆->中
{
	engineer.bullet.temp_arm_angle+=80000;
	engineer.bullet.temp_arm_x_angle+=70000;

}

void back_arm(void)				//右拨杆->上
{
	engineer.bullet.temp_arm_angle-=80000;
	engineer.bullet.temp_arm_x_angle-=70000;
}

//void air_pump_on(void)
//{
//	GPIO_ResetBits(GPIOD, GPIO_Pin_6);
//}

//void air_pump_off(void)
//{
//	GPIO_SetBits(GPIOD, GPIO_Pin_6);
//}

/*************************救援***************************/
void task_engineer_relief_control(void* param)//救援 can2 ，左拨杆中->下
{
	int i;
	static u8 relief_v_count = 0,relief_i_count = 0,relief_p_count = 0;
	for(i=0;i<2;i++)
	{
		pid_init_absolute(&engineer_control.relief.pid_position[i] , 0.154 , 0.000 , 0.077 , 0 ,500);//参数需要重调完成
		pid_init_absolute(&engineer_control.relief.pid_speed[i] , 20 ,0.01 , 9 , 0 ,500);	
		pid_init_increment(&engineer_control.relief.pid_current[i] , 0.95, 0.060 , 0.524, 10000 , 10000 );
	}
	while(1)
	{
//		LIMIT(engineer.relief.relief_frame,350000,0);
		engineer_control.relief.position_T[0]=-engineer.relief.relief_frame;//抓弹也是一正一负的电机转向
		engineer_control.relief.position_T[1]=engineer.relief.relief_frame;
		
		
		relief_p_count++;relief_v_count++;relief_i_count++;
		if(relief_p_count % 5==0)//位置环5ms
		{
			relief_p_count = 0;
			for(i=0;i<2;i++)
			{
				engineer_control.relief.speed_T[i] = PID_Update(&engineer_control.relief.pid_position[i],engineer_control.relief.position_T[i],engineer_control.relief.position_C[i]);	
			}
		}
		if(relief_v_count % 2==0)//速度环2ms
		{
			relief_v_count = 0;
			for(i=0;i<2;i++)
			{
				engineer_control.relief.current_T[i] = PID_Update(&engineer_control.relief.pid_speed[i],engineer_control.relief.speed_T[i],engineer_control.relief.speed_C[i]);	
			}
		}
		if(relief_i_count % 2==0)//电流环2ms
		{
			relief_i_count = 0;
			for(i=0;i<2;i++)
			{
				engineer_control.relief.CAN_data[i] = (int16_t)PID_IncrementMode(&engineer_control.relief.pid_current[i],engineer_control.relief.current_T[i],engineer_control.relief.current_C[i]);	
			}
			
		//赋值给can2消息 0x205 0x206
			
		if(relief_motors==1)
			{
				for(i = 0; i < 2;i++)
				{
					can2_send_data_2[2*i] = engineer_control.relief.CAN_data[i]>>8;
					can2_send_data_2[(2*i) + 1] = engineer_control.relief.CAN_data[i]&0xff;
				}
			}
			else
			{
			//失能抬升电机
				for(i = 0; i <2;i++)
				{
					can2_send_data_2[2*i] = 0;
					can2_send_data_2[(2*i) + 1] = 0;
				}
			}
		task_delay_ms(1);
	}
}
}
/*************************云台***************************/
float pp1 = 0.345,pp2 = 3,pi1 = 0.0018,pi2 = 0.02,pd1,pd2;
void task_engineer_holder_control(void* param)        //CAN2,0x207
{
	static u8 holder_v_count = 0,holder_p_count = 0,holder_i_count = 0;
	
	pid_init_absolute(&engineer_control.holder.pid_position , 0.0540 , 0.002 , 0.022 , 5500 ,500);//7000
	
	pid_init_absolute(&engineer_control.holder.pid_speed , 11 , 0.9 , 2.35 , 10000 ,5000);//10000


	while(1)
	{ 

		engineer_control.holder.speed_T=engineer.bullet.holder;
		holder_p_count++;holder_v_count++;holder_i_count++;

		if(holder_p_count % 4==0)//位置环4ms
		{
			holder_p_count = 0;
			engineer_control.holder.speed_T = PID_Update(&engineer_control.holder.pid_position,engineer_control.holder.position_T,engineer_control.holder.position_C);
		}
		if(holder_v_count % 2==0)//
		{
			holder_v_count = 0;
			engineer_control.holder.CAN_data = (int16_t)PID_Update(&engineer_control.holder.pid_speed,engineer_control.holder.speed_T,engineer_control.holder.speed_C);
		
			if(holder_motors ==1)
			{
				//赋值给can消息
				can2_send_data_2[4]=engineer_control.holder.CAN_data>>8;
				can2_send_data_2[5]=engineer_control.holder.CAN_data&0xff;
			}
			else
			{
			//失能横移电机
			can2_send_data_2[4]=0;
			can2_send_data_2[5]=0;
			}
		}
		
		task_delay_ms(1);
	}
}
/********************机械臂*********************/
void task_engineer_arm_control(void* param)//jxb can1 ，左拨杆中->下
{
	int i;
	static u8 arm_v_count = 0,arm_i_count = 0,arm_p_count = 0;
	for(i=0;i<2;i++)
	{
		pid_init_absolute(&engineer_control.arm.pid_position[i] , 0.54 , 0.001 , 0.32 , 0 ,1500);//参数需要重调完成
		pid_init_absolute(&engineer_control.arm.pid_speed[i] , 7.25 , 0.2 , 3.125, 0 ,1500);	
		pid_init_increment(&engineer_control.arm.pid_current[i] , 0.95, 0.15 , 0.42, 10000 , 10000 );
	}
	engineer.bullet.arm_angle=2000;
	while(1)
	{
//		LIMIT(engineer.bullet.arm_angle,86000,0);
	  
		engineer_control.arm.position_T[0]=engineer.bullet.arm_angle;//抓弹也是一正一负的电机转向
		engineer_control.arm.position_T[1]=-engineer.bullet.arm_angle;
		
		
		arm_p_count++;arm_v_count++;arm_i_count++;
		if(arm_p_count % 5==0)//位置环5ms
		{
			arm_p_count = 0;
			for(i=0;i<2;i++)
			{
				engineer_control.arm.speed_T[i] = PID_Update(&engineer_control.arm.pid_position[i],engineer_control.arm.position_T[i],engineer_control.arm.position_C[i]);	
			}
		}
		if(arm_v_count % 2==0)//速度环2ms
		{
			arm_v_count = 0;
			for(i=0;i<2;i++)
			{
				engineer_control.arm.current_T[i] = PID_Update(&engineer_control.arm.pid_speed[i],engineer_control.arm.speed_T[i],engineer_control.arm.speed_C[i]);	
			}
		}
		if(arm_i_count % 2==0)//电流环2ms
		{
			arm_i_count = 0;
			for(i=0;i<2;i++)
			{
				engineer_control.arm.CAN_data[i] = (int16_t)PID_IncrementMode(&engineer_control.arm.pid_current[i],engineer_control.arm.current_T[i],engineer_control.arm.current_C[i]);	
			}
		//赋值给can1消息 0x206 0x207
			if(arm_motors ==1)
			{
				for(i = 0; i < 2;i++)
				{
					can1_send_data_2[2*i+2] = engineer_control.arm.CAN_data[i]>>8;
					can1_send_data_2[2*i+3] = engineer_control.arm.CAN_data[i]&0xff;
				}
			}
			else
			{
				for(i = 0; i < 2;i++)
				{
					can1_send_data_2[2*i+2] = 0;
					can1_send_data_2[2*i+3] = 0;
				}
			}
		task_delay_ms(1);
		}
	}
}

void task_engineer_arm_x_control(void* param)//二级机械臂 can1 ，左拨杆中->下
{
	
	static u8 arm_x_v_count = 0,arm_x_i_count = 0,arm_x_p_count = 0;
	
		pid_init_absolute(&engineer_control.arm_x.pid_position ,0.340 , 0.001 , 0.151  , 0 ,1500);//参数需要重调完成
		pid_init_absolute(&engineer_control.arm_x.pid_speed , 5.5 , 0.201 , 0.1125, 8000 ,1500);	
//		pid_init_increment(&engineer_control.arm_x.pid_current , 1, 0 , 0, 10000 , 10000 );
		engineer.bullet.arm_x_angle=1000;
	while(1)
	{
		
		LIMIT(engineer_control.arm_x.position_T,32000,0);
		engineer_control.arm_x.position_T=engineer.bullet.arm_x_angle;
		

		arm_x_p_count++;arm_x_v_count++;arm_x_i_count++;
		if(arm_x_p_count % 8==0)//位置环5ms
		{
			arm_x_p_count = 0;
			engineer_control.arm_x.speed_T = PID_Update(&engineer_control.arm_x.pid_position,engineer_control.arm_x.position_T,engineer_control.arm_x.position_C);	
		}
		if(arm_x_v_count % 2==0)//速度环2ms
		{
			arm_x_v_count = 0;
			engineer_control.arm_x.CAN_data = PID_Update(&engineer_control.arm_x.pid_speed,engineer_control.arm_x.speed_T,engineer_control.arm_x.speed_C);	
		}
		if(arm_x_i_count % 2==0)//电流环2ms
		{
			arm_x_i_count = 0;
//			engineer_control.arm_x.CAN_data = (int16_t)PID_IncrementMode(&engineer_control.arm_x.pid_current,engineer_control.arm_x.current_T,engineer_control.arm_x.current_C);	
			
		//赋值给can2消息 0x208
		
		if(arm_motors ==1)
		{
			can1_send_data_2[6] = engineer_control.arm_x.CAN_data>>8;
			can1_send_data_2[7] = engineer_control.arm_x.CAN_data&0xff;
		}
		else
		{
			can1_send_data_2[6] = 0;
			can1_send_data_2[7] = 0;
		}
		task_delay_ms(1);
	}
}
}


/********************一键取矿*********************/
