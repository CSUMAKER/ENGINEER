#include "task_engineer_state.h"
#include "task_engineer_control.h"
#include "headfile.h"
#include "mak_pid.h"
#include "task_motor_protect.h"

engineer_control_t engineer_control;

void task_engineer_chassis_control(void* param);	//���̿���
void task_engineer_lift_control(void* param);		//̧������
void task_engineer_lift_x_control(void* param);		//̧������
void task_engineer_claw_control(void* param);		//צ�ӿ���
void task_engineer_relief_control(void* param);  //��Ԯ����
void task_engineer_holder_control(void* param);  //��̨����

void task_engineer_arm_control(void* param);			//
void task_engineer_arm_x_control(void* param);			//
void task_engineer_can1_send(void* param);			//ͳһcan1����
void task_engineer_can2_send(void* param);			//ͳһcan2����

void task_engineer_gas(void* param);				//����װ��

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
/*************************CAN����***************************/
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

/*************************����***************************/
/**
  * @brief  �����ٶȼ��㺯��.
  * @param  ������ʸ��.
  * @note   ��.
  * @retval ��.
  */
#define   	GAIN_I		0.5f
#define		GAIN_J		0.5f
#define		GAIN_K		0.9f
void engineer_chassis_speed_T(float	vy,float vx,float vr)
{    //·������
	
	float	temp_vy, temp_vx;
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
	
		
//	�ٶȽ���

	
		
		temp_vy =  vy * (fast_cos((int16_t)engineer_control.holder.angle_T))
						+vx * (fast_sin((int16_t)engineer_control.holder.angle_T));
    
    temp_vx = -vy * (fast_sin((int16_t)engineer_control.holder.angle_T)) 
						+vx * (fast_cos((int16_t)engineer_control.holder.angle_T));
		engineer_control.chassis.position_T[0]	+= (+temp_vx/GAIN_I + temp_vy/GAIN_J + vr/GAIN_K);
		engineer_control.chassis.position_T[1]	+= (+temp_vx/GAIN_I - temp_vy/GAIN_J + vr/GAIN_K);
		engineer_control.chassis.position_T[2]	+= (-temp_vx/GAIN_I - temp_vy/GAIN_J + vr/GAIN_K);
		engineer_control.chassis.position_T[3]	+= (-temp_vx/GAIN_I +	temp_vy/GAIN_J + vr/GAIN_K);
		
	
}



void chassis_pid_init(void)
{
	u8 i;

	pid_init_absolute(&engineer_control.chassis.pid_direction_correct , 1.2 , 0.000 ,00 , 5000 ,500);// ·������pid

//	if(engineer.chassis.motor==RM3508)                         //����ʹ�õĵ����3510��ֻ��ʹ��һ��pid���ٶȻ����ɣ�,����3508����Ҫ����pid
		for(i=0;i<4;i++)
		{
			pid_init_absolute(&engineer_control.chassis.pid_position[i] , 0.54 , 00.01 , 0.02 , 0 ,500);//������Ҫ�ص����
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
		
		if(tick_count_p % 5==0)//λ�û�5ms
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
			
			if(engineer.chassis.motor==RM3508)//����ʹ�õĵ����3510��ֻ��ʹ��һ��pid���ٶȻ����ɣ�,����3508����Ҫ����pid(���飩
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
			//ʧ��̧�����
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
/*************************̧��***************************/


void task_engineer_lift_control(void* param)//����̧�� can1 0x201 x0202����ҡ������
{
	int i;
	static u8 lift_v_count = 0,lift_i_count = 0,lift_p_count = 0;
	for(i=0;i<2;i++)
	{
		pid_init_absolute(&engineer_control.lift.pid_position[i] , 0.64 , 0.0001 , 0.25 , 0     ,500);//������Ҫ�ص����
		pid_init_absolute(&engineer_control.lift.pid_speed[i]    , 7.25 , 0.2 , 3.125   , 0     ,500);	
		pid_init_increment(&engineer_control.lift.pid_current[i] , 1.2 , 0.15 , 0.5 , 10000 , 10000 );
	}
	while(1)
	{
		LIMIT(engineer.bullet.lift_height,250000,0);	
		engineer_control.lift.position_T[0]=engineer.bullet.lift_height;//ץ��Ҳ��һ��һ���ĵ��ת��
		engineer_control.lift.position_T[1]=-engineer.bullet.lift_height;
		
		lift_p_count++;lift_v_count++;lift_i_count++;
		if(lift_p_count % 5==0)//λ�û�5ms
		{
			lift_p_count = 0;
			for(i=0;i<2;i++)
			{
				engineer_control.lift.speed_T[i] = PID_Update(&engineer_control.lift.pid_position[i],engineer_control.lift.position_T[i],engineer_control.lift.position_C[i]);	
			}
		}
		if(lift_v_count % 2==0)//�ٶȻ�2ms
		{
			lift_v_count = 0;
			for(i=0;i<2;i++)
			{
				engineer_control.lift.current_T[i] = PID_Update(&engineer_control.lift.pid_speed[i],engineer_control.lift.speed_T[i],engineer_control.lift.speed_C[i]);	
			}
		}
		if(lift_i_count % 2==0)//������2ms
		{
			lift_i_count = 0;
			for(i=0;i<2;i++)
			{
				engineer_control.lift.CAN_data[i] = (int16_t)PID_IncrementMode(&engineer_control.lift.pid_current[i],engineer_control.lift.current_T[i],engineer_control.lift.current_C[i]);	
			}
			//��ֵ��can��Ϣ
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
			//ʧ��̧�����
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

void task_engineer_lift_x_control(void* param)//pid����δ֪��˿�˵����ǰ����ҡ������
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

		if(lift_x_p_count % 4==0)//λ�û�4ms
		{
			lift_x_p_count = 0;
			engineer_control.lift_x.speed_T = PID_Update(&engineer_control.lift_x.pid_position,engineer_control.lift_x.position_T,engineer_control.lift_x.position_C);
		}
		
		if(lift_x_v_count % 2==0)//�ٶȻ�2ms
		{
			lift_x_v_count = 0;
			engineer_control.lift_x.current_T = PID_Update(&engineer_control.lift_x.pid_speed,engineer_control.lift_x.speed_T,engineer_control.lift_x.speed_C);
		}
		
		if(life_x_i_count % 2==0)//�ٶȻ�2ms
		{
			life_x_i_count = 0;
			engineer_control.lift_x.CAN_data = (int16_t)PID_IncrementMode(&engineer_control.lift_x.pid_current,engineer_control.lift_x.current_T,engineer_control.lift_x.current_C);	
			
			if(lift_x_motors==1)
			{
				//��ֵ��can��Ϣ
				can1_send_data_1[4]=engineer_control.lift_x.CAN_data>>8;
				can1_send_data_1[5]=engineer_control.lift_x.CAN_data&0xff;
			}
			else
			{
			//ʧ�ܺ��Ƶ��
			can1_send_data_1[4]=0;
			can1_send_data_1[5]=0;
			}
		}
		
		task_delay_ms(1);
	}
}

void task_engineer_claw_control(void* param)//pid����δ֪  		//һ��̧������ҡ������
{
	u8 i;
	static u8 claw_i_count = 0,claw_v_count = 0,claw_p_count = 0;
	
	for(i=0;i<2;i++)
	{
		pid_init_absolute(&engineer_control.claw.pid_position[i] , 0.64 , 0.001 , 0.22 , 3000 ,500);//������Ҫ�ص����
		pid_init_absolute(&engineer_control.claw.pid_speed[i]    ,   7.25 , 0.2 , 3.125  , 4000 ,500);	
		pid_init_increment(&engineer_control.claw.pid_current[i] , 1.2 , 0.15 , 0.5 , 10000 , 10000 );
	}
	
	
	while(1)
	{
		LIMIT(engineer.bullet.claw_angle,255000,0);   							//�˴��޸����޷���65000->0
		engineer_control.claw.position_T[0]=engineer.bullet.claw_angle;
		engineer_control.claw.position_T[1]=-engineer.bullet.claw_angle;
		claw_p_count++;claw_v_count++;
		if(claw_p_count % 5==0)//λ�û�5ms
		{
			claw_p_count = 0;
			for(i=0;i<2;i++)
			{
				engineer_control.claw.speed_T[i] = PID_Update(&engineer_control.claw.pid_position[i],engineer_control.claw.position_T[i],engineer_control.claw.position_C[i]);
			}
		}
		if(claw_v_count % 2==0)//�ٶȻ�2ms
		{
			claw_v_count = 0;
			for(i=0;i<2;i++)
			{
				engineer_control.claw.current_T[i] = (int16_t)PID_Update(&engineer_control.claw.pid_speed[i],engineer_control.claw.speed_T[i],engineer_control.claw.speed_C[i]);	
			}
		}
		if(claw_i_count % 2==0)//������2ms
		{
			claw_i_count = 0;
			for(i=0;i<2;i++)
			{
				engineer_control.claw.CAN_data[i] = (int16_t)PID_IncrementMode(&engineer_control.claw.pid_current[i],engineer_control.claw.current_T[i],engineer_control.claw.current_C[i]);	
			}
			//��ֵ��can��Ϣ
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

/********************ץȡ��ҩ*********************/
void out_arm(void)					//�Ҳ���->��
{
	engineer.bullet.temp_arm_angle+=80000;
	engineer.bullet.temp_arm_x_angle+=70000;

}

void back_arm(void)				//�Ҳ���->��
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

/*************************��Ԯ***************************/
void task_engineer_relief_control(void* param)//��Ԯ can2 ���󲦸���->��
{
	int i;
	static u8 relief_v_count = 0,relief_i_count = 0,relief_p_count = 0;
	for(i=0;i<2;i++)
	{
		pid_init_absolute(&engineer_control.relief.pid_position[i] , 0.154 , 0.000 , 0.077 , 0 ,500);//������Ҫ�ص����
		pid_init_absolute(&engineer_control.relief.pid_speed[i] , 20 ,0.01 , 9 , 0 ,500);	
		pid_init_increment(&engineer_control.relief.pid_current[i] , 0.95, 0.060 , 0.524, 10000 , 10000 );
	}
	while(1)
	{
//		LIMIT(engineer.relief.relief_frame,350000,0);
		engineer_control.relief.position_T[0]=-engineer.relief.relief_frame;//ץ��Ҳ��һ��һ���ĵ��ת��
		engineer_control.relief.position_T[1]=engineer.relief.relief_frame;
		
		
		relief_p_count++;relief_v_count++;relief_i_count++;
		if(relief_p_count % 5==0)//λ�û�5ms
		{
			relief_p_count = 0;
			for(i=0;i<2;i++)
			{
				engineer_control.relief.speed_T[i] = PID_Update(&engineer_control.relief.pid_position[i],engineer_control.relief.position_T[i],engineer_control.relief.position_C[i]);	
			}
		}
		if(relief_v_count % 2==0)//�ٶȻ�2ms
		{
			relief_v_count = 0;
			for(i=0;i<2;i++)
			{
				engineer_control.relief.current_T[i] = PID_Update(&engineer_control.relief.pid_speed[i],engineer_control.relief.speed_T[i],engineer_control.relief.speed_C[i]);	
			}
		}
		if(relief_i_count % 2==0)//������2ms
		{
			relief_i_count = 0;
			for(i=0;i<2;i++)
			{
				engineer_control.relief.CAN_data[i] = (int16_t)PID_IncrementMode(&engineer_control.relief.pid_current[i],engineer_control.relief.current_T[i],engineer_control.relief.current_C[i]);	
			}
			
		//��ֵ��can2��Ϣ 0x205 0x206
			
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
			//ʧ��̧�����
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
/*************************��̨***************************/
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

		if(holder_p_count % 4==0)//λ�û�4ms
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
				//��ֵ��can��Ϣ
				can2_send_data_2[4]=engineer_control.holder.CAN_data>>8;
				can2_send_data_2[5]=engineer_control.holder.CAN_data&0xff;
			}
			else
			{
			//ʧ�ܺ��Ƶ��
			can2_send_data_2[4]=0;
			can2_send_data_2[5]=0;
			}
		}
		
		task_delay_ms(1);
	}
}
/********************��е��*********************/
void task_engineer_arm_control(void* param)//jxb can1 ���󲦸���->��
{
	int i;
	static u8 arm_v_count = 0,arm_i_count = 0,arm_p_count = 0;
	for(i=0;i<2;i++)
	{
		pid_init_absolute(&engineer_control.arm.pid_position[i] , 0.54 , 0.001 , 0.32 , 0 ,1500);//������Ҫ�ص����
		pid_init_absolute(&engineer_control.arm.pid_speed[i] , 7.25 , 0.2 , 3.125, 0 ,1500);	
		pid_init_increment(&engineer_control.arm.pid_current[i] , 0.95, 0.15 , 0.42, 10000 , 10000 );
	}
	engineer.bullet.arm_angle=2000;
	while(1)
	{
//		LIMIT(engineer.bullet.arm_angle,86000,0);
	  
		engineer_control.arm.position_T[0]=engineer.bullet.arm_angle;//ץ��Ҳ��һ��һ���ĵ��ת��
		engineer_control.arm.position_T[1]=-engineer.bullet.arm_angle;
		
		
		arm_p_count++;arm_v_count++;arm_i_count++;
		if(arm_p_count % 5==0)//λ�û�5ms
		{
			arm_p_count = 0;
			for(i=0;i<2;i++)
			{
				engineer_control.arm.speed_T[i] = PID_Update(&engineer_control.arm.pid_position[i],engineer_control.arm.position_T[i],engineer_control.arm.position_C[i]);	
			}
		}
		if(arm_v_count % 2==0)//�ٶȻ�2ms
		{
			arm_v_count = 0;
			for(i=0;i<2;i++)
			{
				engineer_control.arm.current_T[i] = PID_Update(&engineer_control.arm.pid_speed[i],engineer_control.arm.speed_T[i],engineer_control.arm.speed_C[i]);	
			}
		}
		if(arm_i_count % 2==0)//������2ms
		{
			arm_i_count = 0;
			for(i=0;i<2;i++)
			{
				engineer_control.arm.CAN_data[i] = (int16_t)PID_IncrementMode(&engineer_control.arm.pid_current[i],engineer_control.arm.current_T[i],engineer_control.arm.current_C[i]);	
			}
		//��ֵ��can1��Ϣ 0x206 0x207
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

void task_engineer_arm_x_control(void* param)//������е�� can1 ���󲦸���->��
{
	
	static u8 arm_x_v_count = 0,arm_x_i_count = 0,arm_x_p_count = 0;
	
		pid_init_absolute(&engineer_control.arm_x.pid_position ,0.340 , 0.001 , 0.151  , 0 ,1500);//������Ҫ�ص����
		pid_init_absolute(&engineer_control.arm_x.pid_speed , 5.5 , 0.201 , 0.1125, 8000 ,1500);	
//		pid_init_increment(&engineer_control.arm_x.pid_current , 1, 0 , 0, 10000 , 10000 );
		engineer.bullet.arm_x_angle=1000;
	while(1)
	{
		
		LIMIT(engineer_control.arm_x.position_T,32000,0);
		engineer_control.arm_x.position_T=engineer.bullet.arm_x_angle;
		

		arm_x_p_count++;arm_x_v_count++;arm_x_i_count++;
		if(arm_x_p_count % 8==0)//λ�û�5ms
		{
			arm_x_p_count = 0;
			engineer_control.arm_x.speed_T = PID_Update(&engineer_control.arm_x.pid_position,engineer_control.arm_x.position_T,engineer_control.arm_x.position_C);	
		}
		if(arm_x_v_count % 2==0)//�ٶȻ�2ms
		{
			arm_x_v_count = 0;
			engineer_control.arm_x.CAN_data = PID_Update(&engineer_control.arm_x.pid_speed,engineer_control.arm_x.speed_T,engineer_control.arm_x.speed_C);	
		}
		if(arm_x_i_count % 2==0)//������2ms
		{
			arm_x_i_count = 0;
//			engineer_control.arm_x.CAN_data = (int16_t)PID_IncrementMode(&engineer_control.arm_x.pid_current,engineer_control.arm_x.current_T,engineer_control.arm_x.current_C);	
			
		//��ֵ��can2��Ϣ 0x208
		
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
  * @brief  ���ټ���cos
  * @param  �����Ƕ�ֵ
  * @retval ����ֵ
  * @attention �������Ǻ��������ԺͶԳ��ԣ�ֻ��ȡ90����ֵ֪���ܼ�������нǶ�ֵ
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
  * @brief  ���ټ���sin
  * @param  �����Ƕ�ֵ
  * @retval ����ֵ
  * @attention �������Ǻ��������ԺͶԳ��ԣ�ֻ��ȡ90����ֵ֪���ܼ�������нǶ�ֵ
  */
float fast_sin(int16_t angle)
{
	return fast_cos(angle - 90);
	
}

///**
//  * @brief  ���ټ���arctan
//  * @param  ����ֵ
//  * @retval ����ֵ
//  * @attention 
//  */
//float fast_atan(int16_t num)
//{
//    
//}
/********************һ��ȡ��*********************/
