#include "task_engineer_state.h"
#include "task_engineer_control.h"
#include "headfile.h"
#include "task_sensors.h"
#include "task_motor_protect.h"
#include "task_cancel_auto_collect.h"

//����������
//engineer.sensors.laser_switch_left	//�󼤹�
//engineer.sensors.laser_switch_right	//�Ҽ���
//catch_ultrasound						//�м䳬����

engineer_state_t engineer;		//���̳�Ŀ��ֵ
engineer_remote_t remote_origin;//�ݴ�ң��������
static uint16_t last_keybrod[8] = {0};
int16_t front_temp,right_temp;
								//��Ӷ�ת�������񣨽������Ʊ�����
void task_print(void *param);
void chassis_keybord_mode(engineer_chassis_t* robot);
void chassis_handle_mode(engineer_chassis_t* robot);
void lift_handle_mode(void);
void lift_keybord_mode(void);
bool ammunition_box_towards(void);
bool badass;

void task_engineer_state(void* param)
{
	task_insert_CCM(task_print, NULL, 1);
//	task_insert_CCM(task_sensors, NULL, 2);
	p_remote_data data = NULL;
	engineer.chassis.motor=RM3508;     //��ָ����������Է��㷢�ӽ������Ĵ��빦��
	
	while(1)
	{
		data = msg_get_read_some(&remote_msg);
		if(data!=NULL)
		{
			//����ң����ԭʼ�����ź�
			remote_origin.remote_JL_LR=RPC_ZERO(((s16)data->JL_LR - 1024),10);
			remote_origin.remote_JL_UD=RPC_ZERO(((s16)data->JL_UD - 1024),10);
			remote_origin.remote_JR_LR=RPC_ZERO(((s16)data->JR_LR - 1024),10);
			remote_origin.remote_JR_UD=RPC_ZERO(((s16)data->JR_UD - 1024),10);
			remote_origin.remote_LL=(s16)data->LL - 1024;
			remote_origin.remote_SR=data->SR;
			remote_origin.remote_SL=data->SL;
			//�жϿ���ģʽ��������ң����/����
			switch (data->SL)
			{
			case RP_S_UP:
				engineer.remote_mode = handle;
				engineer.chassis.mode = catwalk;
				break;
			case RP_S_MID:
				engineer.remote_mode = handle;
				engineer.chassis.mode = follow;
				if (lift_motors == 0)
				{           
					chassis_motors = 1;           
					lift_motors = 1;           
					lift_x_motors = 1;           
					claw_motors = 1;           
					relief_motors = 1;           
					holder_motors = 1;           
					arm_motors = 1;           
				}
				break;
			case RP_S_DOWN:
				engineer.remote_mode = keyboard;
				chassis_motors = 0;
				lift_motors = 0;
				lift_x_motors = 0;
				claw_motors = 0;
				relief_motors = 0;
				holder_motors = 0;
				arm_motors = 0;
				break;
			default:
				engineer.remote_mode = handle;
				engineer.chassis.mode = stop;
				break;
			}
			//��Բ�ͬ����ģʽ�ֱ����Ŀ��ֵ����
			//
			switch(engineer.remote_mode)
			{
				case handle:
					chassis_handle_mode(&engineer.chassis);
					lift_handle_mode();
					lift_keybord_mode();
					break;
				case keyboard:
					chassis_keybord_mode(&engineer.chassis);
					lift_keybord_mode();
					break;
			}
		}
		else
		{
			engineer.chassis.mode=stop;
		}
		task_delay_ms(1);
	}
}

void chassis_handle_mode(engineer_chassis_t* robot)
{
	switch(robot->mode)
	{
		case follow:
			engineer.chassis.move_X_SPD=remote_origin.remote_JL_LR;
			engineer.chassis.move_Y_SPD=remote_origin.remote_JL_UD;
			engineer.chassis.move_Z_SPD=remote_origin.remote_JR_LR;
			engineer.relief.relief_frame+=remote_origin.remote_JR_UD*1;
			break;
		case catwalk:
			engineer.chassis.move_X_SPD=0;
			engineer.chassis.move_Y_SPD=0;
			engineer.chassis.move_Z_SPD=0;
			engineer.relief.relief_frame=0;
			break;
		case stop:
			/*
			
			*/
			break;
		default:
			engineer.chassis.move_X_SPD=0;
			engineer.chassis.move_Y_SPD=0;
			engineer.chassis.move_Z_SPD=0;
			engineer.relief.relief_frame=0;
			break;
	}
	engineer_chassis_speed_T(engineer.chassis.move_Y_SPD,engineer.chassis.move_X_SPD,engineer.chassis.move_Z_SPD);
}

void chassis_keybord_mode(engineer_chassis_t *robot)
{
	static int16_t speedcount = 0;

	switch (speedcount)
	{
	case 0:
		engineer.chassis.move_Y_SPD = 300 * remote_origin.KeyBoard.w - 300 * remote_origin.KeyBoard.s;
		engineer.chassis.move_X_SPD = 300 * remote_origin.KeyBoard.d - 300 * remote_origin.KeyBoard.a;
		engineer.chassis.move_Z_SPD = 300 * remote_origin.KeyBoard.q - 300 * remote_origin.KeyBoard.e;
		last_keybrod[0] = remote_origin.KeyBoard.w;
		last_keybrod[0] = remote_origin.KeyBoard.s;
		last_keybrod[2] = remote_origin.KeyBoard.d;
		last_keybrod[3] = remote_origin.KeyBoard.a;
		last_keybrod[4] = remote_origin.KeyBoard.q;
		last_keybrod[5] = remote_origin.KeyBoard.e;
		speedcount++;
		break;

	case 1:
		if ((last_keybrod[0] == remote_origin.KeyBoard.w) && (last_keybrod[1] == remote_origin.KeyBoard.s) && (last_keybrod[2] == remote_origin.KeyBoard.d) && (last_keybrod[3] == remote_origin.KeyBoard.a) && (last_keybrod[4] == remote_origin.KeyBoard.q) && (last_keybrod[5] == remote_origin.KeyBoard.e))
		{
			speedcount++;
		}
		else
		{
			engineer.chassis.move_Y_SPD = 300 * remote_origin.KeyBoard.w - 300 * remote_origin.KeyBoard.s;
			engineer.chassis.move_X_SPD = 300 * remote_origin.KeyBoard.d - 300 * remote_origin.KeyBoard.a;
			engineer.chassis.move_Z_SPD = 300 * remote_origin.KeyBoard.q - 300 * remote_origin.KeyBoard.e;
			speedcount = 0;
		}
		break;

	case 2:
		if ((remote_origin.KeyBoard.w == 1) || (remote_origin.KeyBoard.s == 1) || (remote_origin.KeyBoard.a == 1) || (remote_origin.KeyBoard.d == 1) || (remote_origin.KeyBoard.q == 1) || (remote_origin.KeyBoard.e == 1))
		{
			if ((remote_origin.KeyBoard.w == 0) && (remote_origin.KeyBoard.s == 1))
			{
				front_temp = engineer.chassis.move_Y_SPD;
				engineer.chassis.move_Y_SPD = front_temp - 1 * (remote_origin.KeyBoard.shift - remote_origin.KeyBoard.ctrl);
				if (engineer.chassis.move_Y_SPD > 0)
				{
					engineer.chassis.move_Y_SPD = 0;
				}
			}
			else if (remote_origin.KeyBoard.w == 1)
			{
				front_temp = engineer.chassis.move_Y_SPD;
				engineer.chassis.move_Y_SPD = front_temp + 1 * (remote_origin.KeyBoard.shift - remote_origin.KeyBoard.ctrl);
				if (engineer.chassis.move_Y_SPD < 0)
				{
					engineer.chassis.move_Y_SPD = 0;
				}
			}
			if ((remote_origin.KeyBoard.d == 0) && (remote_origin.KeyBoard.a == 1))
			{
				right_temp = engineer.chassis.move_X_SPD;
				engineer.chassis.move_X_SPD = right_temp - 1 * (remote_origin.KeyBoard.shift - remote_origin.KeyBoard.ctrl);
				if (engineer.chassis.move_X_SPD > 0)
				{
					engineer.chassis.move_X_SPD = 0;
				}
			}
			else if (remote_origin.KeyBoard.d == 1)
			{
				right_temp = engineer.chassis.move_X_SPD;
				engineer.chassis.move_X_SPD = right_temp + 1 * (remote_origin.KeyBoard.shift - remote_origin.KeyBoard.ctrl);
				if (engineer.chassis.move_X_SPD < 0)
				{
					engineer.chassis.move_X_SPD = 0;
				}
			}
			if ((remote_origin.KeyBoard.q == 0) && (remote_origin.KeyBoard.e == 1))
			{
				engineer.chassis.move_Z_SPD -= 1 * (remote_origin.KeyBoard.shift - remote_origin.KeyBoard.ctrl);
				if (engineer.chassis.move_Z_SPD > 0)
				{
					engineer.chassis.move_Z_SPD = 0;
				}
			}
			else if (remote_origin.KeyBoard.q == 1)
			{
				engineer.chassis.move_Z_SPD += 1 * (remote_origin.KeyBoard.shift - remote_origin.KeyBoard.ctrl);
				if (engineer.chassis.move_Z_SPD < 0)
				{
					engineer.chassis.move_Z_SPD = 0;
				}
			}
			speedcount = 1;
		}
		else
		{
			speedcount = 0;
		}
		break;
	}
	engineer.chassis.move_X_SPD = RPC_ZERO((engineer.chassis.move_X_SPD - 1024), 10);
	engineer.chassis.move_Y_SPD = RPC_ZERO((engineer.chassis.move_Y_SPD - 1024), 10);
	engineer.chassis.move_Z_SPD = RPC_ZERO((engineer.chassis.move_Z_SPD - 1024), 10);
}

void lift_handle_mode(void)
{
	static u8 last_sr=4;
	static S16 last_ll=0;
	static u8 f_last_sr=4;
	static S16 f_last_ll=0;
	if(engineer.chassis.mode==catwalk)
	{
		//���¿���Ϊλ�û�����
		engineer.bullet.lift_height += remote_origin.remote_JL_UD * 1;	//��ҡ�����¿���̧���߶ȣ����Զ�
					//�˴��Ƕȵ����ֵӦ�øĳɺ궨��
		engineer.bullet.lift_x_dis += remote_origin.remote_JR_LR * 1;	//̧�����Һ��ƣ����Զ�
					//��һ�������ٸ����޷���Ĭ���м�λ�û�Ϊ0
		engineer.bullet.claw_angle += remote_origin.remote_JR_UD * 1;	//��ҡ��צ�ӵ���ת�ǣ����Զ�
		
		
		engineer.bullet.holder += remote_origin.remote_JL_LR * 0.05f;   
//		engineer.bullet.arm_angle+=remote_origin.remote_JL_UD*0.1;	
//					
//		engineer.bullet.arm_x_angle+=remote_origin.remote_JR_UD*0.1;	
		
		if(remote_origin.remote_SR==RP_S_DOWN&&last_sr==RP_S_MID)	//�ҿ������м䲦��������ƻ�е��DK
		{
			out_arm();
		}

		if(remote_origin.remote_SR==RP_S_MID&&last_sr==RP_S_DOWN)
		{
			back_arm();
		}
		if(remote_origin.remote_SR==RP_S_UP&&last_sr==RP_S_MID)
		{
			engineer.bullet.temp_arm_x_angle -= 30000;
		}
		if(remote_origin.remote_SR==RP_S_MID&&last_sr==RP_S_UP)
		{
			engineer.bullet.temp_arm_x_angle += 30000;
		}
		
		if(remote_origin.remote_LL<-250&&abs(last_ll)<50)		//�ҿ������м䲦���������צ�ӵ�ץȡ�Ϸ�
		{
				TIM_SetCompare1(TIM4,10000);	
		}
		else
		{
				TIM_SetCompare1(TIM4,10000);
		}

		int s=40;


		if(engineer.bullet.temp_arm_x_angle!=0&&engineer.bullet.temp_arm_x_angle>0)
		{
			engineer.bullet.arm_x_angle+=1.4*s;
			engineer.bullet.temp_arm_x_angle-=1.4*s;
		}
		if(engineer.bullet.temp_arm_x_angle!=0&&engineer.bullet.temp_arm_x_angle<0)
		{
			engineer.bullet.arm_x_angle-=1.4*s;
			engineer.bullet.temp_arm_x_angle+=1.4*s;
		}
		if(engineer.bullet.temp_arm_angle!=0&&engineer.bullet.temp_arm_angle>0)
		{
			engineer.bullet.arm_angle+=s;
			engineer.bullet.temp_arm_angle-=s;
		}
		if(engineer.bullet.temp_arm_angle!=0&&engineer.bullet.temp_arm_angle<0)
		{
			engineer.bullet.arm_angle-=s;
			engineer.bullet.temp_arm_angle+=s;
		}

	last_sr=remote_origin.remote_SR;
	last_ll=remote_origin.remote_LL;
	}
	if(engineer.chassis.mode==follow)
	{
//		engineer.relief.relief_frame+=remote_origin.remote_JR_UD*1;
		if(remote_origin.remote_SR==RP_S_DOWN&&f_last_sr==RP_S_MID)	//�ҿ������м䲦��������ƾ�Ԯ
		{
			engineer.relief.relief_frame-=76800;
		}

		if(remote_origin.remote_SR==RP_S_MID&&f_last_sr==RP_S_DOWN)
		{
			engineer.relief.relief_frame+=76800;
		}

    if(remote_origin.remote_SR==RP_S_UP && f_last_sr==RP_S_MID)
		{

				engineer_control.holder.holder_control_status = 1;

		}

		if(remote_origin.remote_SR==RP_S_MID && f_last_sr==RP_S_UP)
		{
        engineer_control.holder.holder_control_status = 0;
			  
		}

		f_last_sr=remote_origin.remote_SR;
		f_last_ll=remote_origin.remote_LL;
	}
}


//����������
//engineer.sensors.laser_switch_left	//�󼤹�
//engineer.sensors.laser_switch_right	//�Ҽ���
//engineer.sensors.catch_ultrasound		//�м䳬����
//���ô�����ʵ�ֵ�һ���Զ�ץ��

int start_location=110000 	;				//�������ƶ��Ŀ�ʼֵ���˳�һ��ץ����Ļظ���
int left_limits=-540000		;				//���������˳���ʱ�����޷�ֵ,������˵�Ǹ���
int right_limits=1120000	;				//���������˳���ʱ�����޷�ֵ
#define lift_height_max 60000;				//̧�����ֵ

bool ammunition_box_towards(void)										//�жϵ�ҩ���Ƿ���ǰ��
{	
	if (engineer.sensors.laser_switch_left==1 && engineer.sensors.laser_switch_right==1 && engineer.sensors.laser_switch_mid==0)
		return true;
	else
		return false;
}


static bool flag_temp_catchok=false;

void lift_keybord_mode(void)											//ң������һ��ץ��
{	
	
	static u8 last_sl=4;												//ң�����󲦸˴��м䲦�����濪��һ��ץ��
																		//ң�����Ҳ��˴��м䲦������ȡ��һ��ץ�������̣�(���Ҳ������·���ʱ���޷�ȡ����
	u16 i=0;
	if(remote_origin.remote_SL==RP_S_DOWN && last_sl==RP_S_MID)
	{																	//�˴�������л��������˶���ע���Ⱥ�����   
		
																		// צ�����Һ��ƹ�λ   //����Ϊ������ʱ�پ�������������޷�����������������������������
		if(abs(engineer_control.lift_x.position_C)>500)
		{
			engineer.bullet.lift_x_dis=0;
			while(abs(engineer_control.lift_x.position_C)>500)
				task_delay_ms(1);	
		}					
		if(abs(engineer_control.claw.position_C[0])>500||abs(engineer_control.claw.position_C[1])>500)
		{
			engineer.bullet.claw_angle=0;								//צ�ӹ�λ
			while(abs(engineer_control.claw.position_C[0])>500||abs(engineer_control.claw.position_C[1])>500)
				task_delay_ms(1);
		}

																		//����ȷ�����������Լ���λ��ϡ�
		if (abs(engineer_control.claw.position_C[1])<500 && abs(engineer_control.lift_x.position_C)<500)
		{
				engineer.bullet.lift_height=0;							//̧����λ
				while(abs(engineer_control.lift.position_C[0])>5000||abs(engineer_control.lift.position_C[1])>5000)
					task_delay_ms(1);
		}				

				
		engineer.bullet.lift_x_dis=start_location;						//�ص�������ʼλ��
		while(abs(engineer.bullet.lift_x_dis-start_location)>600)
			task_delay_ms(1);
		
	
		for(i=0;i<350;i++)
		{					
			engineer.bullet.lift_height+=1000;
			task_delay_ms(5);											//�˺�ʹ�ü��̲���Ӧ�����һ�������������ܣ���for����break���������в�������Ķ�����
		}
		task_delay_ms(1000);
		
		engineer.bullet.lift_x_dis=left_limits;							//�ߵ�����߿�ʼ����ɨ��
		while(abs(engineer_control.lift_x.position_C-left_limits)>600)
			task_delay_ms(1);
							
		for(;abs(engineer.bullet.lift_x_dis-right_limits)>=600;)		//�趨�ұ�Ŀ��
		{
			engineer.bullet.lift_x_dis+=2000;
			task_delay_ms(10);
			flag_temp_catchok=ammunition_box_towards();
			
			if(auto_collect_mode)//һ���˳�
				{	
					break;
				}
				
			if(flag_temp_catchok)
			{
				engineer.bullet.lift_x_dis=engineer_control.lift_x.position_C;
				for(i=0;i<76;i++)
				{
					if(auto_collect_mode)//һ���˳�
					{	
						i=100;																						 //ʧ����·
						break;
					}
					engineer.bullet.claw_angle+=1000;
					task_delay_ms(5);
				}
				
				if(i==76)
					engineer_control.gas.clamp_claw();
				task_delay_ms(500);
				
				
				for(i=0;i<60;i++)
				{
					if(auto_collect_mode)//һ���˳�
					{	
						i=100;
						break;
					}
					engineer.bullet.claw_angle-=1000;
					task_delay_ms(5);					
				}
				task_delay_ms(1500);

				
				for(i=0;i<50;i++)
				{
					if(auto_collect_mode)//һ���˳�
					{	
						i=100;
						break;
					}
					engineer.bullet.claw_angle+=1000;
					task_delay_ms(5);
				}
				
				if(i==50)
					engineer_control.gas.loose_calw();
				task_delay_ms(100);

				
				for(i=0;i<66;i++)
				{
					if(auto_collect_mode)//һ���˳�
					{	
						i=100;
						break;
					}
						
					engineer.bullet.claw_angle-=1000;
					task_delay_ms(5);
				}
				
				
				engineer.bullet.claw_angle=0;
				
			if(remote_origin.remote_SL==RP_S_DOWN && remote_origin.remote_SR==RP_S_DOWN)
			{	 
				break;
				
			}

			}
		}
		
		engineer.bullet.lift_x_dis=start_location;						//�ص�������ʼλ��
		while(abs(engineer_control.lift_x.position_C-start_location)>600)
			task_delay_ms(1);
		task_delay_ms(700);
	
		for(i=0;i<350;i++)													//�½�����
			{
				
				engineer.bullet.lift_height-=1000;
				task_delay_ms(5);
			}
			engineer.bullet.lift_height=0;
	}
	
	else
	{
		
	}
	last_sl=remote_origin.remote_SL;
	
	
	//�˴�Ӧ����ӳ���������Ĺ���
}
void task_print(void *param)
{
	float temp_send[8];

	while(1)
	{
		temp_send[0]=engineer_control.holder.position_C;
		temp_send[1]=engineer_control.holder.position_T;
		temp_send[2]=engineer.chassis.YAW;
		temp_send[3]=engineer_control.holder.yaw_angle;
		temp_send[4]=engineer.bullet.holder;
		temp_send[5]=engineer_control.holder.speed_C;
		temp_send[6]=engineer_control.holder.speed_T;
		temp_send[7]=Dir_angle;
		
		print_wave(8,4,&temp_send[0],&temp_send[1],&temp_send[2],&temp_send[3],&temp_send[4],&temp_send[5],&temp_send[6],&temp_send[7]);
		
		task_delay_ms(10);
	}
}

	
