#include "can.h"
#include "user_can.h"
#include "task_holder.h"
#include "motor.h"
#include "flash_data.h"
#include "timer.h"
#include "task_engineer_state.h"
#include "task_engineer_control.h"

void	CAN1_TX_IRQHandler(void)
{
	if(CAN_GetITStatus(CAN1, CAN_IT_TME) != RESET)
	{
		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
		canrate.inc.can1_tx++;
	}
}

void	CAN2_TX_IRQHandler(void)
{
	if(CAN_GetITStatus(CAN2,CAN_IT_TME) != RESET)
	{		
		CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
		canrate.inc.can2_tx++;
	}
}

#define	CIRCLE_FASTEST	1000

void	circle_count(p_mcircle pmc, U16 angle, S16 speed)
{
	if(	(angle < CIRCLE_FASTEST) && (pmc->angle > 8192-CIRCLE_FASTEST) && (speed > 0) )
	{
		pmc->circle++;
	}
	else if((angle > 8192-CIRCLE_FASTEST) && (pmc->angle < CIRCLE_FASTEST) && (speed < 0) )
	{
		pmc->circle--;
	}
	pmc->angle = angle;
}

mcircle_t	motor_circle[2][8] = {0};

void	circle_count_simple(p_mcircle pmc, U16 angle)
{
	if(	(angle < CIRCLE_FASTEST) && (pmc->angle > 8192-CIRCLE_FASTEST) )
	{
		pmc->circle++;
	}
	else if((angle > 8192-CIRCLE_FASTEST) && (pmc->angle < CIRCLE_FASTEST) )
	{
		pmc->circle--;
	}
	pmc->angle = angle;
}

void	circle_count_engineer(p_mcircle pmc, U16 angle ,U32 uplimit ,U32 downlimit)
{
	if(	(angle < downlimit) && (pmc->angle > uplimit-downlimit) )
	{
		pmc->circle++;
	}
	else if((angle > uplimit-downlimit) && (pmc->angle < downlimit) )
	{
		pmc->circle--;
	}
	pmc->angle = angle;
}

mcircle_t	motor_circle_simple[2][8] = {0};//电机角度和圈数
mcircle_t temp_chassis_yaw;

extern int16_t pitch_torque;	//扭矩电流
extern int16_t yaw_torque;
extern int16_t pitch_speed;		//转速
extern int16_t yaw_speed;

static float n = 0.6;//电流低通滤波的n值

static S32 lift_init_position[2]={0,0};
static S32 lift_x_init_position=0;
static S32 claw_init_position[2]={0,0};
static S32 relief_init_position[2]={0,0};
static S32 holder_init_position=0;
static S32 arm_init_position[2]={0,0};
static S32 arm_x_init_position=0;
static S32 chassis_init_position[4]={0,0,0,0};


void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg	RxMessage;

	static u8 flag_claw1=0,flag_claw2=0,flag_lift[2]={0,0},flag_lift_x=0,flag_arm[2]={0,0},flag_arm_x=0;
	if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		CAN_ClearFlag(CAN1, CAN_FLAG_FMP0);
		
		CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
		
		if ((RxMessage.IDE==CAN_ID_STD)&&(RxMessage.DLC==8))	
		{
			if((RxMessage.StdId >= 0x201) && (RxMessage.StdId <= 0x208))
    				circle_count_simple(&(motor_circle_simple[0][RxMessage.StdId-0x201]), ((U16)(RxMessage.Data[0]<<8))|((U16)RxMessage.Data[1]));

			//二级抬升电机
			if(RxMessage.StdId == 0x201)
			{
				engineer_control.lift.position_C[0]  = (S32)(motor_circle_simple[0][0].angle) + motor_circle_simple[0][0].circle*8192-lift_init_position[0];
				if(!flag_lift[0])
				{
					if(engineer_control.lift.position_C[0]!=0)
					{
						lift_init_position[0]=engineer_control.lift.position_C[0];
						flag_lift[0]=1;
					}
				}
				engineer_control.lift.speed_C[0]=(float)RPC_ZERO((S16)(RxMessage.Data[2]<<8|RxMessage.Data[3]),10);
				engineer_control.lift.current_C[0]=engineer_control.lift.current_C[0]*(1-n)+n*(float)RPC_ZERO((S16)(RxMessage.Data[4]<<8|RxMessage.Data[5]),10);
				canrate.inc.rx_motor[7]++;
			}
			if(RxMessage.StdId == 0x202)
			{
				engineer_control.lift.position_C[1]  = (S32)(motor_circle_simple[0][1].angle) + motor_circle_simple[0][1].circle*8192-lift_init_position[1];
				if(!flag_lift[1])
				{
					if(engineer_control.lift.position_C[1]!=1)
					{
						lift_init_position[1]=engineer_control.lift.position_C[1];
						flag_lift[1]=1;
					}
				}
				engineer_control.lift.speed_C[1]=(float)RPC_ZERO((S16)(RxMessage.Data[2]<<8|RxMessage.Data[3]),10);
				engineer_control.lift.current_C[1]=engineer_control.lift.current_C[1]*(1-n)+n*(float)RPC_ZERO((S16)(RxMessage.Data[4]<<8|RxMessage.Data[5]),10);
				canrate.inc.rx_motor[8]++;
			}

			
			//丝杆
			if(RxMessage.StdId == 0x203)
			{
				engineer_control.lift_x.position_C  = (S32)(motor_circle_simple[0][2].angle) + motor_circle_simple[0][2].circle*8192-lift_x_init_position;
				if(!flag_lift_x)
				{
					if(engineer_control.lift_x.position_C!=0)
					{
						lift_x_init_position=engineer_control.lift_x.position_C;
						flag_lift_x=1;
					}
				}
				engineer_control.lift_x.speed_C=(float)RPC_ZERO((S16)(RxMessage.Data[2]<<8|RxMessage.Data[3]),10);
				engineer_control.lift_x.current_C=engineer_control.lift_x.current_C*(1-n)+n*(float)RPC_ZERO((S16)(RxMessage.Data[4]<<8|RxMessage.Data[5]),10);
				canrate.inc.rx_motor[9]++;
			}
			//一级抬升
			if(RxMessage.StdId == 0x204)
			{
				engineer_control.claw.position_C[0]  = (S32)(motor_circle_simple[0][3].angle) + motor_circle_simple[0][3].circle*8192-claw_init_position[0];
				if(engineer_control.claw.position_C[0]!=0)
				{
					if(!flag_claw1)
					{
						claw_init_position[0]=engineer_control.claw.position_C[0];
						flag_claw1=1;
					}
				}
				engineer_control.claw.speed_C[0]=(float)RPC_ZERO((S16)(RxMessage.Data[2]<<8|RxMessage.Data[3]),10);
				engineer_control.claw.current_C[0]=engineer_control.lift.current_C[0]*(1-n)+n*(float)RPC_ZERO((S16)(RxMessage.Data[4]<<8|RxMessage.Data[5]),10);
				canrate.inc.rx_motor[10]++;
			}
			if(RxMessage.StdId == 0x205)
			{
				engineer_control.claw.position_C[1]  = ((S32)(motor_circle_simple[0][4].angle) + motor_circle_simple[0][4].circle*8192-claw_init_position[1]);
				if(engineer_control.claw.position_C[1]!=0)
				{
					if(!flag_claw2)
					{
						claw_init_position[1]=engineer_control.claw.position_C[1];
						flag_claw2=1;
					}
				}
				engineer_control.claw.speed_C[1]=((float)RPC_ZERO((S16)(RxMessage.Data[2]<<8|RxMessage.Data[3]),10));
				engineer_control.claw.current_C[1]=(engineer_control.lift.current_C[1]*(1-n)+n*(float)RPC_ZERO((S16)(RxMessage.Data[4]<<8|RxMessage.Data[5]),10));
				canrate.inc.rx_motor[11]++;
			}
			
			
			//一级机械臂
			if(RxMessage.StdId == 0x206)
			{
				engineer_control.arm.position_C[0]  = (S32)(motor_circle_simple[0][5].angle) + motor_circle_simple[0][5].circle*8192-arm_init_position[0];
				if(!flag_arm[0])
				{
					if(engineer_control.arm.position_C[0]!=0)
					{
						arm_init_position[0]=engineer_control.arm.position_C[0];
						flag_arm[0]=1;
					}
				}
				engineer_control.arm.speed_C[0]=(float)RPC_ZERO((S16)(RxMessage.Data[2]<<8|RxMessage.Data[3]),10);
				engineer_control.arm.current_C[0]=engineer_control.arm.current_C[0]*(1-n)+n*(float)RPC_ZERO((S16)(RxMessage.Data[4]<<8|RxMessage.Data[5]),10);
				canrate.inc.rx_motor[12]++;
			}
			if(RxMessage.StdId == 0x207)
			{
				engineer_control.arm.position_C[1]  = (S32)(motor_circle_simple[0][6].angle) + motor_circle_simple[0][6].circle*8192-arm_init_position[1];
				if(!flag_arm[1])
				{
					if(engineer_control.arm.position_C[1]!=1)
					{
						arm_init_position[1]=engineer_control.arm.position_C[1];
						flag_arm[1]=1;
					}
				}
				engineer_control.arm.speed_C[1]=(float)RPC_ZERO((S16)(RxMessage.Data[2]<<8|RxMessage.Data[3]),10);
				engineer_control.arm.current_C[1]=engineer_control.arm.current_C[1]*(1-n)+n*(float)RPC_ZERO((S16)(RxMessage.Data[4]<<8|RxMessage.Data[5]),10);
				canrate.inc.rx_motor[13]++;
			}
			
			//二级机械臂
			if(RxMessage.StdId == 0x208)
			{
				engineer_control.arm_x.position_C  = (S32)(motor_circle_simple[0][7].angle) + motor_circle_simple[0][7].circle*8192-arm_x_init_position;
				if(!flag_arm_x)
				{
					if(engineer_control.arm_x.position_C!=0)
					{
						arm_x_init_position=engineer_control.arm_x.position_C;
						flag_arm_x=1;
					}
				}
				engineer_control.arm_x.speed_C=(float)RPC_ZERO((S16)(RxMessage.Data[2]<<8|RxMessage.Data[3]),10);
				engineer_control.arm_x.current_C=engineer_control.arm_x.current_C*(1-n)+n*(float)RPC_ZERO((S16)(RxMessage.Data[4]<<8|RxMessage.Data[5]),10);
				canrate.inc.rx_motor[14]++;
			}
			
			
			
			//测试
//			if(RxMessage.StdId == 0x206)
//			{
//				canrate.inc.rx_motor[5]++;
//			}
			
			//底盘测试用陀螺仪
			if(RxMessage.StdId == 0x513)
			{
				circle_count_engineer(&temp_chassis_yaw,(U32)((int16_t)(RxMessage.Data[5]<<8|RxMessage.Data[4])),3600,400);
				engineer.chassis.YAW=(temp_chassis_yaw.angle+temp_chassis_yaw.circle*3600);
			}
			
		}
	}
}

float Real_Center = 0,Dir_angle = 0,Dir_angle_adj = 22;	//云台与底盘夹角相关
int Dir_circle = 0;										//云台相对底盘旋转的圈数

int16_t mpu_yaw_speed;				//mpu6050角速度
mcircle_t mpu_hi216_yaw = {0};		//hi216角度和圈数
float imu_yaw,real_mpu_hi216_yaw;	//标度转换后的hi216角度与圈数
U16 usart_imu_rate;					//hi216进中断的频率

s16 CANLASTOUT[4];					//用于滤波的记录上次值的缓冲数组

float holder_yaw_angle_temp;		//偏航电机角度反馈原始值

void	CAN2_RX0_IRQHandler(void)
{
	size_t		i;
	CanRxMsg	RxMessage;
	float n = 0.6;
	float temp_chassis_V_C[4],temp_chassis_I_C[4];

	if(CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
	{
		static u8 flag_relief[2]={0,0},flag_chassis[4]={0,0,0,0},flag_holder=0;
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
		CAN_ClearFlag(CAN2, CAN_FLAG_FMP0); 
		
		CAN_Receive(CAN2,CAN_FIFO0,&RxMessage);	
				
		if ((RxMessage.IDE==CAN_ID_STD)&&(RxMessage.DLC==8))	
		{
			if((RxMessage.StdId >= 0x201) && (RxMessage.StdId <= 0x208))
				circle_count_simple(&(motor_circle_simple[1][RxMessage.StdId-0x201]), ((U16)(RxMessage.Data[0]<<8))|((U16)RxMessage.Data[1]));

			if((RxMessage.StdId >= 0x201) && (RxMessage.StdId <= 0x204))//底盘电机
			{
				for(i = 0;i < 4;i++)
				{
					if(RxMessage.StdId == Driver_ID[i])
					{
						engineer_control.chassis.position_C[i]  = (S32)(motor_circle_simple[1][i].angle) + motor_circle_simple[1][i].circle*8192-chassis_init_position[i];
						if(!flag_chassis[i])
						{
							if(engineer_control.chassis.position_C[i]!=0)
							{
								chassis_init_position[i]=engineer_control.chassis.position_C[i];
								flag_chassis[i]=1;
							}
						}
						engineer_control.chassis.speed_C[i]=(float)RPC_ZERO((S16)(RxMessage.Data[2]<<8|RxMessage.Data[3]),10);
						engineer_control.chassis.current_C[i]=engineer_control.chassis.current_C[i]*(1-n)+n*(float)RPC_ZERO((S16)(RxMessage.Data[4]<<8|RxMessage.Data[5]),10);
						
						
						canrate.inc.rx_motor[i]++;
						break;
					}
				}
			}
			
			//救援
			if(RxMessage.StdId == 0x205)
			{
				engineer_control.relief.position_C[0]  = (S32)(motor_circle_simple[1][4].angle) + motor_circle_simple[1][4].circle*8192-relief_init_position[0];
				if(!flag_relief[0])
				{
					if(engineer_control.relief.position_C[0]!=0)
					{
						relief_init_position[0]=engineer_control.relief.position_C[0];
						flag_relief[0]=1;
					}
				}
				engineer_control.relief.speed_C[0]=(float)RPC_ZERO((S16)(RxMessage.Data[2]<<8|RxMessage.Data[3]),10);
				engineer_control.relief.current_C[0]=engineer_control.relief.current_C[0]*(1-n)+n*(float)RPC_ZERO((S16)(RxMessage.Data[4]<<8|RxMessage.Data[5]),10);
				canrate.inc.rx_motor[4]++;
			}
			if(RxMessage.StdId == 0x206)
			{
				engineer_control.relief.position_C[1]  = (S32)(motor_circle_simple[1][5].angle) + motor_circle_simple[1][5].circle*8192-relief_init_position[1];
				if(!flag_relief[1])
				{
					if(engineer_control.relief.position_C[1]!=1)
					{
						relief_init_position[1]=engineer_control.relief.position_C[1];
						flag_relief[1]=1;
					}
				}
				engineer_control.relief.speed_C[1]=(float)RPC_ZERO((S16)(RxMessage.Data[2]<<8|RxMessage.Data[3]),10);
				engineer_control.relief.current_C[1]=engineer_control.relief.current_C[1]*(1-n)+n*(float)RPC_ZERO((S16)(RxMessage.Data[4]<<8|RxMessage.Data[5]),10);
				canrate.inc.rx_motor[5]++;
			}
			//云台
			if (RxMessage.StdId == 0x207)
			{
//				engineer_control.holder.position_C = (S32)(motor_circle_simple[1][6].angle) + motor_circle_simple[1][6].circle * 8192 - holder_init_position;
//				if (!flag_holder)
//				{
//					if (engineer_control.holder.position_C != 0)
//					{
//						holder_init_position = engineer_control.holder.position_C;
//						flag_holder = 1;
//					}
//				}
				engineer_control.holder.speed_C = (float)RPC_ZERO((S16)(RxMessage.Data[2] << 8 | RxMessage.Data[3]), 10);
				engineer_control.holder.current_C = engineer_control.holder.current_C * (1 - n) + n * (float)RPC_ZERO((S16)(RxMessage.Data[4] << 8 | RxMessage.Data[5]), 10);
				canrate.inc.rx_motor[6]++;
			}

			if(RxMessage.StdId == 0x513)
			{
				circle_count_engineer(&temp_chassis_yaw,(U32)((int16_t)(RxMessage.Data[5]<<8|RxMessage.Data[4])+1800),3600,400);
				engineer.chassis.YAW = (temp_chassis_yaw.angle+temp_chassis_yaw.circle*3600);
				
				engineer_control.holder.position_C = (engineer.chassis.YAW - holder_init_position) * 8192 / 360;
				if (!flag_holder)
				{
					if (engineer_control.holder.position_C != 0)
					{
						holder_init_position = engineer.chassis.YAW;
						flag_holder = 1;
					}
				}
			}
//			//以下为云台电机反馈信号处理，根据不同机器人分别处理
//			if(RxMessage.StdId == 0x205)
//			{
//				holder_yaw_angle_temp = (int16_t)(RxMessage.Data[0]<<8) | (RxMessage.Data[1]);
//				Dir_circle = motor_circle_simple[1][4].circle;
//				Dir_angle = (float)((holder_yaw_angle_temp - Real_Center) * 360 / 8192) + Dir_angle_adj;
//				if(Dir_angle > 180)
//					Dir_angle -= 360;
//				if(Dir_angle < -180)
//					Dir_angle += 360;

//				holder_yaw_angle  = 819 * ((float)(motor_circle_simple[1][4].angle) / 8192 + motor_circle_simple[1][4].circle);
//				yaw_speed = (int16_t)(RxMessage.Data[2]<<8) | (RxMessage.Data[3]);
//				yaw_speed *= 20;
//				yaw_torque = (int16_t)((RxMessage.Data[4])<<8)|(int16_t)(RxMessage.Data[5]);
//					
//				canrate.inc.rx_motor[4]++;
//			}

//			if(RxMessage.StdId == 0x206)
//			{
//				holder_pitch_angle  = (int16_t)(RxMessage.Data[0]<<8) | (RxMessage.Data[1]);
//				holder_pitch_angle *= 4;		//归一化到32767
//				pitch_speed = (int16_t)(RxMessage.Data[2]<<8) | (RxMessage.Data[3]);
//				pitch_speed *= 20;				//大概归一化到了6000，就先这样吧
//				pitch_torque = (int16_t)((RxMessage.Data[4])<<8)|(int16_t)(RxMessage.Data[5]);
//					
//				canrate.inc.rx_motor[5]++;
//			}
//			
//			if(RxMessage.StdId == 0x2B1)
//			{
//				memcpy(&imu_yaw,RxMessage.Data,4);
//				mpu_yaw_speed = RxMessage.Data[4]<<8|RxMessage.Data[5];
//				usart_imu_rate = RxMessage.Data[6]<<8|RxMessage.Data[7];
//				
//				circle_count_simple(&mpu_hi216_yaw,(imu_yaw + 180.0f) * 22.7555556f);
//				real_mpu_hi216_yaw = 819 * (float)(mpu_hi216_yaw.circle + (float)(mpu_hi216_yaw.angle) / 8192);
//			}
		}
	}
}

