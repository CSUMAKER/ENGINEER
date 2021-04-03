/**
  ****************************(C)                 ****************************
  * @file       task_holder.c/h
  * @brief      ��̨��������Ӧ�ò㣬���pitch��yaw�������ɶȵĿ���
  *				Ӳ��ƽ̨ΪSTM32F405RGT6
  * @note       ��Բ�ͬ���������࣬���ò�ͬ�Ŀ��Ʋ���
					����������6020	 Ӣ�ۣ�����6623   ���˻�������3510  �ڱ���3510+6020
				1. ʹ�����������ٶȷ���ʱ���м�����MPU6050��MPU_LOW_PassΪ0��Ȼ�˲��ӳٻᵼ�¿��ƱȽ�����
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2019-1-26       L.Z.M           1.������ɲ�����̨���ƣ�����6020����ʱ��������ʹ�õ����ϢΪ������
  *                                             2.�������3510�������ƣ�����3510��λ�û��õ���Ƕȣ��ٶȻ��������Ƿ�����
  *	 V1.0.1		2019-3-3		L.Z.M			�����ڱ����ƣ�6020ƫ��+3510�������������ǵ����IIC��ʱʹ���Լ��ģ�			
  *  
  *  V2.0.0		2019-9-11		L.Z.M			1.����������򣬴˳���Ӵ�ֻ��������
  *												2.��̨��yaw���Ʋ�����������
  *  V2.0.1		2019-9-12		L.Z.M			����������λ��ѡ����̨����̣�
  * @copyright    CSU RM. All rights reserved.
  ==============================================================================
  */
#include "headfile.h"
#include "task_holder.h"
#include "task_chassis.h"
#include "task_remote.h"
#include "holder_pid.h"
#include "pos_ctrl.h"
#include "task_vision.h"
#include "correct_ctrl.h"
#include "task_mpu.h"

U8 Soilder_ID;
//*********�����궨��**********///
#define	SOLDIER1_PITCH_HIGH				25000		//PITCH�����λ����
#define SOLDIER1_PITCH_LOW				22000
#define SOLDIER1_PITCH_MID				24188
#define	SOLDIER2_PITCH_HIGH				25800		//PITCH�����λ����
#define SOLDIER2_PITCH_LOW				22700
#define SOLDIER2_PITCH_MID				25050
//******************************//

//������������
float	holder_yaw_angle,holder_pitch_angle;			//������������ƫ���ǶȺ͸����Ƕȣ��Ѿ���λת������������
int16_t pitch_torque,yaw_torque,pitch_speed,yaw_speed;	//������������Ť�ص�����ת�٣�����λת����

extern S16 camera_num;//�Ӿ�����֡�ʣ�����ֵ��
u8 camera_max_num;//�Ӿ�����֡�ʣ����ֵ��
//�����ñ�־λ�ͱ���
u8 Vision_Flag = 0,MyScope_Flag = 0;
float	holder_x_target,holder_x_target2,holder_scope_target;	
float	holder_y_target,holder_y_target2;
S16 	holder_x_init,holder_y_init;
#define	ALPHA	0.2	//��ͨ�˲�����
//pid���ֵ
float pitch_ex_v,yaw_ex_v;
int32_t yaw_output,pitch_output;
int32_t yaw_output_last,pitch_output_last;
//can��Ϣ
CanTxMsg	Holder_Motor_set_data;

u8 Send_Flag = 0;//�Ƿ��Ͳ���
#define CMD_WAVE 3	//ɽ����λ����Ӧ�Ĳ���ָ��
/**
  * @brief  ���η��ͺ�������.
  * @param  none.
  * @note   ��Ӧɽ����λ��.
  * @retval ��.
  */
extern float Dir_angle_adj;
extern u8 Target_Flag,holder_fixed_flag;
extern float x_tar_base,x_tar_base_last,y_tar_base,a_moment_x;
extern float yaw_new2020,pitch_new2020,dist_new2020;

void 	task_holder_sendwave(void* param)
{
	uint8_t cmdf[2] = {CMD_WAVE, ~CMD_WAVE};
	uint8_t cmdr[2] = {~CMD_WAVE, CMD_WAVE};
	S32 DataInfo[8];
	while(1)
	{
		//����
		DataInfo[0]	= (S32)(holder_x_target);  
		DataInfo[1] = (S32)0x00;
		DataInfo[2] = (S32)0x00;
		DataInfo[3]	= (S32)(holder_yaw_angle);
		DataInfo[4]	= (S32)(10 * imu_yaw);
		DataInfo[5]	= (S32)(real_mpu_hi216_yaw);
		DataInfo[6] = (S32)yaw_ex_v;
		DataInfo[7]	= (S32)(yaw_speed);
		
		//�Ӿ� - x
//		DataInfo[0]	= (S32)(100 * yaw_new2020);  //����ƫ��
//		DataInfo[1] = (S32)(100 * pitch_new2020);			//ԭʼģ��ֵ
//		DataInfo[2] = (S32)(dist_new2020);		//�Ӿ�Ԥ���ٶ�
//		DataInfo[3]	= (S32)(rm_mpu_real_yaw);	//����ģ��ֵ
//		DataInfo[4]	= (S32)camera_max_num;	//ʵ��λ��
//		DataInfo[5]	= (S32)Dir_angle;
//		DataInfo[6] = (S32)CAN_data[0];
//		DataInfo[7]	= (S32)CAN_data[1];    //��֡�ź�
		
//		//�Ӿ� - y
//		DataInfo[0]	= (S16)(Camera_Y_offset);  //����ƫ��
//		DataInfo[1] = (S16)y_tar_base;			//ԭʼģ��ֵ
//		DataInfo[2] = (S16)vision_y_speed;		//�Ӿ�Ԥ���ٶ�
//		DataInfo[3]	= (S16)(holder_y_target);	//����ģ��ֵ
//		DataInfo[4]	= (S16)holder_pitch_angle;	//ʵ��λ��
//		DataInfo[5]	= (S16)(holder_pitch_angle);
//		DataInfo[6] = (S16)holder_y_target;
//		DataInfo[7]	= (S16)(Target_Flag);    //��֡�ź�		
//						
		usart2_send_string(cmdf, sizeof(cmdf));    
		
		usart2_send_string((uint8_t *)&DataInfo[0],4);
		usart2_send_string((uint8_t *)&DataInfo[1],4);
		usart2_send_string((uint8_t *)&DataInfo[2],4);
		usart2_send_string((uint8_t *)&DataInfo[3],4);
		usart2_send_string((uint8_t *)&DataInfo[4],4);
		usart2_send_string((uint8_t *)&DataInfo[5],4);
		usart2_send_string((uint8_t *)&DataInfo[6],4);
		usart2_send_string((uint8_t *)&DataInfo[7],4);
		
		usart2_send_string(cmdr, sizeof(cmdr)); 
		
		task_delay_ms(10);
	}
}
/**
  * @brief  ��̨�������ݵĳ�ʼ��
  * @param  yaw��pitch����ı���ϵ��
  * @note   .
  * @retval none.
  */
void	holder_data_init(float* Remote_x,float* Remote_y)
{
	Holder_Motor_set_data_init(&Holder_Motor_set_data);

	Dir_angle_adj = 24;
	holder_y_init = SOLDIER1_PITCH_MID + 800;
	
	holder_x_init = real_mpu_hi216_yaw;
	holder_y_target  = holder_y_init;
	holder_x_target  = holder_x_init;
	*Remote_y = -1.85;
	*Remote_x = -0.18;
	
}

/**
  * @brief  Ŀ��ֵ��������ֹ��������
  * @param  
  * @note   .
  * @retval none.
  */
void	holder_target_protect(void)
{
	if(Soilder_ID == 1)
		holder_y_target = SOLDIER_LIMIT(holder_y_target,SOLDIER1_PITCH_LOW,SOLDIER1_PITCH_HIGH);
	else
		holder_y_target = SOLDIER_LIMIT(holder_y_target,SOLDIER2_PITCH_LOW,SOLDIER2_PITCH_HIGH);
			
	if(holder_pitch_angle < 100)	//�����û�й���
		holder_y_target = holder_pitch_angle;		//��ֹû�����ʱ��PID����
	
//	if(!MyScope_Flag)
//	{
//		if(holder_x_target - holder_yaw_angle > 419)
//			while(holder_x_target - holder_yaw_angle > 419)
//				holder_x_target -= 819;
//		else if	(holder_x_target - holder_yaw_angle < -419)
//			while(holder_x_target - holder_yaw_angle < -419)
//				holder_x_target += 819;
//	}
}

void	task_holder_control(void* param);

void	task_holder(void* param)
{
	float	 Remote_PY,Remote_PX;			//����ң�����ķ�Χ����̨ʵ�ʿ���ת���ķ�Χ
	p_remote_data data = NULL;
	p_holder_ctrl holder_ctrl_data = mymalloc(SRAMIN,sizeof(holder_ctrl_t));
	
	task_delay_ms(1500);

	holder_data_init(&Remote_PX,&Remote_PY);
	
	task_delay_ms(1500);

	task_insert(task_holder_control, NULL, 1);
	float last_x_tar = holder_x_init;
	
	while(1)
	{
		data = msg_get_read_some(&remote_msg);
		if(data && !Vision_Flag)
		{
			if(data->SL == RP_S_DOWN)		//��ʾ�ǵ��Կ���״̬
			{
				s32 temp_mouse_x = data->MX * 5.5f;
				s32 temp_mouse_y = data->MY * 3.2f;

				holder_ctrl_data->mouse_x = SOLDIER_LIMIT(temp_mouse_x,-1000,1000);
				holder_ctrl_data->mouse_y = SOLDIER_LIMIT(temp_mouse_y,-1000,1000);
			}
			else							//ң����������
			{
				s32 temp = (data->JR_LR - 1024)*0.9f;
				holder_ctrl_data->mouse_x = SOLDIER_RPC_ZERO(temp,5);
				
				temp = (data->JR_UD - 1024) * 1.5f;
				holder_ctrl_data->mouse_y = SOLDIER_RPC_ZERO(temp,5);
			}

			holder_y_target = holder_y_target - 0.13f * holder_ctrl_data->mouse_y;
			holder_x_target = holder_x_target - 0.03f * holder_ctrl_data->mouse_x;
			
			//����һ�μ򵥵ĵ�ͨ�˲�
			holder_x_target = 0.5f * holder_x_target + 0.5f * last_x_tar;
			last_x_tar = holder_x_target;
			
			msg_read_finish_some(&remote_msg);
		}
		task_delay_ms(5);
	}
}

PID_AbsoluteType_holder	pid_holder_x_speed;
PID_IncrementType	pid_holder_y_speed;
PID_AbsoluteType_section	pid_holder_y_position_section;
PID_AbsoluteType_section	pid_holder_x_position_section;
float yaw_speed_mpu;

void	task_holder_control(void* param)
{
	if(Send_Flag)
		task_insert((task_f)task_holder_sendwave,NULL,3);
	task_delay_ms(20);
	
	holder_pid_init_absolute_section(&pid_holder_x_position_section, 28,28, 28, 0.00, 10, 0,\
																				20, 50, 100, 3, 160, 2800 );
	holder_pid_init_absolute_section(&pid_holder_y_position_section, 1.4, 1.15, 0.8 , 0.089, 0.3, 300,\
																				50, 120, 300, 10,30000 , 600 );
	holder_pid_init_absolute(&pid_holder_x_speed, 1,  0.2,  1,  1500);		//����ʽ 36 0.36
	holder_pid_init_increment(&pid_holder_y_speed, 16, 0.2, 0.0, 1800, 8000);
	
	while(1)
	{			
		holder_target_protect();
		static int count_ticks = 0,camera_count_ticks = 0;
		count_ticks ++;		
		camera_count_ticks ++;
		if(camera_count_ticks == 500)//1s���һ���Ӿ�֡��
		{
			camera_count_ticks = 0;
			camera_max_num = camera_num;
			camera_num = 0;
		}
		if(count_ticks == 5)		//λ�û���������10ms,�ٶȻ�2ms
		{ 
			count_ticks = 0;

			yaw_ex_v = PID_Update_Yaw_index(holder_x_target,real_mpu_hi216_yaw,&pid_holder_x_position_section);
			pitch_ex_v = PID_Update_Yaw_index(holder_y_target, (float)holder_pitch_angle, &pid_holder_y_position_section);
		}
		pitch_output = (S32)PID_IncrementMode_Yaw((float)pitch_ex_v,pitch_speed, &pid_holder_y_speed);		
		long int yaw_out_temp;
		yaw_speed_mpu = -(float)mpu_yaw_speed/10;
		yaw_out_temp = PID_Update_Yaw(yaw_ex_v,yaw_speed_mpu,&pid_holder_x_speed);	
		yaw_out_temp *= 35;
		yaw_output = SOLDIER_LIMIT(yaw_out_temp,-32767,32767);
		
		//����ĵ�ͨ�˲�
		pitch_output = pitch_output*ALPHA + pitch_output_last * (1-ALPHA);
		yaw_output = yaw_output*ALPHA + yaw_output_last * (1-ALPHA);
		pitch_output_last = pitch_output;
		yaw_output_last = yaw_output;
		
		yaw_output = 0;
		pitch_output = 0;   
		Holder_Motor_set_yaw_pitch(yaw_output , pitch_output);

		task_delay_ms(2);
	}
}

/**
  * @brief  ��̨���CAN�ṹ���ʼ��.
  * @param  �ṹ��ĵ�ַ
  * @note   
  * @retval void
  */
void	Holder_Motor_set_data_init(CanTxMsg* data)
{
        data->StdId	= 0x1ff;
        data->IDE	= CAN_ID_STD;
        data->RTR	= CAN_RTR_DATA;
        data->DLC	= 8;
        data->Data[0]	= 0x00;
        data->Data[1]	= 0x00;
        data->Data[2]	= 0x00;
        data->Data[3]	= 0x00;
        data->Data[4]	= 0x00;
        data->Data[5]	= 0x00;
        data->Data[6]	= 0x00;
        data->Data[7]	= 0x00;
}

/**
  * @brief  ����IDΪ205��206�ĵ��������Ϣ
  * @param  ID205��Ϣ��ID206��Ϣ
  * @note   ע��������Ϣ��˳��
  * @retval �ɹ��򷵻�1
  */
U32	Holder_Motor_set_yaw_pitch(S16 yaw, S16 pitch)
{
	pitch = SOLDIER_LIMIT(pitch,-32767,32767);
	yaw = SOLDIER_LIMIT(yaw,-32767,32767);
	Holder_Motor_set_data.Data[0] = (U8)(yaw>>8);
	Holder_Motor_set_data.Data[1] = (U8)(yaw&0xff);
	Holder_Motor_set_data.Data[2] = (U8)(pitch>>8);
	Holder_Motor_set_data.Data[3] = (U8)(pitch&0xff);
	
	CAN_Transmit(CAN2, &Holder_Motor_set_data);
	return	1;
}
