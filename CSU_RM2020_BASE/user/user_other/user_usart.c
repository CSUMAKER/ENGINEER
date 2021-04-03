#include "user_usart.h"
#include "irq_dead_check.h"
#include "usart.h"
#include "can.h"
#include "user_can.h"
#include "task_holder.h"
#include "makos_print.h"
#include "makos_resolver.h"
#include "task_nimin.h"
#include "data_fifo.h"
#include "communicate_nimin.h"
#include "nimin_info.h"
#include "stdarg.h"
#include "task_vision.h"
#include "imu_packet.h"
#include "imu_data_decode.h"

extern u8 VS_Rx_Pos;                           // 用于记录接收缓存的位置（视觉通信）
//extern u8 VS_rx_Msg[vs_rxbuf_len];           //存储 PC->Robot 视觉信息
u8 VS_rx_Msg[8];
extern u8 visual_Enable;

IRQ_CHECK_DEFINE(usart1);
void	USART1_IRQHandler(void)
{
	IRQ_CHECK(usart1);
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}

IRQ_CHECK_DEFINE(usart5);
void	UART5_IRQHandler(void)
{
	IRQ_CHECK(usart5);
	if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(UART5, USART_IT_RXNE);
	}
}


uint8_t Camera_Uart_Buffer[256];
u8 Target_Flag,Target_Flag_last;
S16 camera_num = 0;

static float x_tar_base,x_tar_base_last,holder_x_target2;
//static float y_tar_base,y_tar_base_last;
static float vision_a,v_last;
float a_moment_x,a_moment_y;
float vision_a_k = 4.4;
float vision_a_k_pitch = 5000;
static float k = 0.250;
static float kf_q3 = 0.3,kf_r3 = 1;

u8 flag_fass2 = 1;

u8 flag_pass_x_base,flag_pass_y_base;
#define max(a, b)			(a>b? a:b)
#define min(a, b)			(a<b? a:b)
#define range(x, a, b)		(min(max(x, a), b))//a-最小值   b-最大值

IRQ_CHECK_DEFINE(usart6);
void	USART6_IRQHandler(void)
{
	static u8 count = 0;
	IRQ_CHECK(usart6);
	if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART6, USART_IT_RXNE);
		Camera_Uart_Buffer[count] = USART_ReceiveData(USART6);	//读取接收到的数据
		if(Camera_Uart_Buffer[count] == 0X03 && Camera_Uart_Buffer[count-1] == 0XFC)
		{
			if(Camera_Uart_Buffer[count - 10] == 0X03 && Camera_Uart_Buffer[count - 9] == 0XFC)
			{
				vs_get = 1;
				vs_connect = 1;
				Target_Flag_last = Target_Flag;				
				Target_Flag = Camera_Uart_Buffer[count - 2];
				Camera_Z_offset = (int16_t)(Camera_Uart_Buffer[count - 3] << 8 | Camera_Uart_Buffer[count - 4]);
				Camera_Y_offset = (int16_t)(Camera_Uart_Buffer[count - 5] << 8 | Camera_Uart_Buffer[count - 6]);
				Camera_X_offset = (int16_t)(Camera_Uart_Buffer[count - 7] << 8 | Camera_Uart_Buffer[count - 8]);
								
				count = 0;
				camera_num ++;
				if(Vision_Flag)
				{	
					//********x轴处理**********
					//获取目标位置-模型
					x_tar_base = get_x_tart_base(flag_pass_x_base);
					//获取速度-修正信息
					vision_x_speed = get_speed(x_tar_base,x_tar_base_last);	
					
					vision_a = vision_x_speed - v_last;//加速度
					v_last = vision_x_speed;
					
					if(vision_x_speed > vision_xspeed_limit)
						vision_x_speed = vision_xspeed_limit;
					else if(vision_x_speed < -vision_xspeed_limit)
						vision_x_speed = -vision_xspeed_limit;
					
					a_moment_x = vision_a * vision_a_k;
					
					a_moment_x = range(a_moment_x,-200,200);
					
					vision_x_speed += a_moment_x;
					
					//获取目标位置-修正
					
					if(Target_Flag == 255)//识别到目标
						holder_x_target2 = x_tar_base + vision_x_speed*k;
					else//未识别到目标
						holder_x_target2 = holder_yaw_angle;
					
					if(flag_fass2)
					{
						holder_kalman_init(kf_q3,kf_r3,&vision_yaw_kalman_tar);
						holder_x_target = holder_KalmanFilter_cal(holder_x_target2,&vision_yaw_kalman_tar);
					}
					else
						holder_x_target = holder_x_target2;
					
					x_tar_base_last = x_tar_base;
					
//					//********y轴处理**********
//					y_tar_base = get_y_tart_base();//获取目标位置-模型
//					//获取速度-修正信息
//					vision_y_speed = get_speed_pitch(y_tar_base,y_tar_base_last);
////	
//					v_last_pitch = vision_y_speed;	
//					
//					vision_a_pitch = vision_y_speed - v_last_pitch;
//					
//					if(vision_y_speed > limit)
//						vision_y_speed = limit;
//					else if(vision_y_speed < -limit)
//						vision_y_speed = -limit;
//					
//					vision_y_speed += vision_a_pitch * vision_a_k_pitch;
//					//获取目标位置-修正
//					if(Target_Flag == 255)//识别到目标
//						holder_y_target2 = y_tar_base + vision_y_speed*k;
//					else//未识别到目标
//						holder_y_target2 = holder_pitch_angle;
//					
//						holder_y_target = KalmanFilter2_pitch(holder_y_target2,kf_q3,kf_r3,1);
//					
//					y_tar_base_last = y_tar_base;					
				}
			}
		}
		count ++;
		if(count == 255)
			count = 0;
	}
}

void usart6_send_char(char ch)
{
	while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)==RESET);
	USART_SendData(USART6,(u8)ch);	
}

void usart6_send_string(u8 *buff, u32 len)
{
    while(len--)
    {
        usart6_send_char(*buff);
        buff++;
    }
}

void usart2_send_char(char ch)
{
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
	USART_SendData(USART2,(u8)ch);	
}

void usart2_send_string(u8 *buff, u32 len)
{
    while(len--)
    {
        usart2_send_char(*buff);
        buff++;
    }
}

#define CMD_WARE     3
/**************
函数功能：向山外上位机输出1到8个通道的波形
形参说明：number：变量数目
		  length：变量大小，以字节为单位
备注：调用示例：print_wave(3,2,(uint8_t *)&send_date_1,(uint8_t *)&send_date_2,(uint8_t *)&send_date_3);
**************/
void print_wave(u8 number, u8 length, ...)
{
    va_list ap;
    u8 temp_number;
    uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};
    uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};

    va_start(ap, length);

    usart2_send_string(cmdf, sizeof(cmdf));

    for(temp_number = 0; temp_number < number; temp_number++)
    {
        usart2_send_string(va_arg(ap, u8*), length);
    }

    usart2_send_string(cmdr, sizeof(cmdr));
    va_end(ap);
}

