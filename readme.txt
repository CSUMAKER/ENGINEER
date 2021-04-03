csu fyt机器人战队的各机器人间通用代码部分

资源使用情况：
tim1：小摩擦轮pwm输出
tim11:微妙延时用
tim12：中断速率检测用
uart1:遥控器（dma1）
uart2:调试用
uart3:裁判系统（dma2）
uart4:hi216陀螺仪
uart6:视觉通讯
iic1:oled
spi1:mpu6500陀螺仪

更新记录：
2019.8.31 
1.创建此仓库，相较之前的程序加入了task_mems，即单轴陀螺仪程序
2.加入了stm32的dsp库
3.更新了内存管理，现在可以使用task_insert_CCM函数将任务添加至stm32f4的cpu专属ccm内存中，理论上使用ccm内存的运行速度会快于普通内存，需要注意的是ccm内存区只有cpu可以访问，因此使用dma功能的
任务不要添加到ccm内存中
4.更新了mak_pid.c，搬运了去年的云台pid部分内容。
2019.9.18
更新文件结构，需要用户修改的文件全部放在user_app和user_other中，基本分离了操作系统底层与用户文件。之后开发的时候也注意这一点。
2019.9.23
修改了task_remote的机制，用户可以直接在自己的文件中读取remote_mag
调用示例：
	#include "headfile.h"
	void	task_example(void* param)
	{
		p_remote_data	data;
		S16 temp,temp2;
		while(1)
		{
			data=msg_get_read_some(&remote_msg);	//注意这里的读写消息函数必须是_some
			if(data!=NULL)
			{
				temp=((S16)data->LL)-1024;			//LL是新加的遥控器左上方滚轮的值，减去1024后是+-660
				temp2=data->KEY_data.Key_W;			//键盘上W是否被按下，按下为1，未按下是0
				msg_read_finish_some(&remote_msg);
			}
			task_delay_ms(5);
		}
	}
	
2019.10.4
更新了task_ANO即新匿名上位机程序，接口改为全局变量，使用方便。使用说明见ANO.txt

2019.10.5
device中加入rng_led.c，即硬件随机数功能，接口详见rng_led.h（已加入headfile.h中）
user_other中加入mak_filter.c，提供一些常用滤波函数，目前移植了两款网上的二阶卡尔曼滤波函数

2019.11.11
修复了oled交互界面在show界面按确认键程序卡死的bug

2019.11.4
针对新主控更改了6500陀螺仪、串口陀螺仪、oled及按键、can2、usart2、usart3、usart4的相关底层，
使用headfile.h中的宏定义选择硬件接口，UI的用户接口不变。

陀螺仪用户接口更改为全局变量：
#include "task_mpu.h"
IMU_data.gyro_z (float 角速度逆时针为正 +-32767对应+-1000deg/s 1000hz更新)
IMU_data.yaw（float 偏航角逆时针为正 +-180 200hz更新）

2019.11.19
oled更新为可热拔插式
ui界面引入超级电容的工作开关：在主页面长按up和down键改变全局变量bool is_supercup_work 的值用于判断是否开启超级电容，开启超级电容后oled右上角显示scw。（相关头文件已加入headfile.h中，可直接调用）