#include "task_UI.h"
#include "UI_lib.h"
#include "keyboard.h"
#include "UI.h"
#include "oled.h"

void task_keyscan(void);
void task_ui_get_remote(void);

U16 ui_show_remote;

extern unsigned char pic_test[];
extern unsigned char pic_test2[][32];

void    task_UI(void *param)
{
	u8 temp_oled_init=0;//Ϊ0����Ҫ���³�ʼ����Ϊһ��Ҫ���³�ʼ��
	task_insert(task_keyscan, NULL, 3);	
	task_insert(task_ui_get_remote, NULL, 3);	
	
	OLED_DrawBMP(33,0,33+58,7,pic_test,0);//����������ʾ
	 
	task_delay_ms(1500);          
													
    while(1)
    {
		if(flag_oled)//oled����
		{
			if(temp_oled_init)
			{
				task_delay_ms(1000);
				OLED_Init();
			}
			temp_oled_init=0;
			OS_menu();
		}
		else		//oled����
		{
			temp_oled_init=1;
			OLED_WR_Byte(0xAE, OLED_CMD, 1);
		}
		
        task_delay_ms(30);
    }
}

void task_keyscan()
{	
	
	while(1)
	{
		if(flag_oled)
		{
			key_scan();
		}
		
		task_delay_ms(10);
	}
}
void task_ui_get_remote()
{
	p_remote_data	data;
	while(1)
	{
		data=msg_get_read_some(&remote_msg);
		if(data!=NULL)
		{
			ui_show_remote=data->JL_UD;
			msg_read_finish_some(&remote_msg);
		}
		task_delay_ms(20);
	}
}
