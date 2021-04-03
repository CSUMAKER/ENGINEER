/*
	FILE NAME:	task_remote.c
	DESCRIPTION:	the task dispose remote messages by the control rules
			and send the result to the relatively tasks
	
	EDIT RECORDS:
	---------
	AUTHOR:		FAN YAOWEN
	EDIT TIME:	2017/11/6-21:25
	MODIFICATION:	built the frame of remote task, simply send the msg to chassis and holder.
	---------
	AUTHOR:		FAN YAOWEN
	EDIT TIME:	2018/3/30
	MODIFICATION:	rewrote the shit code of receiver, add and change the romoting logic.
	---------
*/

#include "task_remote.h"
#include "task_init.h"
#include "dbus.h"
#include "task_holder.h"
#include "usart.h"
#include "timer.h"
#include "mak_filter.h"

msg_t	remote_msg;
msg_t	remotekf_msg;
unsigned char magazine = 0;

void	remote_send_msg_init(void)
{
	msg_init(&remote_msg,	1,	sizeof(remote_data_t));
	/*
	msg_init(&remotekf_msg,	1,	sizeof(remote_data_t));
	*/
}


void	remote_data_dispose(p_remote_data tg)
{
	tg->JR_LR	= (U16)((remote_buff[0]		| (remote_buff[1]<<8))				& 0x07ff);
	tg->JR_UD	= (U16)(((remote_buff[1]>>3)	| (remote_buff[2]<<5))				& 0x07ff);
	tg->JL_LR	= (U16)(((remote_buff[2]>>6)	| (remote_buff[3]<<2)	| (remote_buff[4]<<10))	& 0x07ff);
	tg->JL_UD	= (U16)(((remote_buff[4]>>1)	| (remote_buff[5]<<7))				& 0x07ff);
	tg->SL		= (U8)	((remote_buff[5]>>6)							& 0x03);
	tg->SR		= (U8)	((remote_buff[5]>>4)							& 0x03);
	
	tg->LL		= (U16)((remote_buff[16] | (remote_buff[17]<<8))&0x07FF);
	
	tg->MX		= (S16)(remote_buff[6]		| (remote_buff[7]<<8));
	tg->MY		= (S16)(remote_buff[8]		| (remote_buff[9]<<8));
	tg->MCL		= (U8)	remote_buff[12];
	tg->MCR		= (U8)	remote_buff[13];
	
	tg->KEY		= (U16)(remote_buff[14]		| (remote_buff[15]<<8));
	
	tg->KEY_data.Key_W=_K_CHK(W);
	tg->KEY_data.Key_A=_K_CHK(A);
	tg->KEY_data.Key_S=_K_CHK(S);
	tg->KEY_data.Key_D=_K_CHK(D);
	tg->KEY_data.Key_Q=_K_CHK(Q);
	tg->KEY_data.Key_E=_K_CHK(E);
	tg->KEY_data.Key_SHIFT=_K_CHK(SHIFT);
	tg->KEY_data.Key_CTRL=_K_CHK(CTRL);
	tg->KEY_data.Key_R=_K_CHK(R);
	tg->KEY_data.Key_F=_K_CHK(F);
	tg->KEY_data.Key_G=_K_CHK(G);
	tg->KEY_data.Key_Z=_K_CHK(Z);
	tg->KEY_data.Key_X=_K_CHK(X);
	tg->KEY_data.Key_C=_K_CHK(C);
	tg->KEY_data.Key_V=_K_CHK(V);
	tg->KEY_data.Key_B=_K_CHK(B);
}

void	remote_handle(void)
{
	irq_close();
	if(dbus_buff_checked())
	{
		p_remote_data	data = msg_get_write_some(&remote_msg);
		remote_data_dispose(data);
		msg_write_finish_some(&remote_msg);
	}
	irq_restore();
}

static
void	memcopy(void* target, void* source, U32 size)
{
	U32	i;
	U8	*pt = target;
	U8	*ps = source;
	for(i=0; i<size; i++){
		*pt	= *ps;
		pt++;
		ps++;
	}
}

/*
static kalman_filter_init_t remote_kalman_filter_init;
static kalman_filter_t remote_kalman_filter;
float mouse_x_position;
float kf_mouse_x_position;
*/
remote_data_t	task_remote_buff;
uint32_t remote_connect = 0;

void	task_remote(void* param)
{
	p_remote_data	data;
	
	dbus_init(1, remote_handle);
	/*
	//Ô¤²â¾ØÕó
	remote_kalman_filter_init.A_data[0]=1;
	remote_kalman_filter_init.A_data[1]=1;
	remote_kalman_filter_init.A_data[2]=0;
	remote_kalman_filter_init.A_data[3]=1;
	//²âÁ¿¾ØÕó
	remote_kalman_filter_init.H_data[0]=1;
	remote_kalman_filter_init.H_data[1]=0;
	remote_kalman_filter_init.H_data[2]=0;
	remote_kalman_filter_init.H_data[3]=1;
	//Ô¤²âÎó²î
	remote_kalman_filter_init.P_data[0]=0.5;
	remote_kalman_filter_init.P_data[1]=0;
	remote_kalman_filter_init.P_data[2]=0;
	remote_kalman_filter_init.P_data[3]=0.998;
	//´¦ÀíÔëÉùÐ­·½²î
	remote_kalman_filter_init.Q_data[0]=0.5;
	remote_kalman_filter_init.Q_data[1]=0;
	remote_kalman_filter_init.Q_data[2]=0;
	remote_kalman_filter_init.Q_data[3]=1;
	//²âÁ¿ÔëÉùÐ­·½²î
	remote_kalman_filter_init.R_data[0]=5;
	remote_kalman_filter_init.R_data[1]=0;
	remote_kalman_filter_init.R_data[2]=0;
	remote_kalman_filter_init.R_data[3]=5;
	
	kalman_filter_init(&remote_kalman_filter,&remote_kalman_filter_init);
	*/
	while(1)
	{
			
		data	= msg_get_read_some(&remote_msg);
		if(data)
		{
			irq_close();
			memcopy(&task_remote_buff, data, sizeof(remote_data_t));
			msg_read_finish_some(&remote_msg);
			/*
			remote_KF(&task_remote_buff);
			*/
//			remote_test(&task_remote_buff);
			irq_restore();
			remote_connect = 0;
		}else{
			remote_connect++;
			if(remote_connect == 1500)
			{
				remote_connect = 0;
				dbus_init(1, remote_handle);
			}
		}
		task_delay_ms(0);
	}
}

void remote_test(p_remote_data data)
{
//	U16 temp_SL,temp_SR;
	S16 temp_mx,temp_my;
//	temp_SL=(U16)data->SL;
//	temp_SR=(U16)data->SR;
	temp_mx=data->MX;
	temp_my=data->MY;
	temp_my=(S16)canrate.send.dbus;
	print_wave(8,2,&data->JL_LR,&data->JL_UD,&data->JR_LR,&data->JR_UD,&data->LL,&temp_mx,&temp_my,&data->KEY);
}
/*
float * kf_temp;
S32 jscop_temp1,jscop_temp2;
float before_kf_1;
void remote_KF(p_remote_data origin_data)
{
	p_remote_data data = msg_get_write_some(&remotekf_msg);
	//Ò»Ð©¿¨¶ûÂüÂË²¨´¦Àí£¬ÏÈÕ¼¿Ó
	mouse_x_position+=(float)((S16)origin_data->MX)*1;
	kf_temp=kalman_filter_calc(&remote_kalman_filter,mouse_x_position,((S16)origin_data->MX));
	kf_mouse_x_position=kf_temp[0];
	jscop_temp1=mouse_x_position*1;
	jscop_temp2=kf_mouse_x_position*1;
	
	before_kf_1=(float)origin_data->MX;
	print_wave(2,4,&before_kf_1,&kf_temp[1]);
	//KF½áÊø
	data->MX=kf_temp[1];
	
	data->KEY=origin_data->KEY;
	data->KEY_data=origin_data->KEY_data;
	data->SL=origin_data->SL;
	data->SR=origin_data->SR;
	data->MCL=origin_data->MCL;
	data->MCR=origin_data->MCR;
	msg_write_finish_some(&remotekf_msg);
}
*/
