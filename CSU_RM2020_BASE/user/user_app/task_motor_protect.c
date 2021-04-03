#include "task_engineer_state.h"
#include "task_engineer_control.h"
#include "headfile.h"
#include "task_sensors.h"
#include "task_motor_protect.h"

////���̵����ֹ��ת���֣��ĸ����ͳһ����
bool chassis_motors=1;							//Ĭ��ʹ�ܣ�������ת��,������ͬ����׸��

//����̧�����ͳһ����
bool lift_motors=1 ;

//һ��ץ�����Ƶ��
bool lift_x_motors=1 ;

//����צ�ӵ��ͳһ����
bool claw_motors=1 ;

//��Ԯ���ͳһ����
bool relief_motors=1;

//��̨���
bool holder_motors=1;//��ʱ��û��д��������

//������е�۵��ͳһ����
bool arm_motors=1;

u8 claw_jam_time;//����צ�ӵ������ʱ�����
u8 lift_x_jam_time;//������Ƶ������ʱ�����
u8 lift_jam_time;// ����̧���������ʱ�����


void task_motor_protect(void *pram)
{
	while(1)
		{
		//צ�ӵ����ֹ��ת����
		if(engineer_control.claw.speed_T[0]>150||engineer_control.claw.speed_T[0]<-150)
		{
			
			if(abs(engineer_control.claw.position_T[0]-engineer_control.claw.position_C[0])>3000 || abs(engineer_control.claw.position_T[1]-engineer_control.claw.position_C[1])>3000)
				if(abs(engineer_control.claw.speed_T[0]-engineer_control.claw.speed_C[0])>700 || abs(engineer_control.claw.speed_T[1]-engineer_control.claw.speed_C[1])>700)
					claw_jam_time++;
			
			if(claw_jam_time>=400)
			claw_motors=0;
		}
		task_delay_ms(5);
		
		
		//ץ�����Ƶ��
		if(engineer_control.lift_x.speed_T>300 || engineer_control.lift_x.speed_T<-300)
		{
			
			if(abs(engineer_control.lift_x.position_T-engineer_control.lift_x.position_C)>3000)
				if(abs(engineer_control.lift_x.speed_T-engineer_control.lift_x.speed_C)>600)
					lift_x_jam_time++;
			
			if(lift_x_jam_time>=200)
				lift_x_motors=0;
		}
		//���¶�ת������δ����
//		
//		//̧�����
//		if(engineer_control.lift.speed_T[0]>150 || engineer_control.lift.speed_T[0]<-150)
//		{
//			
//			if(abs(engineer_control.lift.position_T[0]-engineer_control.lift.position_C[0])>3000 || abs(engineer_control.lift.position_T[1]-engineer_control.lift.position_C[1])>3000)
//				if(abs(engineer_control.lift.speed_T[0]-engineer_control.lift.speed_C[0])>600 || abs(engineer_control.lift.speed_T[1]-engineer_control.lift.speed_C[1])>600)
//					lift_jam_time++;
//			
//			if(lift_jam_time>=200)
//				lift_motors=0;
//		}
//		
		}
}
