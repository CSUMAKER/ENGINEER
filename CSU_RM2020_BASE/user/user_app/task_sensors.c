//#include "task_sensors.h"
//#include "headfile.h"
//#include "task_engineer_state.h"

//void TIM1_Cap_Init(u32 arr, u16 psc);
//void init_ultrasonic(void);
//void init_laser_switch(void);

////����״̬
////[7]:0,û�гɹ��Ĳ���;1,�ɹ�����һ��.
////[6]:0,��û���񵽵͵�ƽ;1,�Ѿ����񵽵͵�ƽ��.
////[0]:�����������־λ
//vu8  TIM1CH1_CAPTURE_STA = 0;	//���벶��״̬
//static __align(4) vu32	TIM1CH1_CAPTURE_VAL;	//���벶��ֵ(TIM2/TIM5��32λ)
//static __align(4) vu32	TIM1CH1_CAPTURE_VAL_LAST;	//���벶��ֵ(TIM2/TIM5��32λ)

//vu8  TIM1CH2_CAPTURE_STA = 0;	//���벶��״̬
//static __align(4) vu32	TIM1CH2_CAPTURE_VAL;	//���벶��ֵ(TIM2/TIM5��32λ)
//static __align(4) vu32	TIM1CH2_CAPTURE_VAL_LAST;	//���벶��ֵ(TIM2/TIM5��32λ)

////vu8  TIM1CH3_CAPTURE_STA = 0;	//���벶��״̬
////static __align(4) vu32	TIM1CH3_CAPTURE_VAL;	//���벶��ֵ(TIM2/TIM5��32λ)
////static __align(4) vu32	TIM1CH3_CAPTURE_VAL_LAST;	//���벶��ֵ(TIM2/TIM5��32λ)

////vu8  TIM1CH4_CAPTURE_STA = 0;	//���벶��״̬
////static __align(4) vu32	TIM1CH4_CAPTURE_VAL;	//���벶��ֵ(TIM2/TIM5��32λ)
////static __align(4) vu32	TIM1CH4_CAPTURE_VAL_LAST;	//���벶��ֵ(TIM2/TIM5��32λ)

//vu8  TIM3CH4_CAPTURE_STA = 0;	//���벶��״̬
//static __align(4) vu32	TIM3CH4_CAPTURE_VAL;	//���벶��ֵ(TIM2/TIM5��32λ)
//static __align(4) vu32	TIM3CH4_CAPTURE_VAL_LAST;	//���벶��ֵ(TIM2/TIM5��32λ)

//void task_sensors(void *param)
//{
//	init_laser_switch();
//	init_ultrasonic();
//    TIM1_Cap_Init(0XFFFF, 84 - 1); //��1Mhz��Ƶ�ʼ���
//	
//	
//	while(1)
//	{
//		badass=ammunition_box_towards();
//		
//		engineer.sensors.laser_switch_left=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
//		engineer.sensors.laser_switch_right=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7);
//		engineer.sensors.laser_switch_mid=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5);
//		
//		GPIO_SetBits(GPIOE, GPIO_Pin_8);
//		GPIO_SetBits(GPIOE, GPIO_Pin_10);
//		GPIO_SetBits(GPIOE, GPIO_Pin_12);
//		GPIO_SetBits(GPIOE, GPIO_Pin_15);
//		GPIO_SetBits(GPIOC, GPIO_Pin_8);
//		delay_us(20);
//		GPIO_ResetBits(GPIOE, GPIO_Pin_8);
//		GPIO_ResetBits(GPIOE, GPIO_Pin_10);
//		GPIO_ResetBits(GPIOE, GPIO_Pin_12);
//		GPIO_ResetBits(GPIOE, GPIO_Pin_15);
//		GPIO_ResetBits(GPIOC, GPIO_Pin_8);
//		
//		if(TIM1CH1_CAPTURE_STA & 0X80)      //�ɹ�������һ�θߵ�ƽ
//        {
//            engineer.sensors.chassis_ultrasound[0]=(float)TIM1CH1_CAPTURE_VAL * 0.5f* 340e-4f;//�õ����� cm
//            TIM1CH1_CAPTURE_STA = 0;			   //������һ�β���
//            GPIO_SetBits(GPIOE, GPIO_Pin_8);
//            delay_us(20);
//            GPIO_ResetBits(GPIOE, GPIO_Pin_8);
//        }

//        if(TIM1CH2_CAPTURE_STA & 0X80)      //�ɹ�������һ�θߵ�ƽ
//        {
//            engineer.sensors.chassis_ultrasound[1]=(float)TIM1CH2_CAPTURE_VAL * 0.5f* 340e-4f;//�õ����� cm
//            TIM1CH2_CAPTURE_STA = 0;			   //������һ�β���
//            GPIO_SetBits(GPIOE, GPIO_Pin_10);
//            delay_us(20);
//            GPIO_ResetBits(GPIOE, GPIO_Pin_10);
//        }

////        if(TIM1CH3_CAPTURE_STA & 0X80)      //�ɹ�������һ�θߵ�ƽ
////        {
////            engineer.sensors.chassis_ultrasound[2]=(float)TIM1CH3_CAPTURE_VAL * 0.5f* 340e-4f;//�õ����� cm
////            TIM1CH3_CAPTURE_STA = 0;			   //������һ�β���
////            GPIO_SetBits(GPIOE, GPIO_Pin_12);
////            delay_us(20);
////            GPIO_ResetBits(GPIOE, GPIO_Pin_12);
////        }

////        if(TIM1CH4_CAPTURE_STA & 0X80)      //�ɹ�������һ�θߵ�ƽ
////        {
////            engineer.sensors.chassis_ultrasound[3]=(float)TIM1CH4_CAPTURE_VAL * 0.5f* 340e-4f;//�õ����� cm
////            TIM1CH4_CAPTURE_STA = 0;			   //������һ�β���
////            GPIO_SetBits(GPIOE, GPIO_Pin_15);
////            delay_us(20);
////            GPIO_ResetBits(GPIOE, GPIO_Pin_15);
////        }
////		
////		if(TIM3CH4_CAPTURE_STA & 0X80)      //�ɹ�������һ�θߵ�ƽ
////        {
////            engineer.sensors.catch_ultrasound=(float)TIM3CH4_CAPTURE_VAL * 0.5f* 340e-4f;//�õ����� cm
////            TIM3CH4_CAPTURE_STA = 0;			   //������һ�β���
////            GPIO_SetBits(GPIOC, GPIO_Pin_8);
////            delay_us(20);
////            GPIO_ResetBits(GPIOC, GPIO_Pin_8);
////        }
//		
//		task_delay_ms(2);
//	}
//}
//void init_laser_switch(void)
//{
//	GPIO_InitTypeDef  GPIO_InitStructure;
// 	
// 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7; 
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
// 	GPIO_Init(GPIOA, &GPIO_InitStructure);
//}
//void init_ultrasonic(void)
//{
//    GPIO_InitTypeDef GPIO_InitStructure;

//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

//    GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_8 | GPIO_Pin_10 |GPIO_Pin_12 |GPIO_Pin_15;
//    GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_OUT;
//    GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_100MHz;
//    GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_DOWN;
//    GPIO_Init(GPIOE, &GPIO_InitStructure);
//	
//	GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_8;
//    GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_OUT;
//    GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_100MHz;
//    GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_DOWN;
//    GPIO_Init(GPIOC, &GPIO_InitStructure);

//    GPIO_ResetBits(GPIOE, GPIO_Pin_8);
//    GPIO_ResetBits(GPIOE, GPIO_Pin_10);
//    GPIO_ResetBits(GPIOE, GPIO_Pin_12);
//    GPIO_ResetBits(GPIOE, GPIO_Pin_15);
//	GPIO_ResetBits(GPIOC, GPIO_Pin_8);
//}
////��ʱ��1ͨ��1234���벶������
////arr���Զ���װֵ(TIM2,TIM5��32λ��!!)
////psc��ʱ��Ԥ��Ƶ��
//void TIM1_Cap_Init(u32 arr, u16 psc)
//{
//    TIM_ICInitTypeDef  TIM1_ICInitStructure;
//    GPIO_InitTypeDef GPIO_InitStructure;
//    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//    NVIC_InitTypeDef NVIC_InitStructure;


//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);  	//TIM1ʱ��ʹ��
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  	//TIM3ʱ��ʹ��
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE); 	//ʹ��PORTAʱ��

//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14; 
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
//    GPIO_Init(GPIOE, &GPIO_InitStructure); //��ʼ��PE9
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
//    GPIO_Init(GPIOC, &GPIO_InitStructure); //��ʼ��PC9

//    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1); //PE9����λ��ʱ��1
//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1); //PE11����λ��ʱ��1
////	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1); //PE13����λ��ʱ��1
////	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1); //PE14����λ��ʱ��1
//	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3); //PE9����λ��ʱ��1


//    TIM_TimeBaseStructure.TIM_Prescaler = psc; //��ʱ����Ƶ
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
//    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
//	TIM_TimeBaseStructure.TIM_Period = arr; //�Զ���װ��ֵ
//    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

//    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
//	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);


//    //��ʼ��TIM1���벶�����
//    TIM1_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
//    TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
//    TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
//    TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ
//    TIM1_ICInitStructure.TIM_ICFilter = 0x0f;//IC1F=0000 ���������˲��� ���˲�
//    TIM_ICInit(TIM1, &TIM1_ICInitStructure);
//	
//	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
//    TIM_ICInit(TIM1, &TIM1_ICInitStructure);
//	
//	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
//    TIM_ICInit(TIM1, &TIM1_ICInitStructure);
//	
//	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
//    TIM_ICInit(TIM1, &TIM1_ICInitStructure);
//	TIM_ICInit(TIM3, &TIM1_ICInitStructure);

//    TIM_ITConfig(TIM1, TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, ENABLE); //��������ж� ,����CC1IE�����ж�
//	TIM_ITConfig(TIM3, TIM_IT_Update|TIM_IT_CC4, ENABLE); //��������ж� ,����CC1IE�����ж�
//	
//    TIM_Cmd(TIM1, ENABLE ); 	//ʹ�ܶ�ʱ��1
//	TIM_Cmd(TIM3, ENABLE ); 	//ʹ�ܶ�ʱ��1


//    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //��ռ���ȼ�3
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�3
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//    NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
//	
//	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6; //��ռ���ȼ�3
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�3
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//    NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
//	
//	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //��ռ���ȼ�3
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�3
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//    NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
//}

//void TIM1_CC_IRQHandler(void)
//{
//    if(TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET)//����1���������¼�
//    {
//        if(TIM1CH1_CAPTURE_STA & 0X40)		//����һ���½���
//        {
//            TIM1CH1_CAPTURE_STA |= 0X80;		//��ǳɹ�����һ�θߵ�ƽ����
//            TIM1CH1_CAPTURE_VAL = TIM_GetCapture1(TIM1); //��ȡ��ǰ�Ĳ���ֵ.
//            if(TIM1CH1_CAPTURE_STA & 0x01)
//            {
//                TIM1CH1_CAPTURE_VAL += 0XFFFF - TIM1CH1_CAPTURE_VAL_LAST;
//                TIM1CH1_CAPTURE_STA &= 0xfe;
//            }
//            else TIM1CH1_CAPTURE_VAL -= TIM1CH1_CAPTURE_VAL_LAST;
//            TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
//        } else  								//��δ��ʼ,��һ�β���������
//        {
//            TIM1CH1_CAPTURE_STA = 0;			//���
//            TIM1CH1_CAPTURE_VAL = 0;
//            TIM1CH1_CAPTURE_STA |= 0X40;		//��ǲ�����������
//            TIM1CH1_CAPTURE_VAL_LAST = TIM_GetCapture1(TIM1);
//            TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
//        }
//		TIM_ClearITPendingBit(TIM1, TIM_IT_CC1); //����жϱ�־λ
//	}
///************************************************************************************************************/
//	if(TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET)//����2���������¼�
//    {
//		if(TIM1CH2_CAPTURE_STA & 0X40)		//����һ���½���
//        {
//            TIM1CH2_CAPTURE_STA |= 0X80;		//��ǳɹ�����һ�θߵ�ƽ����
//            TIM1CH2_CAPTURE_VAL = TIM_GetCapture2(TIM1); //��ȡ��ǰ�Ĳ���ֵ.
//            if(TIM1CH2_CAPTURE_STA & 0x01)
//            {
//                TIM1CH2_CAPTURE_VAL += 0XFFFF - TIM1CH2_CAPTURE_VAL_LAST;
//                TIM1CH2_CAPTURE_STA &= 0xfe;
//            }
//            else TIM1CH2_CAPTURE_VAL -= TIM1CH2_CAPTURE_VAL_LAST;
//            TIM_OC2PolarityConfig(TIM1, TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
//        } else  								//��δ��ʼ,��һ�β���������
//        {
//            TIM1CH2_CAPTURE_STA = 0;			//���
//            TIM1CH2_CAPTURE_VAL = 0;
//            TIM1CH2_CAPTURE_STA |= 0X40;		//��ǲ�����������
//            TIM1CH2_CAPTURE_VAL_LAST = TIM_GetCapture2(TIM1);
//            TIM_OC2PolarityConfig(TIM1, TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
//        }
//		TIM_ClearITPendingBit(TIM1, TIM_IT_CC2); //����жϱ�־λ
//	}
///************************************************************************************************************/
//	if(TIM_GetITStatus(TIM1, TIM_IT_CC3) != RESET)//����3���������¼�
//    {
//		if(TIM1CH3_CAPTURE_STA & 0X40)		//����һ���½���
//        {
//            TIM1CH3_CAPTURE_STA |= 0X80;		//��ǳɹ�����һ�θߵ�ƽ����
//            TIM1CH3_CAPTURE_VAL = TIM_GetCapture3(TIM1); //��ȡ��ǰ�Ĳ���ֵ.
//            if(TIM1CH3_CAPTURE_STA & 0x01)
//            {
//                TIM1CH3_CAPTURE_VAL += 0XFFFF - TIM1CH3_CAPTURE_VAL_LAST;
//                TIM1CH3_CAPTURE_STA &= 0xfe;
//            }
//            else TIM1CH3_CAPTURE_VAL -= TIM1CH3_CAPTURE_VAL_LAST;
//            TIM_OC3PolarityConfig(TIM1, TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
//        } else  								//��δ��ʼ,��һ�β���������
//        {
//            TIM1CH3_CAPTURE_STA = 0;			//���
//            TIM1CH3_CAPTURE_VAL = 0;
//            TIM1CH3_CAPTURE_STA |= 0X40;		//��ǲ�����������
//            TIM1CH3_CAPTURE_VAL_LAST = TIM_GetCapture3(TIM1);
//            TIM_OC3PolarityConfig(TIM1, TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
//        }
//		TIM_ClearITPendingBit(TIM1, TIM_IT_CC3); //����жϱ�־λ
//	}
///************************************************************************************************************/
//	if(TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET)//����4���������¼�
//    {
//		if(TIM1CH4_CAPTURE_STA & 0X40)		//����һ���½���
//        {
//            TIM1CH4_CAPTURE_STA |= 0X80;		//��ǳɹ�����һ�θߵ�ƽ����
//            TIM1CH4_CAPTURE_VAL = TIM_GetCapture4(TIM1); //��ȡ��ǰ�Ĳ���ֵ.
//            if(TIM1CH4_CAPTURE_STA & 0x01)
//            {
//                TIM1CH4_CAPTURE_VAL += 0XFFFF - TIM1CH4_CAPTURE_VAL_LAST;
//                TIM1CH4_CAPTURE_STA &= 0xfe;
//            }
//            else TIM1CH4_CAPTURE_VAL -= TIM1CH4_CAPTURE_VAL_LAST;
//            TIM_OC4PolarityConfig(TIM1, TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
//        } else  								//��δ��ʼ,��һ�β���������
//        {
//            TIM1CH4_CAPTURE_STA = 0;			//���
//            TIM1CH4_CAPTURE_VAL = 0;
//            TIM1CH4_CAPTURE_STA |= 0X40;		//��ǲ�����������
//            TIM1CH4_CAPTURE_VAL_LAST = TIM_GetCapture4(TIM1);
//            TIM_OC4PolarityConfig(TIM1, TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
//        }
//		TIM_ClearITPendingBit(TIM1, TIM_IT_CC4); //����жϱ�־λ
//	}
//}
//void TIM1_UP_TIM10_IRQHandler(void)
//{	
//	if(TIM_GetITStatus(TIM1,TIM_IT_Update)==SET)
//    {
//        if(TIM1CH1_CAPTURE_STA & 0X40 && !(TIM1CH1_CAPTURE_STA & 0x80)) //�Ѿ����񵽸ߵ�ƽ��
//        {
//            TIM1CH1_CAPTURE_STA |= 0x01;
//        }
//		if(TIM1CH2_CAPTURE_STA & 0X40 && !(TIM1CH2_CAPTURE_STA & 0x80)) //�Ѿ����񵽸ߵ�ƽ��
//        {
//            TIM1CH2_CAPTURE_STA |= 0x01;
//        }
//		if(TIM1CH3_CAPTURE_STA & 0X40 && !(TIM1CH3_CAPTURE_STA & 0x80)) //�Ѿ����񵽸ߵ�ƽ��
//        {
//            TIM1CH3_CAPTURE_STA |= 0x01;
//        }
//		if(TIM1CH4_CAPTURE_STA & 0X40 && !(TIM1CH4_CAPTURE_STA & 0x80)) //�Ѿ����񵽸ߵ�ƽ��
//        {
//            TIM1CH4_CAPTURE_STA |= 0x01;
//        }
//		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
//	}
//}

//void TIM3_IRQHandler(void)
//{
//	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET)
//    {
//		if(TIM3CH4_CAPTURE_STA & 0X40 && !(TIM3CH4_CAPTURE_STA & 0x80)) //�Ѿ����񵽸ߵ�ƽ��
//        {
//            TIM3CH4_CAPTURE_STA |= 0x01;
//        }
//		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
//	}
//	if(TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)//����4���������¼�
//    {
//		if(TIM3CH4_CAPTURE_STA & 0X40)		//����һ���½���
//        {
//            TIM3CH4_CAPTURE_STA |= 0X80;		//��ǳɹ�����һ�θߵ�ƽ����
//            TIM3CH4_CAPTURE_VAL = TIM_GetCapture4(TIM3); //��ȡ��ǰ�Ĳ���ֵ.
//            if(TIM3CH4_CAPTURE_STA & 0x01)
//            {
//                TIM3CH4_CAPTURE_VAL += 0XFFFF - TIM3CH4_CAPTURE_VAL_LAST;
//                TIM3CH4_CAPTURE_STA &= 0xfe;
//            }
//            else TIM3CH4_CAPTURE_VAL -= TIM3CH4_CAPTURE_VAL_LAST;
//            TIM_OC4PolarityConfig(TIM3, TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
//        } else  								//��δ��ʼ,��һ�β���������
//        {
//            TIM3CH4_CAPTURE_STA = 0;			//���
//            TIM3CH4_CAPTURE_VAL = 0;
//            TIM3CH4_CAPTURE_STA |= 0X40;		//��ǲ�����������
//            TIM3CH4_CAPTURE_VAL_LAST = TIM_GetCapture4(TIM3);
//            TIM_OC4PolarityConfig(TIM3, TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
//        }
//		TIM_ClearITPendingBit(TIM3, TIM_IT_CC4); //����жϱ�־λ
//	}
//}
