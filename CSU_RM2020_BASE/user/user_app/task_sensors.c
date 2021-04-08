//#include "task_sensors.h"
//#include "headfile.h"
//#include "task_engineer_state.h"

//void TIM1_Cap_Init(u32 arr, u16 psc);
//void init_ultrasonic(void);
//void init_laser_switch(void);

////捕获状态
////[7]:0,没有成功的捕获;1,成功捕获到一次.
////[6]:0,还没捕获到低电平;1,已经捕获到低电平了.
////[0]:计数器溢出标志位
//vu8  TIM1CH1_CAPTURE_STA = 0;	//输入捕获状态
//static __align(4) vu32	TIM1CH1_CAPTURE_VAL;	//输入捕获值(TIM2/TIM5是32位)
//static __align(4) vu32	TIM1CH1_CAPTURE_VAL_LAST;	//输入捕获值(TIM2/TIM5是32位)

//vu8  TIM1CH2_CAPTURE_STA = 0;	//输入捕获状态
//static __align(4) vu32	TIM1CH2_CAPTURE_VAL;	//输入捕获值(TIM2/TIM5是32位)
//static __align(4) vu32	TIM1CH2_CAPTURE_VAL_LAST;	//输入捕获值(TIM2/TIM5是32位)

////vu8  TIM1CH3_CAPTURE_STA = 0;	//输入捕获状态
////static __align(4) vu32	TIM1CH3_CAPTURE_VAL;	//输入捕获值(TIM2/TIM5是32位)
////static __align(4) vu32	TIM1CH3_CAPTURE_VAL_LAST;	//输入捕获值(TIM2/TIM5是32位)

////vu8  TIM1CH4_CAPTURE_STA = 0;	//输入捕获状态
////static __align(4) vu32	TIM1CH4_CAPTURE_VAL;	//输入捕获值(TIM2/TIM5是32位)
////static __align(4) vu32	TIM1CH4_CAPTURE_VAL_LAST;	//输入捕获值(TIM2/TIM5是32位)

//vu8  TIM3CH4_CAPTURE_STA = 0;	//输入捕获状态
//static __align(4) vu32	TIM3CH4_CAPTURE_VAL;	//输入捕获值(TIM2/TIM5是32位)
//static __align(4) vu32	TIM3CH4_CAPTURE_VAL_LAST;	//输入捕获值(TIM2/TIM5是32位)

//void task_sensors(void *param)
//{
//	init_laser_switch();
//	init_ultrasonic();
//    TIM1_Cap_Init(0XFFFF, 84 - 1); //以1Mhz的频率计数
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
//		if(TIM1CH1_CAPTURE_STA & 0X80)      //成功捕获到了一次高电平
//        {
//            engineer.sensors.chassis_ultrasound[0]=(float)TIM1CH1_CAPTURE_VAL * 0.5f* 340e-4f;//得到距离 cm
//            TIM1CH1_CAPTURE_STA = 0;			   //开启下一次捕获
//            GPIO_SetBits(GPIOE, GPIO_Pin_8);
//            delay_us(20);
//            GPIO_ResetBits(GPIOE, GPIO_Pin_8);
//        }

//        if(TIM1CH2_CAPTURE_STA & 0X80)      //成功捕获到了一次高电平
//        {
//            engineer.sensors.chassis_ultrasound[1]=(float)TIM1CH2_CAPTURE_VAL * 0.5f* 340e-4f;//得到距离 cm
//            TIM1CH2_CAPTURE_STA = 0;			   //开启下一次捕获
//            GPIO_SetBits(GPIOE, GPIO_Pin_10);
//            delay_us(20);
//            GPIO_ResetBits(GPIOE, GPIO_Pin_10);
//        }

////        if(TIM1CH3_CAPTURE_STA & 0X80)      //成功捕获到了一次高电平
////        {
////            engineer.sensors.chassis_ultrasound[2]=(float)TIM1CH3_CAPTURE_VAL * 0.5f* 340e-4f;//得到距离 cm
////            TIM1CH3_CAPTURE_STA = 0;			   //开启下一次捕获
////            GPIO_SetBits(GPIOE, GPIO_Pin_12);
////            delay_us(20);
////            GPIO_ResetBits(GPIOE, GPIO_Pin_12);
////        }

////        if(TIM1CH4_CAPTURE_STA & 0X80)      //成功捕获到了一次高电平
////        {
////            engineer.sensors.chassis_ultrasound[3]=(float)TIM1CH4_CAPTURE_VAL * 0.5f* 340e-4f;//得到距离 cm
////            TIM1CH4_CAPTURE_STA = 0;			   //开启下一次捕获
////            GPIO_SetBits(GPIOE, GPIO_Pin_15);
////            delay_us(20);
////            GPIO_ResetBits(GPIOE, GPIO_Pin_15);
////        }
////		
////		if(TIM3CH4_CAPTURE_STA & 0X80)      //成功捕获到了一次高电平
////        {
////            engineer.sensors.catch_ultrasound=(float)TIM3CH4_CAPTURE_VAL * 0.5f* 340e-4f;//得到距离 cm
////            TIM3CH4_CAPTURE_STA = 0;			   //开启下一次捕获
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
////定时器1通道1234输入捕获配置
////arr：自动重装值(TIM2,TIM5是32位的!!)
////psc：时钟预分频数
//void TIM1_Cap_Init(u32 arr, u16 psc)
//{
//    TIM_ICInitTypeDef  TIM1_ICInitStructure;
//    GPIO_InitTypeDef GPIO_InitStructure;
//    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//    NVIC_InitTypeDef NVIC_InitStructure;


//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);  	//TIM1时钟使能
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  	//TIM3时钟使能
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE); 	//使能PORTA时钟

//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14; 
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
//    GPIO_Init(GPIOE, &GPIO_InitStructure); //初始化PE9
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
//    GPIO_Init(GPIOC, &GPIO_InitStructure); //初始化PC9

//    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1); //PE9复用位定时器1
//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1); //PE11复用位定时器1
////	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1); //PE13复用位定时器1
////	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1); //PE14复用位定时器1
//	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3); //PE9复用位定时器1


//    TIM_TimeBaseStructure.TIM_Prescaler = psc; //定时器分频
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
//    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
//	TIM_TimeBaseStructure.TIM_Period = arr; //自动重装载值
//    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

//    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
//	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);


//    //初始化TIM1输入捕获参数
//    TIM1_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
//    TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
//    TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
//    TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频
//    TIM1_ICInitStructure.TIM_ICFilter = 0x0f;//IC1F=0000 配置输入滤波器 不滤波
//    TIM_ICInit(TIM1, &TIM1_ICInitStructure);
//	
//	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01 	选择输入端 IC1映射到TI1上
//    TIM_ICInit(TIM1, &TIM1_ICInitStructure);
//	
//	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01 	选择输入端 IC1映射到TI1上
//    TIM_ICInit(TIM1, &TIM1_ICInitStructure);
//	
//	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01 	选择输入端 IC1映射到TI1上
//    TIM_ICInit(TIM1, &TIM1_ICInitStructure);
//	TIM_ICInit(TIM3, &TIM1_ICInitStructure);

//    TIM_ITConfig(TIM1, TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, ENABLE); //允许更新中断 ,允许CC1IE捕获中断
//	TIM_ITConfig(TIM3, TIM_IT_Update|TIM_IT_CC4, ENABLE); //允许更新中断 ,允许CC1IE捕获中断
//	
//    TIM_Cmd(TIM1, ENABLE ); 	//使能定时器1
//	TIM_Cmd(TIM3, ENABLE ); 	//使能定时器1


//    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //抢占优先级3
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
//	
//	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6; //抢占优先级3
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
//	
//	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //抢占优先级3
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
//}

//void TIM1_CC_IRQHandler(void)
//{
//    if(TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET)//捕获1发生捕获事件
//    {
//        if(TIM1CH1_CAPTURE_STA & 0X40)		//捕获到一个下降沿
//        {
//            TIM1CH1_CAPTURE_STA |= 0X80;		//标记成功捕获到一次高电平脉宽
//            TIM1CH1_CAPTURE_VAL = TIM_GetCapture1(TIM1); //获取当前的捕获值.
//            if(TIM1CH1_CAPTURE_STA & 0x01)
//            {
//                TIM1CH1_CAPTURE_VAL += 0XFFFF - TIM1CH1_CAPTURE_VAL_LAST;
//                TIM1CH1_CAPTURE_STA &= 0xfe;
//            }
//            else TIM1CH1_CAPTURE_VAL -= TIM1CH1_CAPTURE_VAL_LAST;
//            TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
//        } else  								//还未开始,第一次捕获上升沿
//        {
//            TIM1CH1_CAPTURE_STA = 0;			//清空
//            TIM1CH1_CAPTURE_VAL = 0;
//            TIM1CH1_CAPTURE_STA |= 0X40;		//标记捕获到了上升沿
//            TIM1CH1_CAPTURE_VAL_LAST = TIM_GetCapture1(TIM1);
//            TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
//        }
//		TIM_ClearITPendingBit(TIM1, TIM_IT_CC1); //清除中断标志位
//	}
///************************************************************************************************************/
//	if(TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET)//捕获2发生捕获事件
//    {
//		if(TIM1CH2_CAPTURE_STA & 0X40)		//捕获到一个下降沿
//        {
//            TIM1CH2_CAPTURE_STA |= 0X80;		//标记成功捕获到一次高电平脉宽
//            TIM1CH2_CAPTURE_VAL = TIM_GetCapture2(TIM1); //获取当前的捕获值.
//            if(TIM1CH2_CAPTURE_STA & 0x01)
//            {
//                TIM1CH2_CAPTURE_VAL += 0XFFFF - TIM1CH2_CAPTURE_VAL_LAST;
//                TIM1CH2_CAPTURE_STA &= 0xfe;
//            }
//            else TIM1CH2_CAPTURE_VAL -= TIM1CH2_CAPTURE_VAL_LAST;
//            TIM_OC2PolarityConfig(TIM1, TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
//        } else  								//还未开始,第一次捕获上升沿
//        {
//            TIM1CH2_CAPTURE_STA = 0;			//清空
//            TIM1CH2_CAPTURE_VAL = 0;
//            TIM1CH2_CAPTURE_STA |= 0X40;		//标记捕获到了上升沿
//            TIM1CH2_CAPTURE_VAL_LAST = TIM_GetCapture2(TIM1);
//            TIM_OC2PolarityConfig(TIM1, TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
//        }
//		TIM_ClearITPendingBit(TIM1, TIM_IT_CC2); //清除中断标志位
//	}
///************************************************************************************************************/
//	if(TIM_GetITStatus(TIM1, TIM_IT_CC3) != RESET)//捕获3发生捕获事件
//    {
//		if(TIM1CH3_CAPTURE_STA & 0X40)		//捕获到一个下降沿
//        {
//            TIM1CH3_CAPTURE_STA |= 0X80;		//标记成功捕获到一次高电平脉宽
//            TIM1CH3_CAPTURE_VAL = TIM_GetCapture3(TIM1); //获取当前的捕获值.
//            if(TIM1CH3_CAPTURE_STA & 0x01)
//            {
//                TIM1CH3_CAPTURE_VAL += 0XFFFF - TIM1CH3_CAPTURE_VAL_LAST;
//                TIM1CH3_CAPTURE_STA &= 0xfe;
//            }
//            else TIM1CH3_CAPTURE_VAL -= TIM1CH3_CAPTURE_VAL_LAST;
//            TIM_OC3PolarityConfig(TIM1, TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
//        } else  								//还未开始,第一次捕获上升沿
//        {
//            TIM1CH3_CAPTURE_STA = 0;			//清空
//            TIM1CH3_CAPTURE_VAL = 0;
//            TIM1CH3_CAPTURE_STA |= 0X40;		//标记捕获到了上升沿
//            TIM1CH3_CAPTURE_VAL_LAST = TIM_GetCapture3(TIM1);
//            TIM_OC3PolarityConfig(TIM1, TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
//        }
//		TIM_ClearITPendingBit(TIM1, TIM_IT_CC3); //清除中断标志位
//	}
///************************************************************************************************************/
//	if(TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET)//捕获4发生捕获事件
//    {
//		if(TIM1CH4_CAPTURE_STA & 0X40)		//捕获到一个下降沿
//        {
//            TIM1CH4_CAPTURE_STA |= 0X80;		//标记成功捕获到一次高电平脉宽
//            TIM1CH4_CAPTURE_VAL = TIM_GetCapture4(TIM1); //获取当前的捕获值.
//            if(TIM1CH4_CAPTURE_STA & 0x01)
//            {
//                TIM1CH4_CAPTURE_VAL += 0XFFFF - TIM1CH4_CAPTURE_VAL_LAST;
//                TIM1CH4_CAPTURE_STA &= 0xfe;
//            }
//            else TIM1CH4_CAPTURE_VAL -= TIM1CH4_CAPTURE_VAL_LAST;
//            TIM_OC4PolarityConfig(TIM1, TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
//        } else  								//还未开始,第一次捕获上升沿
//        {
//            TIM1CH4_CAPTURE_STA = 0;			//清空
//            TIM1CH4_CAPTURE_VAL = 0;
//            TIM1CH4_CAPTURE_STA |= 0X40;		//标记捕获到了上升沿
//            TIM1CH4_CAPTURE_VAL_LAST = TIM_GetCapture4(TIM1);
//            TIM_OC4PolarityConfig(TIM1, TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
//        }
//		TIM_ClearITPendingBit(TIM1, TIM_IT_CC4); //清除中断标志位
//	}
//}
//void TIM1_UP_TIM10_IRQHandler(void)
//{	
//	if(TIM_GetITStatus(TIM1,TIM_IT_Update)==SET)
//    {
//        if(TIM1CH1_CAPTURE_STA & 0X40 && !(TIM1CH1_CAPTURE_STA & 0x80)) //已经捕获到高电平了
//        {
//            TIM1CH1_CAPTURE_STA |= 0x01;
//        }
//		if(TIM1CH2_CAPTURE_STA & 0X40 && !(TIM1CH2_CAPTURE_STA & 0x80)) //已经捕获到高电平了
//        {
//            TIM1CH2_CAPTURE_STA |= 0x01;
//        }
//		if(TIM1CH3_CAPTURE_STA & 0X40 && !(TIM1CH3_CAPTURE_STA & 0x80)) //已经捕获到高电平了
//        {
//            TIM1CH3_CAPTURE_STA |= 0x01;
//        }
//		if(TIM1CH4_CAPTURE_STA & 0X40 && !(TIM1CH4_CAPTURE_STA & 0x80)) //已经捕获到高电平了
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
//		if(TIM3CH4_CAPTURE_STA & 0X40 && !(TIM3CH4_CAPTURE_STA & 0x80)) //已经捕获到高电平了
//        {
//            TIM3CH4_CAPTURE_STA |= 0x01;
//        }
//		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
//	}
//	if(TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)//捕获4发生捕获事件
//    {
//		if(TIM3CH4_CAPTURE_STA & 0X40)		//捕获到一个下降沿
//        {
//            TIM3CH4_CAPTURE_STA |= 0X80;		//标记成功捕获到一次高电平脉宽
//            TIM3CH4_CAPTURE_VAL = TIM_GetCapture4(TIM3); //获取当前的捕获值.
//            if(TIM3CH4_CAPTURE_STA & 0x01)
//            {
//                TIM3CH4_CAPTURE_VAL += 0XFFFF - TIM3CH4_CAPTURE_VAL_LAST;
//                TIM3CH4_CAPTURE_STA &= 0xfe;
//            }
//            else TIM3CH4_CAPTURE_VAL -= TIM3CH4_CAPTURE_VAL_LAST;
//            TIM_OC4PolarityConfig(TIM3, TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
//        } else  								//还未开始,第一次捕获上升沿
//        {
//            TIM3CH4_CAPTURE_STA = 0;			//清空
//            TIM3CH4_CAPTURE_VAL = 0;
//            TIM3CH4_CAPTURE_STA |= 0X40;		//标记捕获到了上升沿
//            TIM3CH4_CAPTURE_VAL_LAST = TIM_GetCapture4(TIM3);
//            TIM_OC4PolarityConfig(TIM3, TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
//        }
//		TIM_ClearITPendingBit(TIM3, TIM_IT_CC4); //清除中断标志位
//	}
//}
