/**
  * @file 		iic_xuni.h
  * @brief		虚拟iic，以解决多个iic设备时代码重复的问题
  * @author		biu123biu
  * @date		    10.15
  * @copyright    CSU RM. All rights reserved. change from seekfree's soft iic.
  */

/*
@use_note:
	需另外：
	1.管脚初始化,gpio->开漏(因此读取操作宏中的方向改变可以不用管)
	2.定义虚拟iic句柄,并关联:
	eg:
	SIIC_HANDLE mpu6050_handle;
	mpu6050_handle.GPIO_SCL = IMU_SCL_GPIO_Port;//GPIOB
	mpu6050_handle.pin_SCL = IMU_SCL_Pin;//GPIO_PIN_7
	mpu6050_handle.GPIO_SDA = IMU_SDA_GPIO_Port;
	mpu6050_handle.pin_SDA = IMU_SDA_Pin;	
*/

#include "iic_xuni.h"
#include "stm32f4xx.h"
S8  transpin(U16 pin);

#define SDA(pHandle)           GPIO_ReadInputDataBit(pHandle->GPIO_SDA,pHandle->pin_SDA)	//读取IO口电平值
#define SDA0(pHandle)          GPIO_ResetBits(pHandle->GPIO_SDA,pHandle->pin_SDA);			//IO口输出低电平
#define SDA1(pHandle)          GPIO_SetBits(pHandle->GPIO_SDA,pHandle->pin_SDA);			//IO口输出高电平
#define SCL0(pHandle)          GPIO_ResetBits(pHandle->GPIO_SCL,pHandle->pin_SCL);			//IO口输出低电平
#define SCL1(pHandle)          GPIO_SetBits(pHandle->GPIO_SCL,pHandle->pin_SCL);			//IO口输出高电平
#define DIR_OUT(pHandle)       {pHandle->GPIO_SDA->MODER&=~(3<<(2*transpin(pHandle->pin_SDA)));pHandle->GPIO_SDA->MODER|=1<<2*transpin(pHandle->pin_SDA);}
#define DIR_IN(pHandle)        {pHandle->GPIO_SDA->MODER&=~(3<<(2*transpin(pHandle->pin_SDA)));pHandle->GPIO_SDA->MODER|=0<<2*transpin(pHandle->pin_SDA);}

S8  transpin(U16 pin)
{
	S8 i=-1;
	while(pin)
	{
		pin=pin>>1;
		i++;
	}
	return i;
}

//内部数据定义
uint8_t IIC_ad_main; //器件从地址	    
uint8_t IIC_ad_sub;  //器件子地址	   
uint8_t *IIC_buf;    //发送|接收数据缓冲区	    
uint8_t IIC_num;     //发送|接收数据个数	     

#define ack 1      //主应答
#define no_ack 0   //从应答	 


//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC延时
//  @return     void						
//  @since      v1.0
//  Sample usage:				如果IIC通讯失败可以尝试增加j的值
//-------------------------------------------------------------------------------------------------------------------
void simiic_delay(void)
{
	//64为100K的速率(bus频率为100M)
	//特别提示OV7725的通信速率不能太高，最好50K左右，j设置为120通信速率为60K，60K的时候可以正常通信
	//其他IIC器件一般可以400K的通信速率
	uint16_t j=25;//25
	while(j--);
}


//内部使用，用户无需调用
void IIC_start(SIIC_HANDLE *pHandle)
{
	SDA1(pHandle);
	SCL1(pHandle);
	simiic_delay();
	SDA0(pHandle);
	simiic_delay();
	SCL0(pHandle);
}

//内部使用，用户无需调用
void IIC_stop(SIIC_HANDLE *pHandle)
{
	SDA0(pHandle);
	SCL0(pHandle);
	simiic_delay();
	SCL1(pHandle);
	simiic_delay();
	SDA1(pHandle);
	simiic_delay();
}

//主应答(包含ack:SDA=0和no_ack:SDA=0)
//内部使用，用户无需调用
void I2C_SendACK(SIIC_HANDLE *pHandle,unsigned char ack_dat)
{
    SCL0(pHandle);
	simiic_delay();
	if(ack_dat)
	{		
		SDA0(pHandle);
	}
    else
	{
		SDA1(pHandle);
	}

    SCL1(pHandle);
    simiic_delay();
    SCL0(pHandle);
    simiic_delay();
}


static int SCCB_WaitAck(SIIC_HANDLE *pHandle)
{
    SCL0(pHandle);
	DIR_IN(pHandle);
	simiic_delay();
	
	SCL1(pHandle);
    simiic_delay();
	
    if(SDA(pHandle))           //应答为高电平，异常，通信失败
    {
        DIR_OUT(pHandle);
        SCL0(pHandle);
        return 0;
    }
    DIR_OUT(pHandle);
    SCL0(pHandle);
	simiic_delay();
    return 1;
}

//字节发送程序
//发送c(可以是数据也可是地址)，送完后接收从应答
//不考虑从应答位
//内部使用，用户无需调用
void send_ch(SIIC_HANDLE *pHandle,uint8_t c)
{
	uint8_t i = 8;
    while(i--)
    {
        if(c & 0x80)
		{
			SDA1(pHandle);//SDA 输出数据
		}
        else
		{
			SDA0(pHandle);
		}
        c <<= 1;
        simiic_delay();
        SCL1(pHandle);                //SCL 拉高，采集信号
        simiic_delay();
        SCL0(pHandle);                //SCL 时钟线拉低
    }
	SCCB_WaitAck(pHandle);
}

//字节接收程序
//接收器件传来的数据，此程序应配合|主应答函数|使用
//内部使用，用户无需调用
uint8_t read_ch(SIIC_HANDLE *pHandle,uint8_t ack_x)
{
    uint8_t i;
    uint8_t c;
    c=0;
    SCL0(pHandle);
    simiic_delay();
    SDA1(pHandle);             //置数据线为输入方式
    DIR_IN(pHandle);
    for(i=0;i<8;i++)
    {
        simiic_delay();
        SCL0(pHandle);         //置时钟线为低，准备接收数据位
        simiic_delay();
        SCL1(pHandle);         //置时钟线为高，使数据线上数据有效
        simiic_delay();
        c<<=1;
        if(SDA(pHandle)) c+=1;   //读数据位，将接收的数据存c
    }
    DIR_OUT(pHandle);
	SCL0(pHandle);
	simiic_delay();
	I2C_SendACK(pHandle,ack_x);
	
    return c;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC写数据到设备寄存器函数
//  @param      dev_add			设备地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      dat				写入的数据
//  @return     void						
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void simiic_write_reg(SIIC_HANDLE *pHandle,uint8_t dev_add, uint8_t reg, uint8_t dat)
{
	IIC_start(pHandle);
    send_ch(pHandle,(dev_add<<1) | 0x00);   //发送器件地址加写位
	send_ch(pHandle,reg);   				 //发送从机寄存器地址
	send_ch(pHandle,dat);   				 //发送需要写入的数据
	IIC_stop(pHandle);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC从设备寄存器读取数据
//  @param      dev_add			设备地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      type			选择通信方式是IIC  还是 SCCB
//  @return     uint8_t			返回寄存器的数据			
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
uint8_t simiic_read_reg(SIIC_HANDLE *pHandle,uint8_t dev_add, uint8_t reg)
{
	uint8_t dat;
	IIC_start(pHandle);
    send_ch(pHandle,(dev_add<<1) | 0x00);  //发送器件地址加写位
	send_ch(pHandle,reg);   				//发送从机寄存器地址
	
	IIC_start(pHandle);
	send_ch(pHandle,(dev_add<<1) | 0x01);  //发送器件地址加读位
	dat = read_ch(pHandle,no_ack);   				//读取数据
	IIC_stop(pHandle);
	
	return dat;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC读取多字节数据
//  @param      dev_add			设备地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      dat_add			数据保存的地址指针
//  @param      num				读取字节数量
//  @return     uint8_t			返回寄存器的数据			
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void simiic_read_regs(SIIC_HANDLE *pHandle,uint8_t dev_add, uint8_t reg, uint8_t *dat_add, uint8_t num)
{
	IIC_start(pHandle);
    send_ch(pHandle,(dev_add<<1) | 0x00);  //发送器件地址加写位
	send_ch(pHandle,reg);   				//发送从机寄存器地址
	
	IIC_start(pHandle);
	send_ch(pHandle,(dev_add<<1) | 0x01);  //发送器件地址加读位
    while(--num)
    {
        *dat_add = read_ch(pHandle,ack); //读取数据
        dat_add++;
    }
    *dat_add = read_ch(pHandle,no_ack); //读取数据
	IIC_stop(pHandle);
}
