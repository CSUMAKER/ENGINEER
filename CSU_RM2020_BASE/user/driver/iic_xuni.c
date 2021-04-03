/**
  * @file 		iic_xuni.h
  * @brief		����iic���Խ�����iic�豸ʱ�����ظ�������
  * @author		biu123biu
  * @date		    10.15
  * @copyright    CSU RM. All rights reserved. change from seekfree's soft iic.
  */

/*
@use_note:
	�����⣺
	1.�ܽų�ʼ��,gpio->��©(��˶�ȡ�������еķ���ı���Բ��ù�)
	2.��������iic���,������:
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

#define SDA(pHandle)           GPIO_ReadInputDataBit(pHandle->GPIO_SDA,pHandle->pin_SDA)	//��ȡIO�ڵ�ƽֵ
#define SDA0(pHandle)          GPIO_ResetBits(pHandle->GPIO_SDA,pHandle->pin_SDA);			//IO������͵�ƽ
#define SDA1(pHandle)          GPIO_SetBits(pHandle->GPIO_SDA,pHandle->pin_SDA);			//IO������ߵ�ƽ
#define SCL0(pHandle)          GPIO_ResetBits(pHandle->GPIO_SCL,pHandle->pin_SCL);			//IO������͵�ƽ
#define SCL1(pHandle)          GPIO_SetBits(pHandle->GPIO_SCL,pHandle->pin_SCL);			//IO������ߵ�ƽ
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

//�ڲ����ݶ���
uint8_t IIC_ad_main; //�����ӵ�ַ	    
uint8_t IIC_ad_sub;  //�����ӵ�ַ	   
uint8_t *IIC_buf;    //����|�������ݻ�����	    
uint8_t IIC_num;     //����|�������ݸ���	     

#define ack 1      //��Ӧ��
#define no_ack 0   //��Ӧ��	 


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ģ��IIC��ʱ
//  @return     void						
//  @since      v1.0
//  Sample usage:				���IICͨѶʧ�ܿ��Գ�������j��ֵ
//-------------------------------------------------------------------------------------------------------------------
void simiic_delay(void)
{
	//64Ϊ100K������(busƵ��Ϊ100M)
	//�ر���ʾOV7725��ͨ�����ʲ���̫�ߣ����50K���ң�j����Ϊ120ͨ������Ϊ60K��60K��ʱ���������ͨ��
	//����IIC����һ�����400K��ͨ������
	uint16_t j=25;//25
	while(j--);
}


//�ڲ�ʹ�ã��û��������
void IIC_start(SIIC_HANDLE *pHandle)
{
	SDA1(pHandle);
	SCL1(pHandle);
	simiic_delay();
	SDA0(pHandle);
	simiic_delay();
	SCL0(pHandle);
}

//�ڲ�ʹ�ã��û��������
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

//��Ӧ��(����ack:SDA=0��no_ack:SDA=0)
//�ڲ�ʹ�ã��û��������
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
	
    if(SDA(pHandle))           //Ӧ��Ϊ�ߵ�ƽ���쳣��ͨ��ʧ��
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

//�ֽڷ��ͳ���
//����c(����������Ҳ���ǵ�ַ)���������մ�Ӧ��
//�����Ǵ�Ӧ��λ
//�ڲ�ʹ�ã��û��������
void send_ch(SIIC_HANDLE *pHandle,uint8_t c)
{
	uint8_t i = 8;
    while(i--)
    {
        if(c & 0x80)
		{
			SDA1(pHandle);//SDA �������
		}
        else
		{
			SDA0(pHandle);
		}
        c <<= 1;
        simiic_delay();
        SCL1(pHandle);                //SCL ���ߣ��ɼ��ź�
        simiic_delay();
        SCL0(pHandle);                //SCL ʱ��������
    }
	SCCB_WaitAck(pHandle);
}

//�ֽڽ��ճ���
//�����������������ݣ��˳���Ӧ���|��Ӧ����|ʹ��
//�ڲ�ʹ�ã��û��������
uint8_t read_ch(SIIC_HANDLE *pHandle,uint8_t ack_x)
{
    uint8_t i;
    uint8_t c;
    c=0;
    SCL0(pHandle);
    simiic_delay();
    SDA1(pHandle);             //��������Ϊ���뷽ʽ
    DIR_IN(pHandle);
    for(i=0;i<8;i++)
    {
        simiic_delay();
        SCL0(pHandle);         //��ʱ����Ϊ�ͣ�׼����������λ
        simiic_delay();
        SCL1(pHandle);         //��ʱ����Ϊ�ߣ�ʹ��������������Ч
        simiic_delay();
        c<<=1;
        if(SDA(pHandle)) c+=1;   //������λ�������յ����ݴ�c
    }
    DIR_OUT(pHandle);
	SCL0(pHandle);
	simiic_delay();
	I2C_SendACK(pHandle,ack_x);
	
    return c;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ģ��IICд���ݵ��豸�Ĵ�������
//  @param      dev_add			�豸��ַ(����λ��ַ)
//  @param      reg				�Ĵ�����ַ
//  @param      dat				д�������
//  @return     void						
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void simiic_write_reg(SIIC_HANDLE *pHandle,uint8_t dev_add, uint8_t reg, uint8_t dat)
{
	IIC_start(pHandle);
    send_ch(pHandle,(dev_add<<1) | 0x00);   //����������ַ��дλ
	send_ch(pHandle,reg);   				 //���ʹӻ��Ĵ�����ַ
	send_ch(pHandle,dat);   				 //������Ҫд�������
	IIC_stop(pHandle);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ģ��IIC���豸�Ĵ�����ȡ����
//  @param      dev_add			�豸��ַ(����λ��ַ)
//  @param      reg				�Ĵ�����ַ
//  @param      type			ѡ��ͨ�ŷ�ʽ��IIC  ���� SCCB
//  @return     uint8_t			���ؼĴ���������			
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
uint8_t simiic_read_reg(SIIC_HANDLE *pHandle,uint8_t dev_add, uint8_t reg)
{
	uint8_t dat;
	IIC_start(pHandle);
    send_ch(pHandle,(dev_add<<1) | 0x00);  //����������ַ��дλ
	send_ch(pHandle,reg);   				//���ʹӻ��Ĵ�����ַ
	
	IIC_start(pHandle);
	send_ch(pHandle,(dev_add<<1) | 0x01);  //����������ַ�Ӷ�λ
	dat = read_ch(pHandle,no_ack);   				//��ȡ����
	IIC_stop(pHandle);
	
	return dat;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ģ��IIC��ȡ���ֽ�����
//  @param      dev_add			�豸��ַ(����λ��ַ)
//  @param      reg				�Ĵ�����ַ
//  @param      dat_add			���ݱ���ĵ�ַָ��
//  @param      num				��ȡ�ֽ�����
//  @return     uint8_t			���ؼĴ���������			
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void simiic_read_regs(SIIC_HANDLE *pHandle,uint8_t dev_add, uint8_t reg, uint8_t *dat_add, uint8_t num)
{
	IIC_start(pHandle);
    send_ch(pHandle,(dev_add<<1) | 0x00);  //����������ַ��дλ
	send_ch(pHandle,reg);   				//���ʹӻ��Ĵ�����ַ
	
	IIC_start(pHandle);
	send_ch(pHandle,(dev_add<<1) | 0x01);  //����������ַ�Ӷ�λ
    while(--num)
    {
        *dat_add = read_ch(pHandle,ack); //��ȡ����
        dat_add++;
    }
    *dat_add = read_ch(pHandle,no_ack); //��ȡ����
	IIC_stop(pHandle);
}
