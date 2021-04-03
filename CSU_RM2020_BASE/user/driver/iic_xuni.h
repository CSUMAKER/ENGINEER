/**
  * @file 		iic_xuni.h
  * @brief		����iic���Խ�����iic�豸ʱ�����ظ�������
  * @author		biu123biu
  * @date		    10.15
  * @copyright    CSU RM. All rights reserved. change from seekfree's soft iic.
  */
  
#ifndef __IIC_XUNI_H
#define __IIC_XUNI_H

#include "makos_includes.h"
#include "stm32f4xx.h"
//���IIC�ӿھ���ṹ
typedef struct 
{
	GPIO_TypeDef *	GPIO_SCL;			//SDA������IO
	uint16_t	pin_SCL;			//SCLʱ����IO
	GPIO_TypeDef *	GPIO_SDA;			//SDA������IO
	uint16_t	pin_SDA;			//SCLʱ����IO	
	uint8_t 				DelayUS;			//ͨ����ʱUS
}SIIC_HANDLE;

void IIC_start(SIIC_HANDLE *pHandle);
void IIC_stop(SIIC_HANDLE *pHandle);
void  IIC_ack_main(uint8_t ack_main);
void send_ch(SIIC_HANDLE *pHandle,uint8_t c);
uint8_t read_ch(SIIC_HANDLE *pHandle,uint8_t ack_x);
void simiic_write_reg(SIIC_HANDLE *pHandle,uint8_t dev_add, uint8_t reg, uint8_t dat);
uint8_t simiic_read_reg(SIIC_HANDLE *pHandle,uint8_t dev_add, uint8_t reg);
void simiic_read_regs(SIIC_HANDLE *pHandle,uint8_t dev_add, uint8_t reg, uint8_t *dat_add, uint8_t num);

#endif

