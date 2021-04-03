/**
  * @file 		iic_xuni.h
  * @brief		虚拟iic，以解决多个iic设备时代码重复的问题
  * @author		biu123biu
  * @date		    10.15
  * @copyright    CSU RM. All rights reserved. change from seekfree's soft iic.
  */
  
#ifndef __IIC_XUNI_H
#define __IIC_XUNI_H

#include "makos_includes.h"
#include "stm32f4xx.h"
//软件IIC接口句柄结构
typedef struct 
{
	GPIO_TypeDef *	GPIO_SCL;			//SDA数据线IO
	uint16_t	pin_SCL;			//SCL时钟线IO
	GPIO_TypeDef *	GPIO_SDA;			//SDA数据线IO
	uint16_t	pin_SDA;			//SCL时钟线IO	
	uint8_t 				DelayUS;			//通信延时US
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

