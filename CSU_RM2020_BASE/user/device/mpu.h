#ifndef _MPU_H
#define _MPU_H

#include "makos_includes.h"


/*SPI1相关引脚定义*/
#define MPU_GPIO               GPIOA                  
#define MPU_GPIO_CLK           RCC_AHB1Periph_GPIOA    

#define MPU_NSS_GPIO           GPIOA
#define MPU_NSS_GPIO_CLK       RCC_AHB1Periph_GPIOA
#define MPU_NSS_PIN            GPIO_Pin_4

#define MPU_SCK_GPIO           GPIOA
#define MPU_SCK_GPIO_CLK       RCC_AHB1Periph_GPIOA
#define MPU_SCK_PIN            GPIO_Pin_5

#define MPU_MISO_GPIO          GPIOA
#define MPU_MISO_GPIO_CLK      RCC_AHB1Periph_GPIOA
#define MPU_MISO_PIN           GPIO_Pin_6

#define MPU_MOSI_GPIO          GPIOA
#define MPU_MOSI_GPIO_CLK      RCC_AHB1Periph_GPIOA
#define MPU_MOSI_PIN           GPIO_Pin_7

// 定义MPU9250内部地址
/*****************************************************************/
#define	SMPLRT_DIV		                    0x19	//陀螺仪采样率
#define	CONFIG			                    0x1A	
#define	GYRO_CONFIG		                    0x1B	
#define	ACCEL_CONFIG	                    0x1C	
#define	ACCEL_CONFIG_2                      0x1D 

//--------------------6axis  reg addr-----------------------//
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

//--------------------other reg addr-----------------------//
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	PWR_MGMT_2		0x6C	//电源管理
#define	WHO_AM_I		  0x75	//ID地址寄存器(正确数值0x71，只读)

/************************SPI CS ********************************/
#define MPU_6500_DISENABLE  GPIOA->BSRRL = GPIO_Pin_4;//片选set
#define MPU_6500_ENABLE  GPIOA->BSRRH = GPIO_Pin_4;//reset

typedef struct{
	short Accel[3];//Accel X,Y,Z
	short Gyro[3];//Gyro X,Y,Z
}MPU_value;
extern MPU_value mpu_value;          //6轴数据

void Init_MPU6500(void);
u8 MPU6500_Write_Reg(u8 reg,u8 value);//SPI写
u8 MPU6500_Read_Reg(u8 reg);//SPI读
void READ_MPU6500_ACCEL(void);//读取加速度
void READ_MPU6500_GYRO(void);//读取陀螺仪

u8 SPI1_ReadWriteByte(u8 TxData);


#endif
