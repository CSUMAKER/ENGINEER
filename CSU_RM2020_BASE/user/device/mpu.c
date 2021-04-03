#include "mpu.h"
#include "headfile.h"

MPU_value mpu_value;          //6轴数据
static unsigned char BUF[6];       //接收数据缓存区

void SPI1_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); //使能SPI1时钟

    GPIO_InitStructure.GPIO_Pin = MPU_NSS_PIN;    //PA4
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //输出
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;       //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;         //上拉
    GPIO_Init(MPU_GPIO, &GPIO_InitStructure);     //初始化
    GPIO_SetBits(MPU_GPIO, MPU_NSS_PIN);

    GPIO_InitStructure.GPIO_Pin = MPU_SCK_PIN | MPU_MISO_PIN | MPU_MOSI_PIN; //PA567复用功能输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                                             //复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                                           //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                       //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                                             //上拉
    GPIO_Init(MPU_GPIO, &GPIO_InitStructure);                                         //初始化

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1); //PA5复用为 SPI1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1); //PA6复用为 SPI1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1); //PA7复用为 SPI1

    //这里只针对SPI口初始化
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, ENABLE); //复位SPI1
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, DISABLE); //停止复位SPI1

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		                    //设置SPI工作模式:设置为主SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		                //设置SPI的数据大小:SPI发送接收8位帧结构
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		                      //串行同步时钟的空闲状态为高电平 SPI_CPOL_Low SPI_CPOL_High
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	                      //串行同步时钟的第二个跳变沿（上升或下降）数据被采样SPI_CPHA_2Edge
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		                        //NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;//42M/128=328.125KHZ定义波特率预分频的值:波特率预分频值为256
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	                //指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
    SPI_InitStructure.SPI_CRCPolynomial = 7;	                          //CRC值计算的多项式
    SPI_Init(SPI1, &SPI_InitStructure);                                 //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

    SPI_Cmd(SPI1, ENABLE);   //使能SPI外设
    SPI1_ReadWriteByte(0xff);
}
/***************************************************************/
//SPIx
//TxData:发送一个字节
//返回值:data
/***************************************************************/
u8 SPI1_ReadWriteByte(u8 TxData)
{
    u8 retry = 0;
    u8 recive;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //等待SPI发送标志位空
    {
        retry++;
        if(retry > 200)
            return 0;
    }
    SPI_I2S_SendData(SPI1, TxData); //发送数据
    retry = 0;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //等待SPI接收标志位空
    {
        retry++;
        if(retry > 200)
            return 0;
    }
    recive = SPI_I2S_ReceiveData(SPI1); //接收数据
    return 	recive;
}
/***************************************************************/
//SPI发送
//reg: addr
//value:数据
/***************************************************************/
u8 MPU6500_Write_Reg(u8 reg, u8 value)
{
    u8 status;
    MPU_6500_ENABLE;   //	MPU9250_CS=0;  //片选MPU9250
    status = SPI1_ReadWriteByte(reg); //发送reg地址
    SPI1_ReadWriteByte(value);//发送数据
    MPU_6500_DISENABLE;//	MPU9250_CS=1;  //失能MPU9250
	delay_us(1);
    return(status);//
}
//---------------------------------------------------------------//
//SPI读取
//reg: addr
u8 MPU6500_Read_Reg(u8 reg)
{
    u8 reg_val;
    MPU_6500_ENABLE;//	MPU9250_CS=0;  //片选MPU9250
    SPI1_ReadWriteByte(reg | 0x80); //reg地址+读命令
    reg_val = SPI1_ReadWriteByte(0xff); //任意数据
    MPU_6500_DISENABLE;//	MPU9250_CS=1;  //失能MPU9250
	delay_us(1);
    return(reg_val);
}

//****************初始化MPU9250，根据需要请参考pdf进行修改************************
void Init_MPU6500(void)
{
    SPI1_Init();

	MPU6500_Write_Reg(PWR_MGMT_1, 0x00);	//解除休眠状态
	MPU6500_Write_Reg(CONFIG, 0x07);      //Internal_Sample_Rate==8K

/*******************Init GYRO and ACCEL******************************/
	MPU6500_Write_Reg(SMPLRT_DIV, 0x07);  //陀螺仪采样率，典型值：0x07(1kHz) (SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV) )
	MPU6500_Write_Reg(GYRO_CONFIG, 0x13); //陀螺仪自检及测量范围，典型值：0x13(不自检，1000deg/s,关低通滤波) 0x1B(不自检，2000deg/s,关低通滤波)
	MPU6500_Write_Reg(ACCEL_CONFIG, 0x18);//加速计自检、测量范围及高通滤波频率，典型值：0x18(不自检，16G)
	MPU6500_Write_Reg(ACCEL_CONFIG_2, 0x08);//加速计高通滤波频率 典型值 ：0x08  （1.13kHz）
	task_delay_ms(50);
}

//************************加速度读取**************************/
void READ_MPU6500_ACCEL(void)//
{

    BUF[0] = MPU6500_Read_Reg(ACCEL_XOUT_L);
    BUF[1] = MPU6500_Read_Reg(ACCEL_XOUT_H);
    mpu_value.Accel[0] =	(BUF[1] << 8) | BUF[0];
    mpu_value.Accel[0] /= 164; 						 //读取计算X轴数据
    BUF[2] = MPU6500_Read_Reg(ACCEL_YOUT_L);
    BUF[3] = MPU6500_Read_Reg(ACCEL_YOUT_H);
    mpu_value.Accel[1] =	(BUF[3] << 8) | BUF[2];
    mpu_value.Accel[1] /= 164; 						 //读取计算Y轴数据
    BUF[4] = MPU6500_Read_Reg(ACCEL_ZOUT_L);
    BUF[5] = MPU6500_Read_Reg(ACCEL_ZOUT_H);
    mpu_value.Accel[2] =  (BUF[5] << 8) | BUF[4];
    mpu_value.Accel[2] /= 164; 					    //读取计算Z轴数据
}
/**********************陀螺仪读取*****************************/
void READ_MPU6500_GYRO(void)
{
    BUF[0] = MPU6500_Read_Reg(GYRO_XOUT_L);
    BUF[1] = MPU6500_Read_Reg(GYRO_XOUT_H);
    mpu_value.Gyro[0] =	(BUF[1] << 8) | BUF[0];
//   mpu_value.Gyro[0]/=16.4; 						   //读取计算X轴数据

    BUF[2] = MPU6500_Read_Reg(GYRO_YOUT_L);
    BUF[3] = MPU6500_Read_Reg(GYRO_YOUT_H);
    mpu_value.Gyro[1] =	(BUF[3] << 8) | BUF[2];
//   mpu_value.Gyro[1]/=16.4; 						   //读取计算Y轴数据
    BUF[4] = MPU6500_Read_Reg(GYRO_ZOUT_L);
    BUF[5] = MPU6500_Read_Reg(GYRO_ZOUT_H);
    mpu_value.Gyro[2] =	(BUF[5] << 8) | BUF[4];
//   mpu_value.Gyro[2]/=16.4; 					       //读取计算Z轴数据
}
