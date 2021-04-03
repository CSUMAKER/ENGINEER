#include "mpu.h"
#include "headfile.h"

MPU_value mpu_value;          //6������
static unsigned char BUF[6];       //�������ݻ�����

void SPI1_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); //ʹ��SPI1ʱ��

    GPIO_InitStructure.GPIO_Pin = MPU_NSS_PIN;    //PA4
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;       //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;         //����
    GPIO_Init(MPU_GPIO, &GPIO_InitStructure);     //��ʼ��
    GPIO_SetBits(MPU_GPIO, MPU_NSS_PIN);

    GPIO_InitStructure.GPIO_Pin = MPU_SCK_PIN | MPU_MISO_PIN | MPU_MOSI_PIN; //PA567���ù������
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                                             //���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                                           //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                       //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                                             //����
    GPIO_Init(MPU_GPIO, &GPIO_InitStructure);                                         //��ʼ��

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1); //PA5����Ϊ SPI1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1); //PA6����Ϊ SPI1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1); //PA7����Ϊ SPI1

    //����ֻ���SPI�ڳ�ʼ��
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, ENABLE); //��λSPI1
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, DISABLE); //ֹͣ��λSPI1

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		                    //����SPI����ģʽ:����Ϊ��SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		                //����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		                      //����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ SPI_CPOL_Low SPI_CPOL_High
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	                      //����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����SPI_CPHA_2Edge
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		                        //NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;//42M/128=328.125KHZ���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	                //ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
    SPI_InitStructure.SPI_CRCPolynomial = 7;	                          //CRCֵ����Ķ���ʽ
    SPI_Init(SPI1, &SPI_InitStructure);                                 //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���

    SPI_Cmd(SPI1, ENABLE);   //ʹ��SPI����
    SPI1_ReadWriteByte(0xff);
}
/***************************************************************/
//SPIx
//TxData:����һ���ֽ�
//����ֵ:data
/***************************************************************/
u8 SPI1_ReadWriteByte(u8 TxData)
{
    u8 retry = 0;
    u8 recive;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //�ȴ�SPI���ͱ�־λ��
    {
        retry++;
        if(retry > 200)
            return 0;
    }
    SPI_I2S_SendData(SPI1, TxData); //��������
    retry = 0;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //�ȴ�SPI���ձ�־λ��
    {
        retry++;
        if(retry > 200)
            return 0;
    }
    recive = SPI_I2S_ReceiveData(SPI1); //��������
    return 	recive;
}
/***************************************************************/
//SPI����
//reg: addr
//value:����
/***************************************************************/
u8 MPU6500_Write_Reg(u8 reg, u8 value)
{
    u8 status;
    MPU_6500_ENABLE;   //	MPU9250_CS=0;  //ƬѡMPU9250
    status = SPI1_ReadWriteByte(reg); //����reg��ַ
    SPI1_ReadWriteByte(value);//��������
    MPU_6500_DISENABLE;//	MPU9250_CS=1;  //ʧ��MPU9250
	delay_us(1);
    return(status);//
}
//---------------------------------------------------------------//
//SPI��ȡ
//reg: addr
u8 MPU6500_Read_Reg(u8 reg)
{
    u8 reg_val;
    MPU_6500_ENABLE;//	MPU9250_CS=0;  //ƬѡMPU9250
    SPI1_ReadWriteByte(reg | 0x80); //reg��ַ+������
    reg_val = SPI1_ReadWriteByte(0xff); //��������
    MPU_6500_DISENABLE;//	MPU9250_CS=1;  //ʧ��MPU9250
	delay_us(1);
    return(reg_val);
}

//****************��ʼ��MPU9250��������Ҫ��ο�pdf�����޸�************************
void Init_MPU6500(void)
{
    SPI1_Init();

	MPU6500_Write_Reg(PWR_MGMT_1, 0x00);	//�������״̬
	MPU6500_Write_Reg(CONFIG, 0x07);      //Internal_Sample_Rate==8K

/*******************Init GYRO and ACCEL******************************/
	MPU6500_Write_Reg(SMPLRT_DIV, 0x07);  //�����ǲ����ʣ�����ֵ��0x07(1kHz) (SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV) )
	MPU6500_Write_Reg(GYRO_CONFIG, 0x13); //�������Լ켰������Χ������ֵ��0x13(���Լ죬1000deg/s,�ص�ͨ�˲�) 0x1B(���Լ죬2000deg/s,�ص�ͨ�˲�)
	MPU6500_Write_Reg(ACCEL_CONFIG, 0x18);//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x18(���Լ죬16G)
	MPU6500_Write_Reg(ACCEL_CONFIG_2, 0x08);//���ټƸ�ͨ�˲�Ƶ�� ����ֵ ��0x08  ��1.13kHz��
	task_delay_ms(50);
}

//************************���ٶȶ�ȡ**************************/
void READ_MPU6500_ACCEL(void)//
{

    BUF[0] = MPU6500_Read_Reg(ACCEL_XOUT_L);
    BUF[1] = MPU6500_Read_Reg(ACCEL_XOUT_H);
    mpu_value.Accel[0] =	(BUF[1] << 8) | BUF[0];
    mpu_value.Accel[0] /= 164; 						 //��ȡ����X������
    BUF[2] = MPU6500_Read_Reg(ACCEL_YOUT_L);
    BUF[3] = MPU6500_Read_Reg(ACCEL_YOUT_H);
    mpu_value.Accel[1] =	(BUF[3] << 8) | BUF[2];
    mpu_value.Accel[1] /= 164; 						 //��ȡ����Y������
    BUF[4] = MPU6500_Read_Reg(ACCEL_ZOUT_L);
    BUF[5] = MPU6500_Read_Reg(ACCEL_ZOUT_H);
    mpu_value.Accel[2] =  (BUF[5] << 8) | BUF[4];
    mpu_value.Accel[2] /= 164; 					    //��ȡ����Z������
}
/**********************�����Ƕ�ȡ*****************************/
void READ_MPU6500_GYRO(void)
{
    BUF[0] = MPU6500_Read_Reg(GYRO_XOUT_L);
    BUF[1] = MPU6500_Read_Reg(GYRO_XOUT_H);
    mpu_value.Gyro[0] =	(BUF[1] << 8) | BUF[0];
//   mpu_value.Gyro[0]/=16.4; 						   //��ȡ����X������

    BUF[2] = MPU6500_Read_Reg(GYRO_YOUT_L);
    BUF[3] = MPU6500_Read_Reg(GYRO_YOUT_H);
    mpu_value.Gyro[1] =	(BUF[3] << 8) | BUF[2];
//   mpu_value.Gyro[1]/=16.4; 						   //��ȡ����Y������
    BUF[4] = MPU6500_Read_Reg(GYRO_ZOUT_L);
    BUF[5] = MPU6500_Read_Reg(GYRO_ZOUT_H);
    mpu_value.Gyro[2] =	(BUF[5] << 8) | BUF[4];
//   mpu_value.Gyro[2]/=16.4; 					       //��ȡ����Z������
}
