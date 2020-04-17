#include "mpu6050.h"
#include "stdio.h"
#include "IIC.h"
#include "imu.h"
#include "filter.h"
#include "filter_test.h"


static uint8_t MPU6050_buff[14];  //���ٶ� ������ �¶� ԭʼ����
MPU_Data_t MPU_Data;  //MPU6050ԭʼ����



/******************************************************************************
* ��  ����void MPU6050_Check()
* ��  �ܣ����IIC�����ϵ�MPU6050�Ƿ����
* ��  ������
* ����ֵ����
* ��  ע����
*******************************************************************************/
void MPU6050_Check(void) 
{ 
	while(!MPU6050_testConnection())
	{
		printf("\n\r MPU6050 no connect... \n\r");
        HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
	}
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
}


/*****************************************************************************
* ��  ����uint8_t MPU6050_ReadByte(uint8_t reg,uint8_t *buf)
* ��  �ܣ���ָ��MPU6050�Ĵ�����ȡһ���ֽ�����
* ��  ����reg�� �Ĵ�����ַ
*         buf:  ��ȡ���ݴ�ŵĵ�ַ
* ����ֵ��1ʧ�� 0�ɹ�
* ��  ע��MPU6050������ֲֻ���I2C�����޸ĳ��Լ��ļ���
*****************************************************************************/
uint8_t MPU6050_ReadByte(uint8_t reg,uint8_t *buf)
{
	if(IIC_ReadByteFromSlave(MPU6050Addr,reg,buf))
		return 1;
	else
		return 0;
}


/******************************************************************************
* ��  ����uint8_tMPU6050_getDeviceID(void)
* ��  �ܣ���ȡ  MPU6050 WHO_AM_I ��ʶ������ 0x68
* ��  ������
* ����ֵ�����ض�ȡ����
* ��  ע����
*******************************************************************************/
uint8_t MPU6050_getDeviceID(void)
{
	uint8_t buf;
	MPU6050_ReadByte(MPU6050_RA_WHO_AM_I, &buf);
	return buf;
}


/******************************************************************************
* ��  ����uint8_tMPU6050_testConnection(void)
* ��  �ܣ����MPU6050 �Ƿ��Ѿ�����
* ��  ������
* ����ֵ��1������ 0δ����
* ��  ע����
*******************************************************************************/
uint8_t MPU6050_testConnection(void) 
{
	if(MPU6050_getDeviceID() == 0x68)  
		return 1;
	else 
		return 0;
}


/*****************************************************************************
* ��  ����uint8_t MPU6050_WriteByte(uint8_t addr,uint8_t reg,uint8_t data)
* ��  �ܣ�дһ���ֽ����ݵ� MPU6050 �Ĵ���
* ��  ����reg�� �Ĵ�����ַ
*         data: Ҫд�������
* ����ֵ��0�ɹ� 1ʧ��
* ��  ע��MPU6050������ֲֻ���I2C�����޸ĳ��Լ��ļ���
*****************************************************************************/
uint8_t MPU6050_WriteByte(uint8_t reg,uint8_t data)
{
	if(IIC_WriteByteToSlave(MPU6050Addr,reg,data))
		return 1;
	else
		return 0;
}


/*****************************************************************************
* ��  ����uint8_t MPU6050_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
* ��  �ܣ���ָ���Ĵ�����ȡָ����������
* ��  ����reg���Ĵ�����ַ
*         len����ȡ���ݳ��� 
*         buf: ��ȡ���ݴ�ŵĵ�ַ
* ����ֵ��0�ɹ� 0ʧ��
* ��  ע��MPU6050������ֲֻ���I2C�����޸ĳ��Լ��ļ���
*****************************************************************************/
uint8_t MPU6050_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(IIC_ReadMultByteFromSlave(MPU6050Addr,reg,len,buf))
		return 1;
	else
		return 0;
}


/******************************************************************************
* ��  ����void MPU6050_Init(void)
* ��  �ܣ���ʼ��MPU6050���빤��״̬
* ��  ������
* ����ֵ����
* ��  ע��DLPF �����Ϊ����Ƶ�ʵ�һ�룡����
*******************************************************************************/
void MPU6050_Init(void)
{
    MPU6050_Check(); //���MPU6050�Ƿ����
    
    MPU6050_WriteByte(MPU6050_RA_PWR_MGMT_1, 0x80); //��λMPU6050
	HAL_Delay(100);
	MPU6050_WriteByte(MPU6050_RA_PWR_MGMT_1, 0x01); //����MPU6050����ѡ��������x��PLLΪʱ��Դ
	MPU6050_WriteByte(MPU6050_RA_INT_ENABLE, 0x00); //��ֹ�ж�
	MPU6050_WriteByte(MPU6050_RA_GYRO_CONFIG, 0x18); //������������+-2000��/�� (��ͷֱ��� = 2^15/2000 = 16.4LSB/��/�� 
	MPU6050_WriteByte(MPU6050_RA_ACCEL_CONFIG, 0x08); //���ٶ�������+-4g   (��ͷֱ��� = 2^15/4g = 8196LSB/g )
	MPU6050_WriteByte(MPU6050_RA_CONFIG, MPU6050_DLPF_BW_20);//�������ݵ����Ϊ1kHZ,DLPF=20Hz 
	MPU6050_WriteByte(MPU6050_RA_SMPLRT_DIV, 0x00);  //������Ƶ (����Ƶ�� = ���������Ƶ�� / (1+DIV)������Ƶ��1000hz��
	MPU6050_WriteByte(MPU6050_RA_INT_PIN_CFG, 0x02); //MPU ��ֱ�ӷ���MPU6050����I2C
    
    HAL_Delay(10);
    MPU_Offset_cali();  //������ٶȺͽ��ٶȵ�ƫ����
    
}


/******************************************************************************
* ��  ����void MPU6050_AccRead(int16_t *accData)
* ��  �ܣ���ȡ���ٶȵ�ԭʼ����
* ��  ����*accData ԭʼ���ݵ�ָ��
* ����ֵ����
* ��  ע����
*******************************************************************************/
void MPU6050_AccRead(int16_t *accData)
{
    uint8_t buf[6];
   	MPU6050_ReadMultBytes(MPU6050_RA_ACCEL_XOUT_H,6,buf);
    accData[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accData[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accData[2] = (int16_t)((buf[4] << 8) | buf[5]);
}


/******************************************************************************
* ��  ����void MPU6050_GyroRead(int16_t *gyroData)
* ��  �ܣ���ȡ�����ǵ�ԭʼ����
* ��  ����*gyroData ԭʼ���ݵ�ָ��
* ����ֵ����
* ��  ע����
*******************************************************************************/
void MPU6050_GyroRead(int16_t *gyroData)
{
    uint8_t buf[6];
	MPU6050_ReadMultBytes(MPU6050_RA_GYRO_XOUT_H, 6, buf);
    gyroData[0] = (int16_t)((buf[0] << 8) | buf[1]) ;
    gyroData[1] = (int16_t)((buf[2] << 8) | buf[3]) ;
    gyroData[2] = (int16_t)((buf[4] << 8) | buf[5]) ;
}


/******************************************************************************
* ��  ����void MPU6050_TempRead(float *tempdata)
* ��  �ܣ��¶�ֵ��ȡ
* ��  ����*tempdata �¶����ݵ�ָ��
* ����ֵ����
* ��  ע����
*******************************************************************************/
void MPU6050_TempRead(float *tempdata)
{
	uint8_t buf[2];
	short data;
	MPU6050_ReadMultBytes(MPU6050_RA_TEMP_OUT_H, 2, buf);
	data = (int16_t)((buf[0] << 8) | buf[1]) ;
	*tempdata = 36.53f + ((float)data/340.0f);
}


/******************************************************************************
* ��  ����void MPU6050_Read(void)
* ��  �ܣ���ȡ�����Ǽ��ٶȼƵ�ԭʼ����
* ��  ������
* ����ֵ����
* ��  ע����
*******************************************************************************/
void MPU6050_Read(void)
{
    
	MPU6050_ReadMultBytes(MPU6050_RA_ACCEL_XOUT_H, 14, MPU6050_buff);// ��ѯ����ȡMPU6050��ԭʼ����
    
    MPU_Data.Acc.X = ((((int16_t)MPU6050_buff[0]) << 8) | MPU6050_buff[1]) - MPU_Data.ax_offset;
    MPU_Data.Acc.Y = ((((int16_t)MPU6050_buff[2]) << 8) | MPU6050_buff[3]) - MPU_Data.ay_offset;
    MPU_Data.Acc.Z = ((((int16_t)MPU6050_buff[4]) << 8) | MPU6050_buff[5]) - MPU_Data.az_offset;
    MPU_Data.temp = (((int16_t)MPU6050_buff[6]) << 8) | MPU6050_buff[7];
    MPU_Data.Gyro.X = ((((int16_t)MPU6050_buff[8]) << 8) | MPU6050_buff[9]) - MPU_Data.gx_offset;
    MPU_Data.Gyro.Y = ((((int16_t)MPU6050_buff[10]) << 8) | MPU6050_buff[11]) - MPU_Data.gy_offset;
    MPU_Data.Gyro.Z = ((((int16_t)MPU6050_buff[12]) << 8) | MPU6050_buff[13]) - MPU_Data.gz_offset;
    
/************************************************�˲�������**************************************************************/
   
    SortAver_FilterAcc(&MPU_Data.Acc, &MPU_Data.Acc_filt, 10); //���ٶ������˲�
    SortAver_FilterGyro(&MPU_Data.Gyro, &MPU_Data.Gyro_filt, 8); //�����������˲�
    
//    MPU_Data.Acc_filt.X = LPF_1st_Filter(&AccLPF_1st[X], (float)MPU_Data.Acc.X); //һ�׵�ͨ�˲���
    
//    MPU_Data.Acc_filt.X = Chebyshev50HzLPF(&filter_test, MPU_Data.Acc.X); //�б�ѩ��II�͵�ͨ�˲���
    
//    MPU_Data.Acc_filt.X = IIR_2st_Filter(&AccLPF_IIR_2st[X], (float)MPU_Data.Acc.X); //���װ�����˹����ͨ�˲���
//    MPU_Data.Acc_filt.Y = IIR_2st_Filter(&AccLPF_IIR_2st[Y], (float)MPU_Data.Acc.Y);
//    MPU_Data.Acc_filt.Z = IIR_2st_Filter(&AccLPF_IIR_2st[Z], (float)MPU_Data.Acc.Z);
    
//    MPU_Data.Acc_filt.X = IIR_4st_Filter(&AccLPF_IIR_4st[X], (float)MPU_Data.Acc.X); //�Ľװ�����˹����ͨ�˲���
//    MPU_Data.Acc_filt.Y = IIR_4st_Filter(&AccLPF_IIR_4st[Y], (float)MPU_Data.Acc.Y);
//    MPU_Data.Acc_filt.Z = IIR_4st_Filter(&AccLPF_IIR_4st[Z], (float)MPU_Data.Acc.Z);
    
//    printf("\n\r %d %f \n\r", MPU_Data.Acc.X, MPU_Data.Acc_filt.X);

/*****************************************************END****************************************************************/ 
    
    /*�����ٶ�ADֵת��Ϊ��m/s^2*/
    IMU_Data.ax = MPU_Data.Acc_filt.X * Acc_Gain * G;
    IMU_Data.ay = MPU_Data.Acc_filt.Y * Acc_Gain * G;
    IMU_Data.az = MPU_Data.Acc_filt.Z * Acc_Gain * G;
    
    /*���¶�ADֵת��Ϊ����*/
    IMU_Data.temp = 36.53f + ((float)MPU_Data.temp/340.0f);
    
    /*�����ٶ�ADֵת��Ϊ��rad/s*/
    IMU_Data.wx = MPU_Data.Gyro.X * Gyro_Gr;
    IMU_Data.wy = MPU_Data.Gyro.Y * Gyro_Gr;
    IMU_Data.wz = MPU_Data.Gyro.Z * Gyro_Gr;
    
}



void MPU_Offset_cali(void)
{
    int i;
	for (i=0; i<300; i++)
    {
        MPU6050_ReadMultBytes(MPU6050_RA_ACCEL_XOUT_H, 14, MPU6050_buff);
        
//        MPU_Data.ax_offset += (((int16_t)MPU6050_buff[0]) << 8) | MPU6050_buff[1];
//        MPU_Data.ay_offset += (((int16_t)MPU6050_buff[2]) << 8) | MPU6050_buff[3];
//        MPU_Data.az_offset += (((int16_t)MPU6050_buff[4]) << 8) | MPU6050_buff[5] - 8192;
        
        MPU_Data.gx_offset += (((int16_t)MPU6050_buff[8]) << 8) | MPU6050_buff[9];
        MPU_Data.gy_offset += (((int16_t)MPU6050_buff[10]) << 8) | MPU6050_buff[11];
        MPU_Data.gz_offset += (((int16_t)MPU6050_buff[12]) << 8) | MPU6050_buff[13];
        
        HAL_Delay(2);
    }
//    MPU_Data.ax_offset = MPU_Data.ax_offset/300;
//    MPU_Data.ay_offset = MPU_Data.ay_offset/300;
//    MPU_Data.az_offset = MPU_Data.az_offset/300;
    
    MPU_Data.ax_offset = 180;
    MPU_Data.ay_offset = 180;
    MPU_Data.az_offset = 7847-8192;
    
    MPU_Data.gx_offset = MPU_Data.gx_offset/300;
    MPU_Data.gy_offset = MPU_Data.gy_offset/300;
    MPU_Data.gz_offset = MPU_Data.gz_offset/300;
    
}










