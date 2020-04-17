#include "mpu6050.h"
#include "stdio.h"
#include "IIC.h"
#include "imu.h"
#include "filter.h"
#include "filter_test.h"


static uint8_t MPU6050_buff[14];  //加速度 陀螺仪 温度 原始数据
MPU_Data_t MPU_Data;  //MPU6050原始数据



/******************************************************************************
* 函  数：void MPU6050_Check()
* 功  能：检测IIC总线上的MPU6050是否存在
* 参  数：无
* 返回值：无
* 备  注：无
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
* 函  数：uint8_t MPU6050_ReadByte(uint8_t reg,uint8_t *buf)
* 功  能：从指定MPU6050寄存器读取一个字节数据
* 参  数：reg： 寄存器地址
*         buf:  读取数据存放的地址
* 返回值：1失败 0成功
* 备  注：MPU6050代码移植只需把I2C驱动修改成自己的即可
*****************************************************************************/
uint8_t MPU6050_ReadByte(uint8_t reg,uint8_t *buf)
{
	if(IIC_ReadByteFromSlave(MPU6050Addr,reg,buf))
		return 1;
	else
		return 0;
}


/******************************************************************************
* 函  数：uint8_tMPU6050_getDeviceID(void)
* 功  能：读取  MPU6050 WHO_AM_I 标识将返回 0x68
* 参  数：无
* 返回值：返回读取数据
* 备  注：无
*******************************************************************************/
uint8_t MPU6050_getDeviceID(void)
{
	uint8_t buf;
	MPU6050_ReadByte(MPU6050_RA_WHO_AM_I, &buf);
	return buf;
}


/******************************************************************************
* 函  数：uint8_tMPU6050_testConnection(void)
* 功  能：检测MPU6050 是否已经连接
* 参  数：无
* 返回值：1已连接 0未链接
* 备  注：无
*******************************************************************************/
uint8_t MPU6050_testConnection(void) 
{
	if(MPU6050_getDeviceID() == 0x68)  
		return 1;
	else 
		return 0;
}


/*****************************************************************************
* 函  数：uint8_t MPU6050_WriteByte(uint8_t addr,uint8_t reg,uint8_t data)
* 功  能：写一个字节数据到 MPU6050 寄存器
* 参  数：reg： 寄存器地址
*         data: 要写入的数据
* 返回值：0成功 1失败
* 备  注：MPU6050代码移植只需把I2C驱动修改成自己的即可
*****************************************************************************/
uint8_t MPU6050_WriteByte(uint8_t reg,uint8_t data)
{
	if(IIC_WriteByteToSlave(MPU6050Addr,reg,data))
		return 1;
	else
		return 0;
}


/*****************************************************************************
* 函  数：uint8_t MPU6050_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
* 功  能：从指定寄存器读取指定长度数据
* 参  数：reg：寄存器地址
*         len：读取数据长度 
*         buf: 读取数据存放的地址
* 返回值：0成功 0失败
* 备  注：MPU6050代码移植只需把I2C驱动修改成自己的即可
*****************************************************************************/
uint8_t MPU6050_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(IIC_ReadMultByteFromSlave(MPU6050Addr,reg,len,buf))
		return 1;
	else
		return 0;
}


/******************************************************************************
* 函  数：void MPU6050_Init(void)
* 功  能：初始化MPU6050进入工作状态
* 参  数：无
* 返回值：无
* 备  注：DLPF 最好设为采样频率的一半！！！
*******************************************************************************/
void MPU6050_Init(void)
{
    MPU6050_Check(); //检测MPU6050是否存在
    
    MPU6050_WriteByte(MPU6050_RA_PWR_MGMT_1, 0x80); //复位MPU6050
	HAL_Delay(100);
	MPU6050_WriteByte(MPU6050_RA_PWR_MGMT_1, 0x01); //唤醒MPU6050，并选择陀螺仪x轴PLL为时钟源
	MPU6050_WriteByte(MPU6050_RA_INT_ENABLE, 0x00); //禁止中断
	MPU6050_WriteByte(MPU6050_RA_GYRO_CONFIG, 0x18); //陀螺仪满量程+-2000度/秒 (最低分辨率 = 2^15/2000 = 16.4LSB/度/秒 
	MPU6050_WriteByte(MPU6050_RA_ACCEL_CONFIG, 0x08); //加速度满量程+-4g   (最低分辨率 = 2^15/4g = 8196LSB/g )
	MPU6050_WriteByte(MPU6050_RA_CONFIG, MPU6050_DLPF_BW_20);//设置陀螺的输出为1kHZ,DLPF=20Hz 
	MPU6050_WriteByte(MPU6050_RA_SMPLRT_DIV, 0x00);  //采样分频 (采样频率 = 陀螺仪输出频率 / (1+DIV)，采样频率1000hz）
	MPU6050_WriteByte(MPU6050_RA_INT_PIN_CFG, 0x02); //MPU 可直接访问MPU6050辅助I2C
    
    HAL_Delay(10);
    MPU_Offset_cali();  //计算加速度和角速度的偏移量
    
}


/******************************************************************************
* 函  数：void MPU6050_AccRead(int16_t *accData)
* 功  能：读取加速度的原始数据
* 参  数：*accData 原始数据的指针
* 返回值：无
* 备  注：无
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
* 函  数：void MPU6050_GyroRead(int16_t *gyroData)
* 功  能：读取陀螺仪的原始数据
* 参  数：*gyroData 原始数据的指针
* 返回值：无
* 备  注：无
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
* 函  数：void MPU6050_TempRead(float *tempdata)
* 功  能：温度值读取
* 参  数：*tempdata 温度数据的指针
* 返回值：无
* 备  注：无
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
* 函  数：void MPU6050_Read(void)
* 功  能：读取陀螺仪加速度计的原始数据
* 参  数：无
* 返回值：无
* 备  注：无
*******************************************************************************/
void MPU6050_Read(void)
{
    
	MPU6050_ReadMultBytes(MPU6050_RA_ACCEL_XOUT_H, 14, MPU6050_buff);// 查询法读取MPU6050的原始数据
    
    MPU_Data.Acc.X = ((((int16_t)MPU6050_buff[0]) << 8) | MPU6050_buff[1]) - MPU_Data.ax_offset;
    MPU_Data.Acc.Y = ((((int16_t)MPU6050_buff[2]) << 8) | MPU6050_buff[3]) - MPU_Data.ay_offset;
    MPU_Data.Acc.Z = ((((int16_t)MPU6050_buff[4]) << 8) | MPU6050_buff[5]) - MPU_Data.az_offset;
    MPU_Data.temp = (((int16_t)MPU6050_buff[6]) << 8) | MPU6050_buff[7];
    MPU_Data.Gyro.X = ((((int16_t)MPU6050_buff[8]) << 8) | MPU6050_buff[9]) - MPU_Data.gx_offset;
    MPU_Data.Gyro.Y = ((((int16_t)MPU6050_buff[10]) << 8) | MPU6050_buff[11]) - MPU_Data.gy_offset;
    MPU_Data.Gyro.Z = ((((int16_t)MPU6050_buff[12]) << 8) | MPU6050_buff[13]) - MPU_Data.gz_offset;
    
/************************************************滤波器测试**************************************************************/
   
    SortAver_FilterAcc(&MPU_Data.Acc, &MPU_Data.Acc_filt, 10); //加速度数据滤波
    SortAver_FilterGyro(&MPU_Data.Gyro, &MPU_Data.Gyro_filt, 8); //陀螺仪数据滤波
    
//    MPU_Data.Acc_filt.X = LPF_1st_Filter(&AccLPF_1st[X], (float)MPU_Data.Acc.X); //一阶低通滤波器
    
//    MPU_Data.Acc_filt.X = Chebyshev50HzLPF(&filter_test, MPU_Data.Acc.X); //切比雪夫II型低通滤波器
    
//    MPU_Data.Acc_filt.X = IIR_2st_Filter(&AccLPF_IIR_2st[X], (float)MPU_Data.Acc.X); //二阶巴特沃斯低能通滤波器
//    MPU_Data.Acc_filt.Y = IIR_2st_Filter(&AccLPF_IIR_2st[Y], (float)MPU_Data.Acc.Y);
//    MPU_Data.Acc_filt.Z = IIR_2st_Filter(&AccLPF_IIR_2st[Z], (float)MPU_Data.Acc.Z);
    
//    MPU_Data.Acc_filt.X = IIR_4st_Filter(&AccLPF_IIR_4st[X], (float)MPU_Data.Acc.X); //四阶巴特沃斯低能通滤波器
//    MPU_Data.Acc_filt.Y = IIR_4st_Filter(&AccLPF_IIR_4st[Y], (float)MPU_Data.Acc.Y);
//    MPU_Data.Acc_filt.Z = IIR_4st_Filter(&AccLPF_IIR_4st[Z], (float)MPU_Data.Acc.Z);
    
//    printf("\n\r %d %f \n\r", MPU_Data.Acc.X, MPU_Data.Acc_filt.X);

/*****************************************************END****************************************************************/ 
    
    /*将加速度AD值转化为：m/s^2*/
    IMU_Data.ax = MPU_Data.Acc_filt.X * Acc_Gain * G;
    IMU_Data.ay = MPU_Data.Acc_filt.Y * Acc_Gain * G;
    IMU_Data.az = MPU_Data.Acc_filt.Z * Acc_Gain * G;
    
    /*将温度AD值转化为：℃*/
    IMU_Data.temp = 36.53f + ((float)MPU_Data.temp/340.0f);
    
    /*将角速度AD值转化为：rad/s*/
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










