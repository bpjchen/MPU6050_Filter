#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f4xx_hal.h"


#define MPU6050Addr 	0xD0

#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C

#define MPU6050_RA_INT_PIN_CFG      0x37
#define MPU6050_RA_INT_ENABLE       0x38

#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48

#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C
#define MPU6050_RA_WHO_AM_I         0x75

//设置低通滤波
#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

#define G			9.80665f		  // m/s^2
#define Acc_Gain  	0.0001220f		  //加速度变成G (初始化加速度满量程-+4g LSBa = 2*4/65535.0)
#define Gyro_Gain 	0.0609756f		  //角速度变成度 (初始化陀螺仪满量程+-2000 LSBg = 2*2000/65535.0)
#define Gyro_Gr	    0.0010641f		  //角速度变成弧度(3.1415/180 * LSBg)    

typedef struct
{
	int16_t X;
	int16_t Y;
	int16_t Z;
}Int16_xyz;

typedef struct
{
	float X;
	float Y;
	float Z;
}Float_xyz;

/*MPU原始数据，即16位的AD值*/
typedef struct
{
	int16_t mx;  //磁力计
	int16_t my;
	int16_t mz;

	int16_t temp;  //温度
    
    Int16_xyz Acc;       //加速度原始数据
    Float_xyz Acc_filt;  //加速度滤波后数据
    
    Int16_xyz Gyro;      //陀螺仪原始数据
    Float_xyz Gyro_filt; //陀螺仪滤波后数据
	
	int16_t ax_offset;
	int16_t ay_offset;
	int16_t az_offset;

	int16_t gx_offset;
	int16_t gy_offset;
	int16_t gz_offset;
} MPU_Data_t;


extern MPU_Data_t MPU_Data;  //MPU6050原始数据


/*初始化MPU6050*/
void MPU6050_Init(void);
/*检测IIC总线上的MPU6050是否存在*/
void MPU6050_Check(void);
/*检测MPU6050 是否已经连接*/
uint8_t MPU6050_testConnection(void);
/*读取MPU6050原始数据*/
void MPU6050_Read(void);
/*读取加速度的原始数据*/
void MPU6050_AccRead(int16_t *accData);
/*读取陀螺仪的原始数据*/
void MPU6050_GyroRead(int16_t *gyroData);
/*温度值读取*/
void MPU6050_TempRead(float *tempdata);
/*计算初始偏差*/
void MPU_Offset_cali(void);


#endif
