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

//���õ�ͨ�˲�
#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

#define G			9.80665f		  // m/s^2
#define Acc_Gain  	0.0001220f		  //���ٶȱ��G (��ʼ�����ٶ�������-+4g LSBa = 2*4/65535.0)
#define Gyro_Gain 	0.0609756f		  //���ٶȱ�ɶ� (��ʼ��������������+-2000 LSBg = 2*2000/65535.0)
#define Gyro_Gr	    0.0010641f		  //���ٶȱ�ɻ���(3.1415/180 * LSBg)    

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

/*MPUԭʼ���ݣ���16λ��ADֵ*/
typedef struct
{
	int16_t mx;  //������
	int16_t my;
	int16_t mz;

	int16_t temp;  //�¶�
    
    Int16_xyz Acc;       //���ٶ�ԭʼ����
    Float_xyz Acc_filt;  //���ٶ��˲�������
    
    Int16_xyz Gyro;      //������ԭʼ����
    Float_xyz Gyro_filt; //�������˲�������
	
	int16_t ax_offset;
	int16_t ay_offset;
	int16_t az_offset;

	int16_t gx_offset;
	int16_t gy_offset;
	int16_t gz_offset;
} MPU_Data_t;


extern MPU_Data_t MPU_Data;  //MPU6050ԭʼ����


/*��ʼ��MPU6050*/
void MPU6050_Init(void);
/*���IIC�����ϵ�MPU6050�Ƿ����*/
void MPU6050_Check(void);
/*���MPU6050 �Ƿ��Ѿ�����*/
uint8_t MPU6050_testConnection(void);
/*��ȡMPU6050ԭʼ����*/
void MPU6050_Read(void);
/*��ȡ���ٶȵ�ԭʼ����*/
void MPU6050_AccRead(int16_t *accData);
/*��ȡ�����ǵ�ԭʼ����*/
void MPU6050_GyroRead(int16_t *gyroData);
/*�¶�ֵ��ȡ*/
void MPU6050_TempRead(float *tempdata);
/*�����ʼƫ��*/
void MPU_Offset_cali(void);


#endif
