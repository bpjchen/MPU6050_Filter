#ifndef  __IMU_H
#define  __IMU_H


#define sampleFreq	1000.0f	    //����Ƶ�ʣ���λ��Hz
#define twoKp	1.6f	//��������
#define twoKi	0.1f	//��������

typedef struct
{
    float ax;  //���ٶ�
    float ay;
    float az;
    
    float temp;
    
    float wx;  //���ٶȣ�rad/s
    float wy;
    float wz;
    
    float rol;
    float pit;
    float yaw;
      
    
    
}IMU_Data_t;


extern IMU_Data_t IMU_Data;


float invSqrt(float x);
void MahonyAHRSupdateIMU(IMU_Data_t *imu_data);


#endif
