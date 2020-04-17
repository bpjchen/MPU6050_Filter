#ifndef  __IMU_H
#define  __IMU_H


#define sampleFreq	1000.0f	    //采样频率，单位：Hz
#define twoKp	1.6f	//比例增益
#define twoKi	0.1f	//积分增益

typedef struct
{
    float ax;  //加速度
    float ay;
    float az;
    
    float temp;
    
    float wx;  //角速度，rad/s
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
