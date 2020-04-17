#include "imu.h"
#include "math.h"


IMU_Data_t IMU_Data;


/*���ټ��� 1/Sqrt(x)*/
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; //��Ԫ����ʼ��
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	//�������ʼ��

void MahonyAHRSupdateIMU(IMU_Data_t *imu_data) 
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
    float ax, ay, az, gx, gy, gz;
    
    ax = imu_data->ax;
    ay = imu_data->ay;
    az = imu_data->az;
    
    gx = imu_data->wx;
    gy = imu_data->wy;
    gz = imu_data->wz;

	/*ֻ���ڼ��ٶȼƲ�����Чʱ�ż���*/
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
    {

		/*���ټƲ���ֵ��һ��*/
		recipNorm = invSqrt(ax*ax + ay*ay + az*az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		/*��������ϵ�µ�������������*/
		halfvx = q1*q3 - q0*q2;
		halfvy = q0*q1 + q2*q3;
		halfvz = 0.5f - q1*q1 - q2*q2;
	
		/*��������˼�����ټƶ�ȡ�ķ������������ټƷ���Ĳ�ֵ*/
		halfex = (ay*halfvz - az*halfvy);
		halfey = (az*halfvx - ax*halfvz);
		halfez = (ax*halfvy - ay*halfvx);

		/*����ۼ�*/
		if(twoKi > 0.0f) 
        {
			integralFBx += twoKi*halfex*(1.0f / sampleFreq);
			integralFBy += twoKi*halfey*(1.0f / sampleFreq);
			integralFBz += twoKi*halfez*(1.0f / sampleFreq);
			gx += integralFBx;
			gy += integralFBy;
			gz += integralFBz;
		}
		else 
        {
			integralFBx = 0.0f;
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		/*����������������PI�������������*/
		gx += twoKp*halfex;
		gy += twoKp*halfey;
		gz += twoKp*halfez;
	}
	
	/*������ֱ仯��*/
	gx *= (0.5f*(1.0f / sampleFreq));
	gy *= (0.5f*(1.0f / sampleFreq));
	gz *= (0.5f*(1.0f / sampleFreq));

    /*����һ����������������Ԫ��΢�ַ���*/
	q0 += (-q1*gx - q2*gy - q3*gz);
	q1 += (+q0*gx + q2*gz - q3*gy);
	q2 += (+q0*gy - q1*gz + q3*gx);
	q3 += (+q0*gz + q1*gy - q2*gx); 
	
	/*��Ԫ����һ��*/
	recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
    
    /*��Ԫ��ת��Ϊŷ����*/
    IMU_Data.rol = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2))*57.3f;
    IMU_Data.pit = asinf(2.0f*(q0*q2 - q1*q3))*57.3f;
    IMU_Data.yaw = atan2f(2.0f*(q1*q2 + q0*q3), 1.0f - 2.0f*(q2*q2 + q3*q3))*57.3f;
    
}


