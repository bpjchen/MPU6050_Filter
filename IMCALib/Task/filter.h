#ifndef __FILTER_H
#define __FILTER_H

#include "mytype.h"
#include "mpu6050.h"


#define N 20      //滤波缓存数组大小

//一阶低通滤波器参数，计算公式：a = 2*pi*T*Fc
//其中：T是采样周期，Fc是截止频率
#define LPF_1st_1000Hz_50Hz_Parameter 0.314159f  

enum{
  X = 0,
  Y = 1,
  Z = 2,    
};

//一阶低通滤波器结构体
typedef struct
{
    float a;
    float x;
    float y[2];
    
}LPF_Filter_1st_t;

//IIR 2阶低通滤波器结构体
typedef struct
{
    float a[3];
    float b[3];
    float x[3];
    float y[3];
    
}IIR_Filter_2st_t;

//IIR 4阶低通滤波器结构体
typedef struct
{
   float a[5];
   float b[5];
   float x[5];
   float y[5];
    
}IIR_Filter_4st_t;


extern LPF_Filter_1st_t AccLPF_1st[3];
extern IIR_Filter_2st_t AccLPF_IIR_2st[3];
extern IIR_Filter_4st_t AccLPF_IIR_4st[3];


/*加速度滤波*/
void SortAver_FilterAcc(Int16_xyz *acc, Float_xyz *Acc_filt, uint8_t n);
/*陀螺仪滤波*/
void SortAver_FilterGyro(Int16_xyz *gyro, Float_xyz *Gyro_filt, uint8_t n);
void QuiteSort(float* a, int low, int high);
float FindPos(float*a, int low, int high);
float LPF_1st_Filter(LPF_Filter_1st_t *lpf, float input); //一阶低通滤波器
float IIR_2st_Filter(IIR_Filter_2st_t *iir, float input); //二阶IIR直接II型巴特沃斯低通滤波器
float IIR_4st_Filter(IIR_Filter_4st_t *iir, float input); //四阶IIR直接II型巴特沃斯低通滤波器
void IIR_Filter_Init(void);


#endif
