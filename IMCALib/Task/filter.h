#ifndef __FILTER_H
#define __FILTER_H

#include "mytype.h"
#include "mpu6050.h"


#define N 20      //�˲����������С

//һ�׵�ͨ�˲������������㹫ʽ��a = 2*pi*T*Fc
//���У�T�ǲ������ڣ�Fc�ǽ�ֹƵ��
#define LPF_1st_1000Hz_50Hz_Parameter 0.314159f  

enum{
  X = 0,
  Y = 1,
  Z = 2,    
};

//һ�׵�ͨ�˲����ṹ��
typedef struct
{
    float a;
    float x;
    float y[2];
    
}LPF_Filter_1st_t;

//IIR 2�׵�ͨ�˲����ṹ��
typedef struct
{
    float a[3];
    float b[3];
    float x[3];
    float y[3];
    
}IIR_Filter_2st_t;

//IIR 4�׵�ͨ�˲����ṹ��
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


/*���ٶ��˲�*/
void SortAver_FilterAcc(Int16_xyz *acc, Float_xyz *Acc_filt, uint8_t n);
/*�������˲�*/
void SortAver_FilterGyro(Int16_xyz *gyro, Float_xyz *Gyro_filt, uint8_t n);
void QuiteSort(float* a, int low, int high);
float FindPos(float*a, int low, int high);
float LPF_1st_Filter(LPF_Filter_1st_t *lpf, float input); //һ�׵�ͨ�˲���
float IIR_2st_Filter(IIR_Filter_2st_t *iir, float input); //����IIRֱ��II�Ͱ�����˹��ͨ�˲���
float IIR_4st_Filter(IIR_Filter_4st_t *iir, float input); //�Ľ�IIRֱ��II�Ͱ�����˹��ͨ�˲���
void IIR_Filter_Init(void);


#endif
