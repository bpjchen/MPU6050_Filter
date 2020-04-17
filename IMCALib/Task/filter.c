#include "filter.h"


LPF_Filter_1st_t AccLPF_1st[3];
IIR_Filter_2st_t AccLPF_IIR_2st[3];
IIR_Filter_4st_t AccLPF_IIR_4st[3];

/*******************************************************
 * ������˹II�׵�ͨ�˲���
 * ����Ƶ�ʣ�1000Hz
 * ��ֹƵ�ʣ�25Hz
*******************************************************/
float Butterwoth_IIR_2st_1000Hz_25Hz_Parameter[2][3] =
{
  {  
    1.000000000000000000
,   -1.7786317778245848       
,   0.80080264666570755 
	},
	{
    0.0055427172102806817 
,   0.011085434420561363    
,   0.0055427172102806817  
	}
};

//����Ƶ��:1000Hz
//��ֹƵ��:60Hz
//������˹
float Butterwoth_IIR_4st_1000Hz_60Hz_Parameter[2][5] =
{
  { 
    1.000000000000000000
,  -3.017555238686489
,   3.5071937247162057
,  -1.847550944118578
,   0.37081421592945485
	},
	{
    0.00080635986503710204
,   0.0032254394601484082
,   0.0048381591902226127
,   0.0032254394601484082
,   0.00080635986503710204
	}
};



/********************************************************************************
* ��  �� ��void  SortAver_FilterAcc(Int16_xyz *acc,Float_xyz *Acc_filt,uint8_t n)
* ��  �� �����ٶ�ȥ��ֵƽ��ֵ���������˲���������
* ��  �� ��*acc Ҫ�˲����ݵ�ַ
*          *Acc_filt �˲������ݵ�ַ
* ����ֵ ����
* ��  ע : ��
********************************************************************************/
void  SortAver_FilterAcc(Int16_xyz *acc, Float_xyz *Acc_filt, uint8_t n)
{
	static float bufx[N],bufy[N],bufz[N];
	static uint8_t cnt =0,flag = 1;
	float temp1=0,temp2=0,temp3=0;
	uint8_t i;
	bufx[cnt] = acc->X;
	bufy[cnt] = acc->Y;
	bufz[cnt] = acc->Z;
	cnt++;      //�����λ�ñ����ڸ�ֵ���󣬷���bufx[0]���ᱻ��ֵ
	if(cnt<n && flag) 
		return;   //�������������
	else
		flag = 0;
	
	QuiteSort(bufx,0,n-1);
	QuiteSort(bufy,0,n-1);
	QuiteSort(bufz,0,n-1);
	for(i=1;i<n-1;i++)
	 {
		temp1 += bufx[i];
		temp2 += bufy[i];
		temp3 += bufz[i];
	 }

	 if(cnt>=n) cnt = 0;
	 Acc_filt->X  = temp1/(n-2);
	 Acc_filt->Y  = temp2/(n-2);
	 Acc_filt->Z  = temp3/(n-2);
}


/********************************************************************************
* ��  �� ��void  SortAver_FilterGyro(Int16_xyz *gyro,Float_xyz *Gyro_filt,uint8_t n)
* ��  �� ��������ȥ��ֵƽ��ֵ���������˲���������
* ��  �� ��*gyro Ҫ�˲����ݵ�ַ
*          *Gyro_filt �˲������ݵ�ַ
* ����ֵ ����
* ��  ע : ��
********************************************************************************/
void SortAver_FilterGyro(Int16_xyz *gyro, Float_xyz *Gyro_filt, uint8_t n)
{
	static float bufx[N],bufy[N],bufz[N];
	static uint8_t cnt =0,flag = 1;
	float temp1=0,temp2=0,temp3=0;
	uint8_t i;
	bufx[cnt] = gyro->X;
	bufy[cnt] = gyro->Y;
	bufz[cnt] = gyro->Z;
	cnt++;      //�����λ�ñ����ڸ�ֵ���󣬷���bufx[0]���ᱻ��ֵ
	if(cnt<n && flag) 
		return;   //�������������
	else
		flag = 0;
	
	QuiteSort(bufx,0,n-1);
	QuiteSort(bufy,0,n-1);
	QuiteSort(bufz,0,n-1);
	for(i=1;i<n-1;i++)
	 {
		temp1 += bufx[i];
		temp2 += bufy[i];
		temp3 += bufz[i];
	 }

	 if(cnt>=n) cnt = 0;
	 Gyro_filt->X  = temp1/(n-2);
	 Gyro_filt->Y  = temp2/(n-2);
	 Gyro_filt->Z  = temp3/(n-2);
}


/*******************************************************************************
* ��  �� ��void QuiteSort(float* a,int low,int high)
* ��  �� ����������
* ��  �� ��a  �����׵�ַ
*          low������С�±�
*          high��������±�
* ����ֵ ����
* ��  ע : ��
*******************************************************************************/
void QuiteSort(float* a, int low, int high)
{
    int pos;
    if(low<high)
    {
     pos = FindPos(a,low,high); //����һ��λ��
     QuiteSort(a,low,pos-1);    //�ݹ����
     QuiteSort(a,pos+1,high);
    }
}


/*******************************************************************************
* ��  �� ��float FindPos(float*a,int low,int high)
* ��  �� ��ȷ��һ��Ԫ��λ��
* ��  �� ��a  �����׵�ַ
*          low������С�±�
*          high��������±�
* ����ֵ ������Ԫ�ص�λ��low
* ��  ע : ��
*******************************************************************************/
float FindPos(float*a, int low, int high)
{
    float val = a[low];                      //ѡ��һ��Ҫȷ��ֵvalȷ��λ��
    while(low<high)
    {
        while(low<high && a[high]>=val)
             high--;                       //����ұߵ�������VAL�±���ǰ��
             a[low] = a[high];             //���ұߵ�ֵС��VAL��ֵ��A[low]

        while(low<high && a[low]<=val)
             low++;                        //�����ߵ���С��VAL�±�������
             a[high] = a[low];             //����ߵ�ֵ����VAL��ֵ���ұ�a[high]
    }
    a[low] = val;
    return low;
}


/*һ�׵�ͨ�˲���*/
float LPF_1st_Filter(LPF_Filter_1st_t *lpf, float input)
{
    lpf->x = input;
    lpf->y[1] = lpf->a*lpf->x + (1 - lpf->a)*lpf->y[0];
    lpf->y[0] = lpf->y[1];
    
    return lpf->y[1];
}


/*����IIRֱ��II�͵�ͨ�˲���*/
float IIR_2st_Filter(IIR_Filter_2st_t *iir, float input)
{
    iir->x[2] = input;
    iir->y[2] = (iir->b[0]*iir->x[2] + 
                 iir->b[1]*iir->x[1] +
                 iir->b[2]*iir->x[0] -
                 iir->a[1]*iir->y[1] -
                 iir->a[2]*iir->y[0] ) / iir->a[0];
    iir->x[0] = iir->x[1];
    iir->x[1] = iir->x[2];
    
    iir->y[0] = iir->y[1];
    iir->y[1] = iir->y[2];
    
    return iir->y[2];
}

/*�Ľ�IIRֱ��II�͵�ͨ�˲���*/
float IIR_4st_Filter(IIR_Filter_4st_t *iir, float input)
{
    iir->x[4] = input;
	iir->y[4] = (iir->b[0]*iir->x[4] +
                 iir->b[1]*iir->x[3] +
                 iir->b[2]*iir->x[2] +
                 iir->b[3]*iir->x[1] +
                 iir->b[4]*iir->x[0] -
                 iir->a[1]*iir->y[3] -
                 iir->a[2]*iir->y[2] -
	             iir->a[3]*iir->y[1] -
	             iir->a[4]*iir->y[0] ) / iir->a[0];
	
	iir->x[0] = iir->x[1];
	iir->x[1] = iir->x[2];
	iir->x[2] = iir->x[3];
	iir->x[3] = iir->x[4];
	
	iir->y[0] = iir->y[1];
	iir->y[1] = iir->y[2];
	iir->y[2] = iir->y[3];
	iir->y[3] = iir->y[4];
    
    return iir->y[4];
}


/*IIR�˲�����ʼ��*/
void IIR_Filter_Init(void)
{
    memset((void *)&AccLPF_IIR_2st,sizeof(AccLPF_IIR_2st),0);
    memset((void *)&AccLPF_IIR_4st,sizeof(AccLPF_IIR_4st),0);
    
    AccLPF_1st[X].a = LPF_1st_1000Hz_50Hz_Parameter;
    
    uint8_t t;
    for(t=0; t<3; t++)
    {
         AccLPF_IIR_2st[X].a[t] = Butterwoth_IIR_2st_1000Hz_25Hz_Parameter[0][t]; //X��
         AccLPF_IIR_2st[X].b[t] = Butterwoth_IIR_2st_1000Hz_25Hz_Parameter[1][t];
         AccLPF_IIR_2st[Y].a[t] = Butterwoth_IIR_2st_1000Hz_25Hz_Parameter[0][t]; //Y��
         AccLPF_IIR_2st[Y].b[t] = Butterwoth_IIR_2st_1000Hz_25Hz_Parameter[1][t];	
         AccLPF_IIR_2st[Z].a[t] = Butterwoth_IIR_2st_1000Hz_25Hz_Parameter[0][t]; //Z��
         AccLPF_IIR_2st[Z].b[t] = Butterwoth_IIR_2st_1000Hz_25Hz_Parameter[1][t];
    }
    
    for(t=0; t<5; t++)
    {
        AccLPF_IIR_4st[X].a[t] = Butterwoth_IIR_4st_1000Hz_60Hz_Parameter[0][t]; //X��
        AccLPF_IIR_4st[X].b[t] = Butterwoth_IIR_4st_1000Hz_60Hz_Parameter[1][t];
        AccLPF_IIR_4st[Y].a[t] = Butterwoth_IIR_4st_1000Hz_60Hz_Parameter[0][t]; //Y��
        AccLPF_IIR_4st[Y].b[t] = Butterwoth_IIR_4st_1000Hz_60Hz_Parameter[1][t];
        AccLPF_IIR_4st[Z].a[t] = Butterwoth_IIR_4st_1000Hz_60Hz_Parameter[0][t]; //Z��
        AccLPF_IIR_4st[Z].b[t] = Butterwoth_IIR_4st_1000Hz_60Hz_Parameter[1][t];
    }
  
}


