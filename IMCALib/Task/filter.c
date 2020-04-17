#include "filter.h"


LPF_Filter_1st_t AccLPF_1st[3];
IIR_Filter_2st_t AccLPF_IIR_2st[3];
IIR_Filter_4st_t AccLPF_IIR_4st[3];

/*******************************************************
 * 巴特沃斯II阶低通滤波器
 * 采样频率：1000Hz
 * 截止频率：25Hz
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

//采样频率:1000Hz
//截止频率:60Hz
//巴特沃斯
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
* 函  数 ：void  SortAver_FilterAcc(Int16_xyz *acc,Float_xyz *Acc_filt,uint8_t n)
* 功  能 ：加速度去最值平均值滑动窗口滤波三组数据
* 参  数 ：*acc 要滤波数据地址
*          *Acc_filt 滤波后数据地址
* 返回值 ：无
* 备  注 : 无
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
	cnt++;      //这个的位置必须在赋值语句后，否则bufx[0]不会被赋值
	if(cnt<n && flag) 
		return;   //数组填不满不计算
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
* 函  数 ：void  SortAver_FilterGyro(Int16_xyz *gyro,Float_xyz *Gyro_filt,uint8_t n)
* 功  能 ：陀螺仪去最值平均值滑动窗口滤波三组数据
* 参  数 ：*gyro 要滤波数据地址
*          *Gyro_filt 滤波后数据地址
* 返回值 ：无
* 备  注 : 无
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
	cnt++;      //这个的位置必须在赋值语句后，否则bufx[0]不会被赋值
	if(cnt<n && flag) 
		return;   //数组填不满不计算
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
* 函  数 ：void QuiteSort(float* a,int low,int high)
* 功  能 ：快速排序
* 参  数 ：a  数组首地址
*          low数组最小下标
*          high数组最大下标
* 返回值 ：无
* 备  注 : 无
*******************************************************************************/
void QuiteSort(float* a, int low, int high)
{
    int pos;
    if(low<high)
    {
     pos = FindPos(a,low,high); //排序一个位置
     QuiteSort(a,low,pos-1);    //递归调用
     QuiteSort(a,pos+1,high);
    }
}


/*******************************************************************************
* 函  数 ：float FindPos(float*a,int low,int high)
* 功  能 ：确定一个元素位序
* 参  数 ：a  数组首地址
*          low数组最小下标
*          high数组最大下标
* 返回值 ：返回元素的位序low
* 备  注 : 无
*******************************************************************************/
float FindPos(float*a, int low, int high)
{
    float val = a[low];                      //选定一个要确定值val确定位置
    while(low<high)
    {
        while(low<high && a[high]>=val)
             high--;                       //如果右边的数大于VAL下标往前移
             a[low] = a[high];             //当右边的值小于VAL则复值给A[low]

        while(low<high && a[low]<=val)
             low++;                        //如果左边的数小于VAL下标往后移
             a[high] = a[low];             //当左边的值大于VAL则复值给右边a[high]
    }
    a[low] = val;
    return low;
}


/*一阶低通滤波器*/
float LPF_1st_Filter(LPF_Filter_1st_t *lpf, float input)
{
    lpf->x = input;
    lpf->y[1] = lpf->a*lpf->x + (1 - lpf->a)*lpf->y[0];
    lpf->y[0] = lpf->y[1];
    
    return lpf->y[1];
}


/*二阶IIR直接II型低通滤波器*/
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

/*四阶IIR直接II型低通滤波器*/
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


/*IIR滤波器初始化*/
void IIR_Filter_Init(void)
{
    memset((void *)&AccLPF_IIR_2st,sizeof(AccLPF_IIR_2st),0);
    memset((void *)&AccLPF_IIR_4st,sizeof(AccLPF_IIR_4st),0);
    
    AccLPF_1st[X].a = LPF_1st_1000Hz_50Hz_Parameter;
    
    uint8_t t;
    for(t=0; t<3; t++)
    {
         AccLPF_IIR_2st[X].a[t] = Butterwoth_IIR_2st_1000Hz_25Hz_Parameter[0][t]; //X轴
         AccLPF_IIR_2st[X].b[t] = Butterwoth_IIR_2st_1000Hz_25Hz_Parameter[1][t];
         AccLPF_IIR_2st[Y].a[t] = Butterwoth_IIR_2st_1000Hz_25Hz_Parameter[0][t]; //Y轴
         AccLPF_IIR_2st[Y].b[t] = Butterwoth_IIR_2st_1000Hz_25Hz_Parameter[1][t];	
         AccLPF_IIR_2st[Z].a[t] = Butterwoth_IIR_2st_1000Hz_25Hz_Parameter[0][t]; //Z轴
         AccLPF_IIR_2st[Z].b[t] = Butterwoth_IIR_2st_1000Hz_25Hz_Parameter[1][t];
    }
    
    for(t=0; t<5; t++)
    {
        AccLPF_IIR_4st[X].a[t] = Butterwoth_IIR_4st_1000Hz_60Hz_Parameter[0][t]; //X轴
        AccLPF_IIR_4st[X].b[t] = Butterwoth_IIR_4st_1000Hz_60Hz_Parameter[1][t];
        AccLPF_IIR_4st[Y].a[t] = Butterwoth_IIR_4st_1000Hz_60Hz_Parameter[0][t]; //Y轴
        AccLPF_IIR_4st[Y].b[t] = Butterwoth_IIR_4st_1000Hz_60Hz_Parameter[1][t];
        AccLPF_IIR_4st[Z].a[t] = Butterwoth_IIR_4st_1000Hz_60Hz_Parameter[0][t]; //Z轴
        AccLPF_IIR_4st[Z].b[t] = Butterwoth_IIR_4st_1000Hz_60Hz_Parameter[1][t];
    }
  
}


