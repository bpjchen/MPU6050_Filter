#include "SW_Wave.h"
#include "usart.h"
#include "imu.h"
#include "filter.h"
	

/*���·��͵�ɽ��๦�ܵ������ֵ�����*/
void SwDataWaveUpdate(void)
{
   
//    SwDataSendFloat(IMU_Data.ax, IMU_Data.ay, IMU_Data.az, 0); //ŷ����
    SwDataSendInt16(MPU_Data.Acc.X, MPU_Data.Acc.Y, MPU_Data.Acc_filt.X, MPU_Data.Acc_filt.Y); //���ٶ�
//    SwDataSendInt16(MPU_Data.Gyro.X, MPU_Data.Gyro.Y, MPU_Data.Gyro_filt.X, MPU_Data.Gyro_filt.Y); //���ٶ�
    
//    SwDataSendFloat(MPU_Filter.Acc_filt.X, MPU_Filter.Acc_filt.Y, MPU_Filter.Acc_filt.Z, 0);
    
}


void SwSendWare(uint8_t *wareaddr, int16_t waresize)
{
    #define CMD_WARE     3
    uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};
    uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};
    UsartSendData(cmdf, sizeof(cmdf));
    UsartSendData(wareaddr, waresize);
    UsartSendData(cmdr, sizeof(cmdr));
}


void UsartSendData(uint8_t *tx_buf, uint8_t len)
{
    uint8_t i;
	
	for(i=0;i<len;i++)
	{
        HAL_UART_Transmit(&huart1,tx_buf+i,1,10);
        
    }
    
}


/*������λfloat�����ݵ�ɽ��๦�ܵ�������*/
void SwDataSendFloat(float data1, float data2, float data3, float data4)
{
    float SendBag[4];

    SendBag[0] = data1;
    SendBag[1] = data2;
    SendBag[2] = data3;
    SendBag[3] = data4; 
    
    SwSendWare((uint8_t *)(SendBag),sizeof(SendBag));
   
}

/*���Ͱ�λfloat�����ݵ�ɽ��๦�ܵ�������*/
void SwDataSendFloatPro(float data1, float data2, float data3, float data4, float data5, float data6, float data7, float data8)
{
    float SendBag[8];

    SendBag[0] = data1;
    SendBag[1] = data2;
    SendBag[2] = data3;
    SendBag[3] = data4; 
    SendBag[4] = data5; 
    SendBag[5] = data6; 
    SendBag[6] = data7; 
    SendBag[7] = data8; 
    SwSendWare((uint8_t *)(SendBag),sizeof(SendBag));
    
}

/*������λint16_t�����ݵ�ɽ��๦�ܵ�������*/
void SwDataSendInt16(int16_t data1, int16_t data2, int16_t data3, int16_t data4)
{
    int16_t SendBag[4];
    
    SendBag[0] = data1;
    SendBag[1] = data2;
    SendBag[2] = data3;
    SendBag[3] = data4; 
    
    SwSendWare((uint8_t *)(SendBag),sizeof(SendBag));
    
}

/*������λuint16_t�����ݵ�ɽ��๦�ܵ�������*/
void SwDataSendUint16(uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4)
{
    uint16_t SendBag[4];
    
    SendBag[0] = data1;
    SendBag[1] = data2;
    SendBag[2] = data3;
    SendBag[3] = data4; 
    
    SwSendWare((uint8_t *)(SendBag),sizeof(SendBag));
    
}

/*������λint32_t�����ݵ�ɽ��๦�ܵ�������*/
void SwDataSendInt32(int32_t data1, int32_t data2, int32_t data3, int32_t data4)
{
    int32_t SendBag[4];
    
    SendBag[0] = data1;
    SendBag[1] = data2;
    SendBag[2] = data3;
    SendBag[3] = data4; 
    
    SwSendWare((uint8_t *)(SendBag),sizeof(SendBag));
    
}


/*������λuint32_t�����ݵ�ɽ��๦�ܵ�������*/
void SwDataSendUint32(uint32_t data1, uint32_t data2, uint32_t data3, uint32_t data4)
{
    uint32_t SendBag[4];
    
    SendBag[0] = data1;
    SendBag[1] = data2;
    SendBag[2] = data3;
    SendBag[3] = data4; 
    
    SwSendWare((uint8_t *)(SendBag),sizeof(SendBag));
    
}

/*������λuint8_t�����ݵ�ɽ��๦�ܵ�������*/
void SwDataSendUint8(uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4)
{
    uint8_t SendBag[4];
    
    SendBag[0] = data1;
    SendBag[1] = data2;
    SendBag[2] = data3;
    SendBag[3] = data4; 
    
    SwSendWare((uint8_t *)(SendBag),sizeof(SendBag));
    
}

/*������λint8_t�����ݵ�ɽ��๦�ܵ�������*/
void SwDataSendInt8(int8_t data1, int8_t data2, int8_t data3, int8_t data4)
{
    int8_t SendBag[4];
    
    SendBag[0] = data1;
    SendBag[1] = data2;
    SendBag[2] = data3;
    SendBag[3] = data4; 
    
    SwSendWare((uint8_t *)(SendBag),sizeof(SendBag));
    
}
