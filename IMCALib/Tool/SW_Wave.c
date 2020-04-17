#include "SW_Wave.h"
#include "usart.h"
#include "imu.h"
#include "filter.h"
	

/*更新发送到山外多功能调试助手的数据*/
void SwDataWaveUpdate(void)
{
   
//    SwDataSendFloat(IMU_Data.ax, IMU_Data.ay, IMU_Data.az, 0); //欧拉角
    SwDataSendInt16(MPU_Data.Acc.X, MPU_Data.Acc.Y, MPU_Data.Acc_filt.X, MPU_Data.Acc_filt.Y); //加速度
//    SwDataSendInt16(MPU_Data.Gyro.X, MPU_Data.Gyro.Y, MPU_Data.Gyro_filt.X, MPU_Data.Gyro_filt.Y); //角速度
    
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


/*发送四位float型数据到山外多功能调试助手*/
void SwDataSendFloat(float data1, float data2, float data3, float data4)
{
    float SendBag[4];

    SendBag[0] = data1;
    SendBag[1] = data2;
    SendBag[2] = data3;
    SendBag[3] = data4; 
    
    SwSendWare((uint8_t *)(SendBag),sizeof(SendBag));
   
}

/*发送八位float型数据到山外多功能调试助手*/
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

/*发送四位int16_t型数据到山外多功能调试助手*/
void SwDataSendInt16(int16_t data1, int16_t data2, int16_t data3, int16_t data4)
{
    int16_t SendBag[4];
    
    SendBag[0] = data1;
    SendBag[1] = data2;
    SendBag[2] = data3;
    SendBag[3] = data4; 
    
    SwSendWare((uint8_t *)(SendBag),sizeof(SendBag));
    
}

/*发送四位uint16_t型数据到山外多功能调试助手*/
void SwDataSendUint16(uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4)
{
    uint16_t SendBag[4];
    
    SendBag[0] = data1;
    SendBag[1] = data2;
    SendBag[2] = data3;
    SendBag[3] = data4; 
    
    SwSendWare((uint8_t *)(SendBag),sizeof(SendBag));
    
}

/*发送四位int32_t型数据到山外多功能调试助手*/
void SwDataSendInt32(int32_t data1, int32_t data2, int32_t data3, int32_t data4)
{
    int32_t SendBag[4];
    
    SendBag[0] = data1;
    SendBag[1] = data2;
    SendBag[2] = data3;
    SendBag[3] = data4; 
    
    SwSendWare((uint8_t *)(SendBag),sizeof(SendBag));
    
}


/*发送四位uint32_t型数据到山外多功能调试助手*/
void SwDataSendUint32(uint32_t data1, uint32_t data2, uint32_t data3, uint32_t data4)
{
    uint32_t SendBag[4];
    
    SendBag[0] = data1;
    SendBag[1] = data2;
    SendBag[2] = data3;
    SendBag[3] = data4; 
    
    SwSendWare((uint8_t *)(SendBag),sizeof(SendBag));
    
}

/*发送四位uint8_t型数据到山外多功能调试助手*/
void SwDataSendUint8(uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4)
{
    uint8_t SendBag[4];
    
    SendBag[0] = data1;
    SendBag[1] = data2;
    SendBag[2] = data3;
    SendBag[3] = data4; 
    
    SwSendWare((uint8_t *)(SendBag),sizeof(SendBag));
    
}

/*发送四位int8_t型数据到山外多功能调试助手*/
void SwDataSendInt8(int8_t data1, int8_t data2, int8_t data3, int8_t data4)
{
    int8_t SendBag[4];
    
    SendBag[0] = data1;
    SendBag[1] = data2;
    SendBag[2] = data3;
    SendBag[3] = data4; 
    
    SwSendWare((uint8_t *)(SendBag),sizeof(SendBag));
    
}
