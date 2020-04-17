#ifndef __IIC_H
#define __IIC_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include "math.h"

/*************************IIC引脚配置*************************************/
#define  IIC_SCL_GPIOX  IIC_SCL_GPIO_Port
#define  IIC_SCL_PINX   IIC_SCL_Pin
#define  IIC_SDA_GPIOX  IIC_SDA_GPIO_Port
#define  IIC_SDA_PINX   IIC_SDA_Pin

/****************************END******************************************/


#define  SCL_H  HAL_GPIO_WritePin(IIC_SCL_GPIOX, IIC_SCL_PINX, GPIO_PIN_SET)
#define  SCL_L  HAL_GPIO_WritePin(IIC_SCL_GPIOX, IIC_SCL_PINX, GPIO_PIN_RESET)
#define  SDA_H  HAL_GPIO_WritePin(IIC_SDA_GPIOX, IIC_SDA_PINX, GPIO_PIN_SET)
#define  SDA_L  HAL_GPIO_WritePin(IIC_SDA_GPIOX, IIC_SDA_PINX, GPIO_PIN_RESET)
#define  SDA_read  HAL_GPIO_ReadPin(IIC_SDA_GPIOX, IIC_SDA_PINX)

#define  OFFSET  (uint8_t)(log10(IIC_SDA_PINX)/log10(2))
#define  SDA_IN()  {IIC_SDA_GPIOX->MODER&=~(3<<(OFFSET*2));IIC_SDA_GPIOX->MODER|=0<<OFFSET*2;}
#define  SDA_OUT()  {IIC_SDA_GPIOX->MODER&=~(3<<(OFFSET*2));IIC_SDA_GPIOX->MODER|=1<<OFFSET*2;}


void IIC_Start(void);			 //发送IIC开始信号
void IIC_Stop(void);	  	  	 //发送IIC停止信号
void IIC_Ack(void);				 //IIC发送ACK信号
void IIC_NAck(void);			 //IIC不发送ACK信号
uint8_t IIC_WaitAck(void); 		 //IIC等待ACK信号

void IIC_SendByte(uint8_t data);  //IIC发送一个字节
uint8_t IIC_ReadByte(uint8_t ack);//IIC读取一个字节

uint8_t IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf);  //读取指定设备指定寄存器的一个值
uint8_t IIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);  //读取指定设备指定寄存器的length个值
uint8_t IIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t buf);  //写入指定设备指定寄存器的一个值
uint8_t IIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);  //将多个字节写入指定设备指定寄存器


#endif
