#ifndef _IIC_H_
#define _IIC_H_

#include "io_bit.h"
#include "delay.h"
#include "main.h"

#define IIC_GPIO_RCC 
#define IIC_GPIO GPIOA
#define IIC_SCL_PIN 
#define IIC_SDA_PIN 

#define IIC_SCL PBout(6)
#define IIC_SDA PBout(7)
#define IIC_SDA_IN PBin(7)

void IIC_Pin_Init(void);

void IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Ack(u8 ack);
u8 IIC_Send_Data(u8 data);
u8 IIC_Read_Data(void);

#endif
