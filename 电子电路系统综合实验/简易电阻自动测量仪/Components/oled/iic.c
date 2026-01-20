#include "iic.h"


void delay_us(u32 us)
{
	
}
/*******************************************************************************
*函数的原型：void IIC_Pin_Init(void)
*函数的功能：IIC引脚初始化
*函数的参数：None
*函数返回值：None
*函数的说明：
	SCL = PA6
	SDA = PA7
*函数编写者：庆
*函数编写日期：2021/2/22
*函数的版本号：V1.0
********************************************************************************/
void IIC_Pin_Init(void)
{
	// GPIO_InitTypeDef GPIO_InitStruct;
	
	// RCC_APB2PeriphClockCmd(IIC_GPIO_RCC,ENABLE);
	
	// //SCL
	// GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	// GPIO_InitStruct.GPIO_Pin = IIC_SCL_PIN;
	// GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	// GPIO_Init(IIC_GPIO,&GPIO_InitStruct);
	
	// //SDA
	// GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
	// GPIO_InitStruct.GPIO_Pin = IIC_SDA_PIN;
	// GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	// GPIO_Init(IIC_GPIO,&GPIO_InitStruct);
}

/*******************************************************************************
*函数的原型：void IIC_Start(void)
*函数的功能：iic起始条件
*函数的参数：None
*函数返回值：None
*函数的说明：
*函数编写者：庆
*函数编写日期：2021/2/22
*函数的版本号：V1.0
********************************************************************************/
void IIC_Start(void)
{
	IIC_SCL = 1;
	IIC_SDA = 1;
	delay_us(5);
	IIC_SDA = 0;
	delay_us(5);
	IIC_SCL = 0;
}

/*******************************************************************************
*函数的原型：void IIC_Stop(void)
*函数的功能：iic停止条件
*函数的参数：None
*函数返回值：None
*函数的说明：
*函数编写者：庆
*函数编写日期：2021/2/22
*函数的版本号：V1.0
********************************************************************************/
void IIC_Stop(void)
{
	IIC_SCL = 0;
//	IIC_SDA = 0;
	
	IIC_SDA = 1;
	delay_us(5);
	IIC_SCL = 1;
	delay_us(5);
	IIC_SDA = 0;
}

/*******************************************************************************
*函数的原型：void IIC_Send_Ack(u8 ack)
*函数的功能：发送应答信号
*函数的参数：
	@ u8 ack：应答信号 0：非应答 1：应答
*函数返回值：None
*函数的说明：
*函数编写者：庆
*函数编写日期：2021/2/22
*函数的版本号：V1.0
********************************************************************************/  
void IIC_Send_Ack(u8 ack)
{
	IIC_SCL = 1;
	//为什么应答信号与非应答信号可以在一个函数中？？
	//它们时钟信号相同，数据信号的初始电平不同？？  分开写
	
	delay_us(5);
	IIC_SCL = 0;
	delay_us(5);
	
	if(ack)
	{
		IIC_SDA = 1;
	}
	else
	{
		IIC_SDA = 0;
	}
	
	delay_us(5);
	//保持周期完整性
	IIC_SCL = 1;
	delay_us(5);
	IIC_SCL = 0;
}

/*******************************************************************************
*函数的原型：u8 IIC_Reception_Ack(void)
*函数的功能：接收应答信号
*函数的参数：None
*函数返回值：
	0：非应答
	1：应答
*函数的说明：高电平数据采集 低电平数据发送
*函数编写者：庆
*函数编写日期：2021/2/22
*函数的版本号：V1.0
********************************************************************************/
u8 IIC_Reception_Ack(void)
{
	IIC_SDA = 1;
	delay_us(5);
	//----
	IIC_SCL = 0;
	delay_us(5);
	IIC_SCL = 1;
	
	if(IIC_SDA_IN)
	{
		return 1;
	}
	
	return 0;
}

//每一个字节后都要跟一个响应位（确认数据是否发送成功）
//发送一字节    
/*******************************************************************************
*函数的原型：u8 IIC_Send_Data(u8 data)
*函数的功能：发送数据
*函数的参数：
	@ u8 data：一字节数据
*函数返回值：应答信号
*函数的说明：8位数据发送 每个字节后面都要跟一个响应
*函数编写者：庆
*函数编写日期：2021/2/22
*函数的版本号：V1.0
********************************************************************************/
u8 IIC_Send_Data(u8 data)
{
	u8 i = 0;
	
//	IIC_Start();
	
	for(i = 0; i < 8; i++)
	{
		//一位一位的发
//		IIC_SCL = 1;
//		delay_us(5);
		IIC_SCL = 0; 
		if(data & 0x80)
		{
			IIC_SDA = 1;
		}
		else
		{
			IIC_SDA = 0;
		}
		data <<= 1;
		delay_us(5);
	//保持周期完整性
		IIC_SCL = 1;
		delay_us(5);
//		IIC_SCL = 0;
	}
	
	IIC_SCL = 0;
	return IIC_Reception_Ack();
	//接收应答
}

u8 IIC_Send_Data_Inv(u8 data)
{
	u8 i = 0;
	
//	IIC_Start();
	
	for(i = 0; i < 8; i++)
	{
		//一位一位的发
//		IIC_SCL = 1;
//		delay_us(5);
		IIC_SCL = 0; 
		if(data & 0x80)
		{
			IIC_SDA = 0;
		}
		else
		{
			IIC_SDA = 1;
		}
		data <<= 1;
		delay_us(5);
	//保持周期完整性
		IIC_SCL = 1;
		delay_us(5);
//		IIC_SCL = 0;
	}
	
	IIC_SCL = 0;
	return IIC_Reception_Ack();
	//接收应答
}

/*******************************************************************************
*函数的原型：u8 IIC_Read_Data(void)
*函数的功能：读一字节数据
*函数的参数：None
*函数返回值：
	一字节数据
*函数的说明：
*函数编写者：庆
*函数编写日期：2021/2/22
*函数的版本号：V1.0
********************************************************************************/
u8 IIC_Read_Data(void)
{
	u8 data = 0;
	u8 i = 0;
	
	IIC_SDA = 1;
	
	for(i = 0; i < 8; i++)
	{
		IIC_SCL = 0;
		delay_us(5);
		IIC_SCL = 1;
		
		if(IIC_SDA_IN)
		{
			data |= 1;
		}
		
		data <<= 1;
	}
	
	IIC_Send_Ack(1);
	
	return data;
}
