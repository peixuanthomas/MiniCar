/**
  ******************************************************************************
  * @file           : i2c_oled.c
  * @brief          : 基于I2C的OLED底层封装
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 上海师范大学 2025-2035
  * All rights reserved.
  *
  * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
  * 上海师范大学 信息与机电工程学院 通信工程专业
  * 开源地址：https://gitee.com/NEagle
  * 修改日期：2025/12/06
  * 版本： V1.0
  * 版权所有，盗版必究
  * V1.0修改说明
  *
  ******************************************************************************
  */

#include "i2c_oled.h"
//#include <string.h>
//#include <math.h>
//#include <stdio.h>
//#include <stdarg.h>

/*
选择OLED驱动方式，默认使用硬件I2C。如果要用软件I2C就将硬件I2C那行的宏定义注释掉，将软件I2C那行的注释取消。
不能同时两个都同时取消注释！
在stm32cubemx中初始化时需要将SCL和SDA引脚的"user lable"分别设置为I2C3_SCL和I2C3_SDA。
*/
//#define OLED_USE_HW_I2C	// 硬件I2C
#define OLED_USE_SW_I2C	// 软件I2C

/*引脚定义，可在此处修改I2C通信引脚*/
#define I2C3_SDA_Pin	GPIO_PIN_9
#define I2C3_SDA_GPIO_Port	GPIOB
#define I2C3_SCL_Pin	GPIO_PIN_8
#define I2C3_SCL_GPIO_Port	GPIOB

/*STM32G474芯片的硬件I2C3: PA8 -- SCL; PC9 -- SDA */

#ifdef OLED_USE_HW_I2C
/*I2C接口，定义OLED屏使用哪个I2C接口*/
#define OLED_I2C            hi2c3
extern  I2C_HandleTypeDef   hi2c3;	//HAL库使用，指定硬件IIC接口
#endif

/*OLED从机地址*/
#define OLED_ADDRESS 0x3C << 1	// 0x3C是OLED的7位地址，左移1位最后位做读写位变成0x78

/*I2C超时时间*/
#define OLED_I2C_TIMEOUT 10
/*软件I2C用的延时时间，下面数值为170MHz主频要延时的值，如果你的主频不一样可以修改一下，100MHz以内的主频改成0就行*/
#define Delay_time 3

/**
 * 数据存储格式：
 * 纵向8点，高位在下，先从左到右，再从上到下
 * 每一个Bit对应一个像素点
 *
 *      B0 B0                  B0 B0
 *      B1 B1                  B1 B1
 *      B2 B2                  B2 B2
 *      B3 B3  ------------->  B3 B3 --
 *      B4 B4                  B4 B4  |
 *      B5 B5                  B5 B5  |
 *      B6 B6                  B6 B6  |
 *      B7 B7                  B7 B7  |
 *                                    |
 *  -----------------------------------
 *  |
 *  |   B0 B0                  B0 B0
 *  |   B1 B1                  B1 B1
 *  |   B2 B2                  B2 B2
 *  --> B3 B3  ------------->  B3 B3
 *      B4 B4                  B4 B4
 *      B5 B5                  B5 B5
 *      B6 B6                  B6 B6
 *      B7 B7                  B7 B7
 *
 * 坐标轴定义：
 * 左上角为(0, 0)点
 * 横向向右为X轴，取值范围：0~127
 * 纵向向下为Y轴，取值范围：0~63
 *
 *       0             X轴           127
 *      .------------------------------->
 *    0 |
 *      |
 *      |
 *      |
 *  Y轴 |
 *      |
 *      |
 *      |
 *   63 |
 *      v
 *
 */


#ifdef OLED_USE_SW_I2C
/**
 * @brief 向 I2C3_SCL_Pin 写高低电平
 * 根据 BitValue 的值，将 I2C3_SCL_Pin 置高电平或低电平。
 * @param BitValue 位值，0 或 1
 */
void OLED_W_SCL(uint8_t BitValue)
{
	/*根据BitValue的值，将SCL置高电平或者低电平*/
	HAL_GPIO_WritePin(I2C3_SCL_GPIO_Port, I2C3_SCL_Pin, (GPIO_PinState)BitValue);
	/*如果单片机速度过快，可在此添加适量延时，以避免超出I2C通信的最大速度*/
    for (volatile uint16_t i = 0; i < Delay_time; i++){
        //for (uint16_t j = 0; j < 10; j++);
    }
}

/**
  * @brief OLED写SDA高低电平
  * @param 要写入SDA的电平值，范围：0/1
  * @return 无
  * @note 当上层函数需要写SDA时，此函数会被调用
  *           用户需要根据参数传入的值，将SDA置为高电平或者低电平
  *           当参数传入0时，置SDA为低电平，当参数传入1时，置SDA为高电平
  */
void OLED_W_SDA(uint8_t BitValue)
{
	/*根据BitValue的值，将SDA置高电平或者低电平*/
	HAL_GPIO_WritePin(I2C3_SDA_GPIO_Port, I2C3_SDA_Pin, (GPIO_PinState)BitValue);
	/*如果单片机速度过快，可在此添加适量延时，以避免超出I2C通信的最大速度*/
	for (volatile uint16_t i = 0; i < Delay_time; i++){
        //for (uint16_t j = 0; j < 10; j++);
    }
}
#endif

/**
 * @brief OLED引脚初始化
 * @param  无
 * @retval 无
 * @note 当上层函数需要初始化时，此函数会被调用,
 *       用户需要将SCL和SDA引脚初始化为开漏模式，并释放引脚
 */
void OLED_GPIO_Init(void)
{
    uint32_t i, j;

    /*在初始化前，加入适量延时，待OLED供电稳定*/
    for (i = 0; i < 1000; i++) {
        for (j = 0; j < 1000; j++)
            ;
    }
#ifdef OLED_USE_SW_I2C
	OLED_W_SCL(1);
	OLED_W_SDA(1);
#endif
}

/*通信协议*********************/

/**
 * @brief I2C起始
 * @param  无
 * @return 无
 */
void OLED_I2C_Start(void)
{
#ifdef OLED_USE_SW_I2C
	OLED_W_SDA(1);		//释放SDA，确保SDA为高电平
	OLED_W_SCL(1);		//释放SCL，确保SCL为高电平
	OLED_W_SDA(0);		//在SCL高电平期间，拉低SDA，产生起始信号
	OLED_W_SCL(0);		//起始后把SCL也拉低，即为了占用总线，也为了方便总线时序的拼接
#endif
}

/**
 * @brief I2C终止
 * @param  无
 * @return 无
 */
void OLED_I2C_Stop(void)
{
#ifdef OLED_USE_SW_I2C
	OLED_W_SDA(0);		//拉低SDA，确保SDA为低电平
	OLED_W_SCL(1);		//释放SCL，使SCL呈现高电平
	OLED_W_SDA(1);		//在SCL高电平期间，释放SDA，产生终止信号
#endif
}

/**
 * @brief I2C发送一个字节
 * @param Byte 要发送的一个字节数据，范围：0x00~0xFF
 * @return 无
 */
void OLED_I2C_SendByte(uint8_t Byte)
{
#ifdef OLED_USE_SW_I2C
	uint8_t i;
	/*循环8次，主机依次发送数据的每一位*/
	for (i = 0; i < 8; i++)
	{
		/*使用掩码的方式取出Byte的指定一位数据并写入到SDA线*/
		/*两个!的作用是，让所有非零的值变为1*/
		OLED_W_SDA(!!(Byte & (0x80 >> i)));
		OLED_W_SCL(1);	//释放SCL，从机在SCL高电平期间读取SDA
		OLED_W_SCL(0);	//拉低SCL，主机开始发送下一位数据
	}
	OLED_W_SCL(1);		//额外的一个时钟，不处理应答信号
	OLED_W_SCL(0);
#endif
}

/**
 * @brief OLED写命令
 * @param Command 要写入的命令值，范围：0x00~0xFF
 * @return 无
 */
void OLED_WriteCommand(uint8_t Command)
{
#ifdef OLED_USE_SW_I2C
    OLED_I2C_Start();           // I2C起始
	OLED_I2C_SendByte(0x78);		//发送OLED的I2C从机地址
	OLED_I2C_SendByte(0x00);	//控制字节，给0x00，表示即将写命令
    OLED_I2C_SendByte(Command); // 写入指定的命令
    OLED_I2C_Stop();            // I2C终止
#elif defined(OLED_USE_HW_I2C)
    uint8_t TxData[2] = {0x00, Command};
    HAL_I2C_Master_Transmit(&OLED_I2C, OLED_ADDRESS, (uint8_t*)TxData, 2, OLED_I2C_TIMEOUT);
#endif 
}

/**
 * @brief OLED写数据
 * @param Data 要写入数据的起始地址
 * @param Count 要写入数据的数量
 * @return 无
 */
void OLED_WriteData(uint8_t *Data, uint8_t Count)
{
    uint8_t i;
#ifdef OLED_USE_SW_I2C
    OLED_I2C_Start();        // I2C起始
	OLED_I2C_SendByte(0x78);		//发送OLED的I2C从机地址

    OLED_I2C_SendByte(0x40); // 控制字节，给0x40，表示即将写数据
    /*循环Count次，进行连续的数据写入*/
    for (i = 0; i < Count; i++) {
        OLED_I2C_SendByte(Data[i]); // 依次发送Data的每一个数据
    }
    OLED_I2C_Stop(); // I2C终止
#elif defined(OLED_USE_HW_I2C)
    uint8_t TxData[Count + 1]; // 分配一个新的数组，大小是Count + 1
    TxData[0] = 0x40; // 起始字节
    // 将Data指向的数据复制到TxData数组的剩余部分
    for (i = 0; i < Count; i++) {
        TxData[i + 1] = Data[i];
    }
    HAL_I2C_Master_Transmit(&OLED_I2C, OLED_ADDRESS, (uint8_t*)TxData, Count + 1, OLED_I2C_TIMEOUT);
#endif    
}

/*********************通信协议*/

