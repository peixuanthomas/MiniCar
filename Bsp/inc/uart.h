/**
  ******************************************************************************
  * @file           : uart.h
  * @brief          : 串口UART的业务封装头文件
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
#ifndef _UART_H_
#define _UART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "FIFO.h"

// UART FIFO缓冲区的大小定义
#define USART_REC_LEN 128	//接收缓冲区大小
#define USART_TRS_LEN 128	//发送缓冲区大小

// UART 含FIFO的接口封装结构体
typedef struct {
	UART_HandleTypeDef *huart;		// HAL UART 句柄指针
	fifo_t tx_fifo;					// 发送 FIFO
	fifo_t rx_fifo;					// 接收 FIFO
	uint8_t tx_buf[USART_TRS_LEN];	// 发送缓冲区
	uint8_t rx_buf[USART_REC_LEN];	// 接收缓冲区
	uint8_t tx_byte;					// HAL interrupt transmit byte storage
	volatile bool tx_busy;			// 标志：是否正在发送 DMA/中断传输
} uart_fifo_handle_t;

//提供给外部其他文件调用的UART对象声明
extern uart_fifo_handle_t g_uart1;
extern uart_fifo_handle_t g_uart3;

/** HAL层的UART初始化函数
  * @brief  UART1 HAL层初始化，准备相应的FIFO缓冲区
  * 		UART的接收、发送均保存在FIFO缓冲区，由底层中断函数执行实际接收/发送动作
  * 		开启接收中断
  * @param  NONE
  * @retval NONE
  */
void InitMwUart1(void);	//UART1 HAL初始化

/** HAL层的UART初始化函数
  * @brief  UART3 HAL层初始化，准备相应的FIFO缓冲区
  * 		UART的接收、发送均保存在FIFO缓冲区，由底层中断函数执行实际接收/发送动作
  * 		开启接收中断
  * @param  NONE
  * @retval NONE
  */
void InitMwUart3(void);	//UART3 HAL初始化

/** HAL层的UART发送函数，即提供业务层进行数据发送
  * @brief  UART HAL层一个字节的发送（非阻塞，写入FIFO）
  * @param  handle	具体向哪个UART发送
  * @param  ch		发送的字符
  * @retval 若FIFO已满，则-1；正确发送则返回0
  */
int uart_fifo_put(uart_fifo_handle_t *handle, uint8_t ch);

/** HAL层的UART批量发送函数，即提供业务层进行数据批量发送
  * @brief  UART HAL层多个字节的发送（非阻塞，写入FIFO）
  * @param  handle	具体向哪个UART发送
  * @param  buf		待发送的缓冲区
  * @param  len		待发送的字符数
  * @retval -1 - FIFO已满	0 - 设备没有初始化	其他 - 实际发送字符数
  */
int uart_fifo_puts(uart_fifo_handle_t *handle, const uint8_t *buf, size_t len);

/** HAL层的UART接收函数，即提供业务层进行数据接收
  * @brief  UART HAL层一个字节的接收，从接收缓冲区读取
  * @param  handle	具体从哪个UART接收
  * @param  ch		接收到的具体字符指针
  * @retval -1 - FIFO空		0 - 接收成功
  */
int uart_fifo_get(uart_fifo_handle_t *handle, uint8_t *ch);

/** HAL层的UART批量接收函数，即提供业务层进行数据批量接收
  * @brief  UART HAL层多个字节的接收，从接收缓冲区读取
  * @param  handle	具体从哪个UART接收
  * @param  data	接收到的具体缓冲区指针
  * @param  len		预期接收的字符数
  * @retval 0 - 没有数据		其他 - 接收成功的实际字符数
  */
int uart_fifo_gets(uart_fifo_handle_t *handle, uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif
