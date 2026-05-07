/**
  ******************************************************************************
  * @file           : uart.c
  * @brief          : 串口UART的业务封装文件
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
#include "uart.h"

//CubeMX创建的两个UART句柄
extern UART_HandleTypeDef huart1;	//UART1的硬件设备句柄
extern UART_HandleTypeDef huart3;	//UART3的硬件设备句柄

//中断服务函数接收缓冲区
uint8_t Rx1Byte;	//UART1中断缓冲区
uint8_t Rx3Byte;	//UART3中断缓冲区

//含FIFO的UART1结构体
uart_fifo_handle_t g_uart1 = { 
	.huart = &huart1,
	.tx_busy = false
};

//含FIFO的UART3结构体
uart_fifo_handle_t g_uart3 = { 
	.huart = &huart3,
	.tx_busy = false
};

//为支持stdio.h头文件支持的标准输出函数，含printf函数的底层支持
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 

/** 重定义fputc函数
  * @brief  重定向标准输出的实际执行函数
  * 		不管输出给哪个文件，均执行相同动作
  * @param  ch	输出内容
  * @param  f	输出设备文件句柄
  * @retval 输出实际内容
  */
int fputc(int ch, FILE *f)
{
	uart_fifo_put(&g_uart1, (uint8_t)ch);
	uart_fifo_put(&g_uart3, (uint8_t)ch);
	return ch;
}

/** HAL层的UART初始化函数
  * @brief  UART1 HAL层初始化，准备相应的FIFO缓冲区
  * 		UART的接收、发送均保存在FIFO缓冲区，由底层中断函数执行实际接收/发送动作
  * 		开启接收中断
  * @param  NONE
  * @retval NONE
  */
void InitMwUart1(void)
{
	fifo_init(&g_uart1.rx_fifo, g_uart1.rx_buf, USART_REC_LEN);
	fifo_init(&g_uart1.tx_fifo, g_uart1.tx_buf, USART_TRS_LEN);
	HAL_UART_Receive_IT(g_uart1.huart, &Rx1Byte, 1);
}

/** HAL层的UART初始化函数
  * @brief  UART3 HAL层初始化，准备相应的FIFO缓冲区
  * 		UART的接收、发送均保存在FIFO缓冲区，由底层中断函数执行实际接收/发送动作
  * 		开启接收中断
  * @param  NONE
  * @retval NONE
  */
void InitMwUart3(void)
{
	fifo_init(&g_uart3.rx_fifo, g_uart3.rx_buf, USART_REC_LEN);
	fifo_init(&g_uart3.tx_fifo, g_uart3.tx_buf, USART_TRS_LEN);
	HAL_UART_Receive_IT(g_uart3.huart, &Rx3Byte, 1);
}

/** HAL层的UART发送函数，即提供业务层进行数据发送
  * @brief  UART HAL层一个字节的发送（非阻塞，写入FIFO）
  * @param  handle	具体向哪个UART发送
  * @param  ch		发送的字符
  * @retval 若FIFO已满，则-1；正确发送则返回0
  */
int uart_fifo_put(uart_fifo_handle_t *handle, uint8_t ch) {
    if (fifo_is_full(&handle->tx_fifo)) {
        return -1; // FIFO 满
    }
    fifo_write(&handle->tx_fifo, ch);

    // 如果当前没有发送正在进行，则启动发送
    if (!handle->tx_busy) {
        handle->tx_busy = true;
        if (fifo_read(&handle->tx_fifo, &handle->tx_byte) == 0) {
            HAL_UART_Transmit_IT(handle->huart, &handle->tx_byte, 1);
        } else {
            handle->tx_busy = false; // 不应发生
        }
    }
    return 0;
}

/** HAL层的UART批量发送函数，即提供业务层进行数据批量发送
  * @brief  UART HAL层多个字节的发送（非阻塞，写入FIFO）
  * @param  handle	具体向哪个UART发送
  * @param  buf		待发送的缓冲区
  * @param  len		待发送的字符数
  * @retval -1 - FIFO已满	0 - 设备没有初始化	其他 - 实际发送字符数
  */
int uart_fifo_puts(uart_fifo_handle_t *handle, const uint8_t *buf, size_t len) {
	size_t wr_len;
    if (fifo_is_full(&handle->tx_fifo)) {
        return -1; // FIFO 满
    }
	wr_len = fifo_write_multi(&handle->tx_fifo, buf, len);
	if(wr_len <= 0)
		return 0;	//FIFO没有初始化
	
    // 如果当前没有发送正在进行，则启动发送
    if (!handle->tx_busy) {
        handle->tx_busy = true;
        if (fifo_read(&handle->tx_fifo, &handle->tx_byte) == 0) {
            HAL_UART_Transmit_IT(handle->huart, &handle->tx_byte, 1);
        } else {
            handle->tx_busy = false; // 不应发生
        }
    }
    return wr_len;
}

/** HAL层的UART接收函数，即提供业务层进行数据接收
  * @brief  UART HAL层一个字节的接收，从接收缓冲区读取
  * @param  handle	具体从哪个UART接收
  * @param  ch		接收到的具体字符指针
  * @retval -1 - FIFO空		0 - 接收成功
  */
int uart_fifo_get(uart_fifo_handle_t *handle, uint8_t *ch) {
    return fifo_read(&handle->rx_fifo, ch); // 返回 0 成功，-1 表示空
}

/** HAL层的UART批量接收函数，即提供业务层进行数据批量接收
  * @brief  UART HAL层多个字节的接收，从接收缓冲区读取
  * @param  handle	具体从哪个UART接收
  * @param  data	接收到的具体缓冲区指针
  * @param  len		预期接收的字符数
  * @retval 0 - 没有数据		其他 - 接收成功的实际字符数
  */
int uart_fifo_gets(uart_fifo_handle_t *handle, uint8_t *data, size_t len) {
    return fifo_read_multi(&handle->rx_fifo, data, len); // 返回 0 成功，-1 表示空
}

/** UART发送完成的回调函数
  * @brief  UART发送完成中断的回调函数
  *			当发生发送完成中断，则继续取FIFO数据，并执行下一次发送动作
  *			当取数据失败，则关闭发送标志位
  * @param  huart	实际UART对象
  * @retval NONE
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == g_uart1.huart) {
        if (fifo_read(&g_uart1.tx_fifo, &g_uart1.tx_byte) == 0) {
            // FIFO 中还有数据，继续发送
            HAL_UART_Transmit_IT(huart, &g_uart1.tx_byte, 1);
        } else {
            // FIFO 已空，标记发送空闲
            g_uart1.tx_busy = false;
        }
    }
    if (huart == g_uart3.huart) {
        if (fifo_read(&g_uart3.tx_fifo, &g_uart3.tx_byte) == 0) {
            // FIFO 中还有数据，继续发送
            HAL_UART_Transmit_IT(huart, &g_uart3.tx_byte, 1);
        } else {
            // FIFO 已空，标记发送空闲
            g_uart3.tx_busy = false;
        }
    }
}

/** UART接收完成的回调函数
  * @brief  UART接收完成中断的回调函数
  *			当发生接收完成中断，则则将收到的字符填写到FIFO，并发起下一次接收动作
  * @param  huart	实际UART对象
  * @retval NONE
  */
// HAL 回调：接收到一个字节
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == g_uart1.huart) {
        // 将刚收到的字节存入接收 FIFO
        uint8_t rx_byte = Rx1Byte; // HAL 内部 buffer
        fifo_write(&g_uart1.rx_fifo, rx_byte);          // 忽略满的情况（可加丢弃策略）

        // 重新启动下一次单字节接收
        HAL_UART_Receive_IT(huart, &Rx1Byte, 1);
    }
    if (huart == g_uart3.huart) {
        // 将刚收到的字节存入接收 FIFO
        uint8_t rx_byte = Rx3Byte; // HAL 内部 buffer
        fifo_write(&g_uart3.rx_fifo, rx_byte);          // 忽略满的情况（可加丢弃策略）

        // 重新启动下一次单字节接收
        HAL_UART_Receive_IT(huart, &Rx3Byte, 1);
    }
}

