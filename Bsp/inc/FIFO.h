/**
  ******************************************************************************
  * @file           : FIFO.h
  * @brief          : FIFO先进先出缓冲区头文件
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
#ifndef _FIFO_H_
#define _FIFO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>    // 包含标准整数类型定义
#include <stddef.h>    // 包含size_t等类型定义
#include <stdbool.h>   // 包含布尔类型定义

/**
 * 通用FIFO对象结构体
 * 用于实现先进先出的数据缓冲区
 */
typedef struct {
    uint8_t *buffer;        // FIFO缓冲区指针，指向实际存储数据的内存区域
    size_t size;            // 缓冲区对象多少，表示缓冲区可以存储的最大对象数
    size_t head;            // 读指针（队头），指向下一个要读取的数据位置
    size_t tail;            // 写指针（队尾），指向下一个要写入数据的位置
    size_t count;           // 当前元素数量，表示当前缓冲区中有效数据的字节数
} fifo_t;

/**
 * 初始化FIFO对象
 * @param fifo FIFO对象指针
 * @param buffer 缓冲区指针
 * @param size 缓冲区大小
 * @return 0表示成功，-1表示失败
 */
int fifo_init(fifo_t *fifo, uint8_t *buffer, size_t size);

/**
 * 向FIFO中写入数据
 * @param fifo FIFO对象指针
 * @param data 要写入的数据
 * @return 0表示成功，-1表示失败（FIFO满）
 */
int fifo_write(fifo_t *fifo, uint8_t data);

/**
 * 从FIFO中读取数据
 * @param fifo FIFO对象指针
 * @param data 读取到的数据存放地址，根据ItemSize大小确定具体写入数据长度
 * @return 0表示成功，-1表示失败（FIFO空）
 */
int fifo_read(fifo_t *fifo, uint8_t *data) ;

/**
 * 向FIFO批量写入数据
 * @param fifo FIFO对象指针
 * @param data 要写入的数据指针
 * @param len 要写入的数据长度
 * @return 实际写入的数据长度
 */
size_t fifo_write_multi(fifo_t *fifo, const uint8_t *data, size_t len);

/**
 * 从FIFO批量读取数据
 * @param fifo FIFO对象指针
 * @param data 读取数据存放的缓冲区指针
 * @param len 要读取的数据长度
 * @return 实际读取的数据长度
 */
size_t fifo_read_multi(fifo_t *fifo, uint8_t *data, size_t len);

/**
 * 获取FIFO当前元素数量
 * @param fifo FIFO对象指针
 * @return 当前元素数量
 */
size_t fifo_get_count(fifo_t *fifo);

/**
 * 获取FIFO剩余空间大小
 * @param fifo FIFO对象指针
 * @return 剩余空间大小
 */
size_t fifo_get_free_size(fifo_t *fifo);

/**
 * 检查FIFO是否为空
 * @param fifo FIFO对象指针
 * @return true表示空，false表示非空
 */
bool fifo_is_empty(fifo_t *fifo);

/**
 * 检查FIFO是否已满
 * @param fifo FIFO对象指针
 * @return true表示满，false表示未满
 */
bool fifo_is_full(fifo_t *fifo);

/**
 * 清空FIFO
 * @param fifo FIFO对象指针
 */
void fifo_clear(fifo_t *fifo);

/**
 * FIFO调试信息打印函数（可选，用于调试）
 * @param fifo FIFO对象指针
 */
void fifo_debug_info(fifo_t *fifo);

#ifdef __cplusplus
}
#endif

#endif
