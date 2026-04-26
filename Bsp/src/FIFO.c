/**
  ******************************************************************************
  * @file           : FIFO.c
  * @brief          : 先进先出缓冲区文件
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
#include "FIFO.h"

/**
 * 初始化FIFO对象
 * @param fifo FIFO对象指针
 * @param buffer 缓冲区指针
 * @param size 缓冲区大小
 * @return 0表示成功，-1表示失败
 */
int fifo_init(fifo_t *fifo, uint8_t *buffer, size_t size) {
    if (fifo == NULL || buffer == NULL || size == 0) {  // 检查参数有效性
        return -1;  // 参数无效，返回错误码
    }
    
    fifo->buffer = buffer;  // 设置缓冲区指针
    fifo->size = size;      // 设置缓冲区大小
    fifo->head = 0;         // 初始化读指针为0
    fifo->tail = 0;         // 初始化写指针为0
    fifo->count = 0;        // 初始化计数器为0
    
    return 0;  // 初始化成功，返回成功码
}

/**
 * 向FIFO中写入数据
 * @param fifo FIFO对象指针
 * @param data 要写入的数据
 * @return 0表示成功，-1表示失败（FIFO满）
 */
int fifo_write(fifo_t *fifo, uint8_t data) {
    if (fifo == NULL || fifo->count >= fifo->size) {  // 检查FIFO是否为空或已满
        return -1;  // FIFO已满或参数无效，返回错误码
    }
    
    fifo->buffer[fifo->tail] = data;        // 将数据写入缓冲区的tail位置
    fifo->tail = (fifo->tail + 1) % fifo->size;  // 更新tail指针，使用模运算实现循环
    fifo->count++;                          // 增加计数器
    
    return 0;  // 写入成功，返回成功码
}

/**
 * 从FIFO中读取数据
 * @param fifo FIFO对象指针
 * @param data 读取到的数据存放地址
 * @return 0表示成功，-1表示失败（FIFO空）
 */
int fifo_read(fifo_t *fifo, uint8_t *data) {
    if (fifo == NULL || data == NULL || fifo->count == 0) {  // 检查参数有效性及FIFO是否为空
        return -1;  // FIFO为空或参数无效，返回错误码
    }
    
    *data = fifo->buffer[fifo->head];       // 从head位置读取数据到data指针指向的位置
    fifo->head = (fifo->head + 1) % fifo->size;  // 更新head指针，使用模运算实现循环
    fifo->count--;                          // 减少计数器
    
    return 0;  // 读取成功，返回成功码
}

/**
 * 向FIFO批量写入数据
 * @param fifo FIFO对象指针
 * @param data 要写入的数据指针
 * @param len 要写入的数据长度
 * @return 实际写入的数据长度
 */
size_t fifo_write_multi(fifo_t *fifo, const uint8_t *data, size_t len) {
    if (fifo == NULL || data == NULL) {  // 检查参数有效性
        return 0;  // 参数无效，返回0
    }
    
    size_t available = fifo->size - fifo->count;  // 计算FIFO中可用空间大小
    size_t write_len = (len < available) ? len : available;  // 确定实际可写入的数据长度
    
    for (size_t i = 0; i < write_len; i++) {  // 循环写入数据
        fifo->buffer[fifo->tail] = data[i];  // 将数据写入缓冲区
        fifo->tail = (fifo->tail + 1) % fifo->size;  // 更新tail指针
    }
    
    fifo->count += write_len;  // 增加计数器
    return write_len;  // 返回实际写入的数据长度
}

/**
 * 从FIFO批量读取数据
 * @param fifo FIFO对象指针
 * @param data 读取数据存放的缓冲区指针
 * @param len 要读取的数据长度
 * @return 实际读取的数据长度
 */
size_t fifo_read_multi(fifo_t *fifo, uint8_t *data, size_t len) {
    if (fifo == NULL || data == NULL) {  // 检查参数有效性
        return 0;  // 参数无效，返回0
    }
    
    size_t available = fifo->count;  // 获取FIFO中当前数据量
    size_t read_len = (len < available) ? len : available;  // 确定实际可读取的数据长度
    
    for (size_t i = 0; i < read_len; i++) {  // 循环读取数据
        data[i] = fifo->buffer[fifo->head];  // 从缓冲区读取数据到目标位置
        fifo->head = (fifo->head + 1) % fifo->size;  // 更新head指针
    }
    
    fifo->count -= read_len;  // 减少计数器
    return read_len;  // 返回实际读取的数据长度
}

/**
 * 获取FIFO当前元素数量
 * @param fifo FIFO对象指针
 * @return 当前元素数量
 */
size_t fifo_get_count(fifo_t *fifo) {
    if (fifo == NULL) {  // 检查参数有效性
        return 0;  // 参数无效，返回0
    }
    return fifo->count;  // 返回当前元素数量
}

/**
 * 获取FIFO剩余空间大小
 * @param fifo FIFO对象指针
 * @return 剩余空间大小
 */
size_t fifo_get_free_size(fifo_t *fifo) {
    if (fifo == NULL) {  // 检查参数有效性
        return 0;  // 参数无效，返回0
    }
    return fifo->size - fifo->count;  // 返回剩余空间大小
}

/**
 * 检查FIFO是否为空
 * @param fifo FIFO对象指针
 * @return true表示空，false表示非空
 */
bool fifo_is_empty(fifo_t *fifo) {
    if (fifo == NULL) {  // 检查参数有效性
        return true;  // 参数无效时视为为空
    }
    return fifo->count == 0;  // 检查计数器是否为0
}

/**
 * 检查FIFO是否已满
 * @param fifo FIFO对象指针
 * @return true表示满，false表示未满
 */
bool fifo_is_full(fifo_t *fifo) {
    if (fifo == NULL) {  // 检查参数有效性
        return true;  // 参数无效时视为已满
    }
    return fifo->count >= fifo->size;  // 检查计数器是否达到或超过缓冲区大小
}

/**
 * 清空FIFO
 * @param fifo FIFO对象指针
 */
void fifo_clear(fifo_t *fifo) {
    if (fifo != NULL) {  // 检查参数有效性
        fifo->head = 0;      // 重置读指针为0
        fifo->tail = 0;      // 重置写指针为0
        fifo->count = 0;     // 重置计数器为0
    }
}

/**
 * FIFO调试信息打印函数（可选，用于调试）
 * @param fifo FIFO对象指针
 */
void fifo_debug_info(fifo_t *fifo) {
    if (fifo != NULL) {  // 检查参数有效性
        // 在实际嵌入式系统中，可能需要通过串口打印
        // printf("FIFO: Size=%d, Count=%d, Head=%d, Tail=%d\n", 
        //        fifo->size, fifo->count, fifo->head, fifo->tail);
    }
}

