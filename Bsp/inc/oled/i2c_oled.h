/**
  ******************************************************************************
  * @file           : i2c_oled.h
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
  * 修改日期：2025/12/10
  * 版本： V1.0
  * 版权所有，盗版必究
  * V1.0修改说明
  *
  ******************************************************************************
  */

#ifndef _I2C_OLED_H_
#define _I2C_OLED_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/**
 * @brief OLED写命令
 * @param Command 要写入的命令值，范围：0x00~0xFF
 * @return 无
 */
void OLED_WriteCommand(uint8_t Command);

/**
 * @brief OLED写数据
 * @param Data 要写入数据的起始地址
 * @param Count 要写入数据的数量
 * @return 无
 */
void OLED_WriteData(uint8_t *Data, uint8_t Count);

/**
 * @brief OLED引脚初始化
 * @param  无
 * @retval 无
 * @note 当上层函数需要初始化时，此函数会被调用,
 *       用户需要将SCL和SDA引脚初始化为开漏模式，并释放引脚
 */
void OLED_GPIO_Init(void);

#ifdef __cplusplus
}
#endif

#endif
