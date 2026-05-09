/**
  ******************************************************************************
  * @file           : UsrTimer.h
  * @brief          : 定时器用户入口
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

#ifndef _UsrTimer_H_
#define _UsrTimer_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#include "board.h"

extern volatile int16_t g_line_error_x10;
extern volatile int16_t g_line_correction;
extern volatile int16_t g_line_left_speed;
extern volatile int16_t g_line_right_speed;
extern volatile uint8_t g_line_black_count;

#ifdef __cplusplus
}
#endif

#endif
