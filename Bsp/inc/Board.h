/**
  ******************************************************************************
  * @file           : Board.h
  * @brief          : 板级HAL抽象层头文件
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
#ifndef _BOARD_H_
#define _BOARD_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#include "uart.h"
#include "laser_distance.h"
#include "i2c_oled.h"
#include "oled_api.h"
#include "adc_bat.h"
#include "Motor.h"

//核心板LED定义
#define CoreLedON()	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)
#define CoreLedOFF()	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define CoreLedToggle()	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13)

//板载LED定义
#define BrdLedON()	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET)
#define BrdLedOFF()	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET)
#define BrdLedToggle()	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11)

//蜂鸣器定义
#define BuzzON()	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)
#define BuzzOFF()	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)
#define BuzzToggle()	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3)

//按键定义
#define GetKey0Val()	(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14)? 0:1)
#define GetKey1Val()	(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)? 0:1)

//传感器输入
#define GetSen0Val()	(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)? 0:1)
#define GetSen1Val()	(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)? 0:1)
#define GetSen2Val()	(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)? 0:1)
#define GetSen3Val()	(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)? 0:1)
#define GetSen4Val()	(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)? 0:1)
#define GetSen5Val()	(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)? 0:1)
#define GetSen6Val()	(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)? 0:1)
#define GetSen7Val()	(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)? 0:1)

/** 板级外设初始化函数
  * @brief  被Main函数调用的板级初始化函数
  * @param  None
  * @retval None
  */
void BoardInit(void);

#ifdef __cplusplus
}
#endif

#endif
