/**
  ******************************************************************************
  * @file           : adc_bat.h
  * @brief          : 电池电压采样
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
#ifndef _ADC_BAT_H_
#define _ADC_BAT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

void AdcReadTask(void);

#ifdef __cplusplus
}
#endif

#endif
