/**
  ******************************************************************************
  * @file           : Motor.h
  * @brief          : 电机控制头文件
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
#ifndef _MOTOR_H_
#define _MOTOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define MOTOR_PWM_PORT    GPIOB       // PWM控制端口定义为GPIOB
#define ENCODER_PORT      GPIOB       // 编码器端口定义为GPIOB

//左轮定义
#define SetLeftIn2()	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)
#define ResetLeftIn2()	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)

//右轮定义
#define SetRightIn2()	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET)
#define ResetRightIn2()	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET)

// 定义PWM参数
//#define PWM_PERIOD        1000  // PWM周期值，用于设置PWM占空比计算

// 电机控制结构体
typedef struct {
    TIM_HandleTypeDef *pwm_timer;     // PWM定时器句柄，用于控制电机速度
    uint32_t pwm_channel1;            // PWM通道1 (PB4 - TIM3_CH1)
} MotorControl_t;                     // 电机控制结构体定义
extern MotorControl_t motor_left;
extern MotorControl_t motor_right;
void InitMotor(void);					// 主函数定义
void Motor_SetSpeed(MotorControl_t *motor, int16_t speed_percent);  // 设置电机速度函数实现

#ifdef __cplusplus
}
#endif

#endif
