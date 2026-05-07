/**
  ******************************************************************************
  * @file           : Motor.c
  * @brief          : 电机驱动C文件
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 上海师范大学 2025-2035
  * All rights reserved.
  *
  * 本代码仅限学习使用，未经作者许可，不得用于其它任何用途
  * 上海师范大学 信息与机电工程学院 通信工程专业
  * 开源地址：https://gitee.com/NEagle
  * 修改日期：2025/12/06
  * 版本： V1.0
  * 版权所有，违者必究
  * V1.0修改说明
  *
  ******************************************************************************
  */
#include "Motor.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

// 电机控制对象实例
MotorControl_t motor_left = {
    .pwm_timer = &htim3,
    .pwm_channel = TIM_CHANNEL_2,
    .dir_port = GPIOB,
    .dir_pin = GPIO_PIN_4,
};

MotorControl_t motor_right = {
    .pwm_timer = &htim4,
    .pwm_channel = TIM_CHANNEL_1,
    .dir_port = GPIOB,
    .dir_pin = GPIO_PIN_7,
};

void SystemClock_Config(void);
void HAL_MspInit(void);
void HAL_TIM_MspInit(TIM_HandleTypeDef* tim_handle);

void Motor_Init(MotorControl_t *motor);
void Motor_Stop(MotorControl_t *motor);
float Motor_GetSpeedRPM(MotorControl_t *motor);
void Encoder_Reset(MotorControl_t *motor);

void Encoder_Init(void);
void Encoder_IRQHandler(void);
int8_t Encoder_GetDirection(uint8_t prev_a, uint8_t prev_b, uint8_t curr_a, uint8_t curr_b);

void InitMotor(void)
{
    Motor_Stop(&motor_left);
    Motor_Stop(&motor_right);

    ResetLeftIn2();	// PB4=0, forward direction
    HAL_TIM_PWM_Start(motor_left.pwm_timer, motor_left.pwm_channel);

    ResetRightIn2();	// PB7=0, forward direction
    HAL_TIM_PWM_Start(motor_right.pwm_timer, motor_right.pwm_channel);
}

// 设置电机速度 (-999 ~ +999，负值为反转)
void Motor_SetSpeed(MotorControl_t *motor, int16_t speed_percent)
{
    // 限幅
    if (speed_percent > 999) speed_percent = 999;
    if (speed_percent < -999) speed_percent = -999;

    if (speed_percent >= 0) {
        // 正转：方向引脚复位
        HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, speed_percent);
    } else {
        // 反转：方向引脚置位，PWM取绝对值
        HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, -speed_percent);
    }
}

// 停止电机
void Motor_Stop(MotorControl_t *motor)
{
    __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, 0);
}
