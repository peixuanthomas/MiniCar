/**
  ******************************************************************************
  * @file           : Motor.c
  * @brief          : 电机控制C文件
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
#include "Motor.h"

extern TIM_HandleTypeDef htim3;	// PWM定时器句柄，用于控制电机PWM输出
extern TIM_HandleTypeDef htim4;

// 创建电机控制实例
MotorControl_t motor_left = {
    .pwm_timer = &htim3,              // 指向TIM3定时器句柄
    .pwm_channel1 = TIM_CHANNEL_2,    // TIM3_CH1 -> PB4
};

MotorControl_t motor_right = {
    .pwm_timer = &htim4,              // 指向TIM4定时器句柄
    .pwm_channel1 = TIM_CHANNEL_1,    // TIM4_CH1 -> PB6
};

/* 函数声明 */
void SystemClock_Config(void);        // 系统时钟配置函数声明
//static void MX_GPIO_Init(void);       // GPIO初始化函数声明
//static void MX_TIM3_Init(void);       // TIM3定时器初始化函数声明
void HAL_MspInit(void);               // HAL库MSP初始化函数声明
void HAL_TIM_MspInit(TIM_HandleTypeDef* tim_handle);  // TIM MSP初始化函数声明

// 电机控制函数声明
void Motor_Init(MotorControl_t *motor);       // 电机初始化函数声明
void Motor_Stop(MotorControl_t *motor);       // 电机停止函数声明
float Motor_GetSpeedRPM(MotorControl_t *motor); // 获取电机转速函数声明
void Encoder_Reset(MotorControl_t *motor);    // 编码器清零函数声明

// 编码器相关函数
void Encoder_Init(void);              // 编码器初始化函数声明
void Encoder_IRQHandler(void);        // 编码器中断处理函数声明
int8_t Encoder_GetDirection(uint8_t prev_a, uint8_t prev_b, uint8_t curr_a, uint8_t curr_b); // 获取编码器方向函数声明

void InitMotor(void)					// 主函数定义
{

    // 初始化电机控制
    Motor_Stop(&motor_left);                  // 调用停止电机函数
    Motor_Stop(&motor_right);                  // 调用停止电机函数

    // 启动PWM输出
	ResetLeftIn2();	// PB4设置为0
    HAL_TIM_PWM_Start(motor_left.pwm_timer, motor_left.pwm_channel1);  // 启动TIM3_CH1的PWM输出
	
	ResetRightIn2();	//PB7设置为0
    HAL_TIM_PWM_Start(motor_right.pwm_timer, motor_right.pwm_channel1);  // 启动TIM3_CH1的PWM输出

}

// 设置电机速度 (-999 到 +999，负数为反转)
void Motor_SetSpeed(MotorControl_t *motor, int16_t speed_percent)  // 设置电机速度函数实现
{
    // 限制速度范围
	if(speed_percent < 0) return;
    if(speed_percent > 999) speed_percent = 999;  // 速度上限为99.9%

	__HAL_TIM_SET_COMPARE(motor->pwm_timer,motor->pwm_channel1, speed_percent);
}

// 停止电机
void Motor_Stop(MotorControl_t *motor)  // 停止电机函数实现
{
    // 设置两个PWM占空比为0
    __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel1, 0); // 设置PB4的PWM占空比为0
}

