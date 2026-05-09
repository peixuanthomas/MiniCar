/**
  ******************************************************************************
  * @file           : UsrTimer.c
  * @brief          : 定时器用户回调 — PID 循迹控制
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 上海师范大学 2025-2035
  * All rights reserved.
  *
  * 本代码仅限学习使用，未经作者许可，不得用于其它任何用途
  * 上海师范大学 信息与机电工程学院 通信工程专业
  * 开源地址：https://gitee.com/NEagle
  * 修改日期：2026/05/07
  * 版本： V2.0 (8-sensor PID)
  * 版权所有，违者必究
  *
  ******************************************************************************
  */

#include "UsrTimer.h"
#include "stdio.h"
#include <stdbool.h>

extern volatile uint8_t runFlag;
extern volatile uint8_t oledProductMode;

/* ==================== PID 可调参数 ==================== */
#define BASE_SPEED        450     // 基础速度 (0-999)
#define MIN_SPEED         200     // 最低速度
#define MAX_SPEED         750     // 最高速度
#define STRAIGHT_TEST_SPEED BASE_SPEED  // Straight-line test speed
#define SPEED_COMPENSATION 4      // Positive: correct left drift, left wheel + and right wheel -

#define KP                28.0f   // 比例增益
#define KI                0.3f    // 积分增益
#define KD                65.0f   // 微分增益
#define INTEGRAL_MAX      40.0f   // 积分限幅 (anti-windup)
#define MAX_CORRECTION    300     // 输出修正限幅

#define LOST_LINE_TIMEOUT 30      // 丢线超时 (×0.5ms ≈ 15ms)
#define SEARCH_SPEED      250     // 丢线寻线旋转速度

/* ==================== 传感器位置权重 ==================== */
// Sen0(PA7,最右)=+7 ... Sen7(PA0,最左)=-7，按传感器编号索引
static const int8_t sensor_pos[8] = {7, 5, 3, 1, -1, -3, -5, -7};

/* ==================== PID 状态变量 ==================== */
static float integral     = 0.0f;
static float last_error   = 0.0f;
static float saved_error  = 0.0f;   // 丢线时保存的最后有效误差
static int   lost_counter = 0;
static volatile bool straight_test_enabled = false; //turn this off to enable PID control

static inline int clamp(int val, int lo, int hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

static inline float clampf(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/**
  * @brief  读取全部8路传感器，计算质心误差
  * @param  error  输出误差值（0=正中，负=偏左，正=偏右）
  * @param  count  输出检测到黑线的传感器数量
  */
static void calc_sensor_error(float *error, int *count)
{
    bool sen[8];
    sen[0] = GetSen0Val();  // GetSenXVal() == 1 means black line detected
    sen[1] = GetSen1Val();
    sen[2] = GetSen2Val();
    sen[3] = GetSen3Val();
    sen[4] = GetSen4Val();
    sen[5] = GetSen5Val();
    sen[6] = GetSen6Val();
    sen[7] = GetSen7Val();

    int sum_pos = 0;
    int cnt = 0;

    for (int i = 0; i < 8; i++) {
        if (sen[i]) {
            sum_pos += sensor_pos[i];
            cnt++;
        }
    }

    *count = cnt;
    if (cnt > 0) {
        *error = (float)sum_pos / (float)cnt;
    } else {
        *error = 0.0f;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM1) return;
    if (oledProductMode) return;

    OLED_ShowNum(48, 0, runFlag, 1, OLED_8X16);

    if (runFlag == 0) {
        Motor_SetSpeed(&motor_left, 0);
        Motor_SetSpeed(&motor_right, 0);
        integral = 0.0f;
        last_error = 0.0f;
        saved_error = 0.0f;
        lost_counter = 0;
        return;
    }

    if (straight_test_enabled) {
        /* Straight-line test mode: ignore sensors/PID, keep only wheel trim. */
        int left_speed  = clamp(STRAIGHT_TEST_SPEED + SPEED_COMPENSATION, MIN_SPEED, MAX_SPEED);
        int right_speed = clamp(STRAIGHT_TEST_SPEED - SPEED_COMPENSATION, MIN_SPEED, MAX_SPEED);
        Motor_SetSpeed(&motor_left, left_speed);
        Motor_SetSpeed(&motor_right, right_speed);
        integral = 0.0f;
        last_error = 0.0f;
        saved_error = 0.0f;
        lost_counter = 0;
        return;
    }

    /* ——— 1. 传感器误差计算 ——— */
    float error;
    int black_count;
    calc_sensor_error(&error, &black_count);

    /* ——— 2. 丢线处理 ——— */
    if (black_count == 0) {
        lost_counter++;
        if (lost_counter < LOST_LINE_TIMEOUT) {
            // 短期丢线：用最后有效误差，降低增益继续修正
            error = saved_error;
        } else {
            // 长期丢线：原地旋转寻线
            if (saved_error < 0.0f) {
                // 线在左边，左转寻线
                Motor_SetSpeed(&motor_left, -SEARCH_SPEED);
                Motor_SetSpeed(&motor_right, SEARCH_SPEED);
            } else {
                // 线在右边，右转寻线
                Motor_SetSpeed(&motor_left, SEARCH_SPEED);
                Motor_SetSpeed(&motor_right, -SEARCH_SPEED);
            }
            return;
        }
    } else if (black_count == 8) {
        // 全部检测到黑线（十字路口/粗线）：直行
        error = 0.0f;
        lost_counter = 0;
    } else {
        // 正常检测
        saved_error = error;
        lost_counter = 0;
    }

    /* ——— 3. PID 计算 ——— */
    // 积分（丢线期间不累积）
    if (lost_counter == 0) {
        integral += error;
        integral = clampf(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
    }

    float derivative = error - last_error;
    last_error = error;

    float correction = KP * error + KI * integral + KD * derivative + SPEED_COMPENSATION;
    correction = clampf(correction, -MAX_CORRECTION, MAX_CORRECTION);

    /* ——— 4. 电机输出 ——— */
    int left_speed  = clamp((int)(BASE_SPEED + correction), MIN_SPEED, MAX_SPEED);
    int right_speed = clamp((int)(BASE_SPEED - correction), MIN_SPEED, MAX_SPEED);

    Motor_SetSpeed(&motor_left, left_speed);
    Motor_SetSpeed(&motor_right, right_speed);

    /* ——— 5. OLED 调试显示 ——— */
    // 第4行显示 error*10 和 correction
    OLED_ShowString(0, 48, "e=", OLED_8X16);
    OLED_ShowSignedNum(16, 48, (int32_t)(error * 10.0f), 3, OLED_8X16);
    OLED_ShowString(48, 48, "c=", OLED_8X16);
    OLED_ShowSignedNum(64, 48, (int32_t)correction, 4, OLED_8X16);
}
