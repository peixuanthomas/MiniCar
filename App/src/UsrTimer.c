/**
  ******************************************************************************
  * @file           : UsrTimer.c
  * @brief          : Timer callback for high-priority line-following control
  ******************************************************************************
  */

#include "UsrTimer.h"
#include "stdio.h"
#include <stdbool.h>

extern volatile uint8_t runFlag;
extern volatile uint8_t oledProductMode;

/* ==================== PID parameters ==================== */
#define BASE_SPEED        400
#define MIN_SPEED         300
#define MAX_SPEED         600
#define STRAIGHT_TEST_SPEED BASE_SPEED
#define SPEED_COMPENSATION 4      // Positive: correct left drift, left wheel + and right wheel -

#define KP                15.0f
#define KI                0.0f
#define KD                65.0f
#define INTEGRAL_MAX      40.0f
#define MAX_CORRECTION    300

#define LOST_LINE_TIMEOUT 30      // 30 * 2ms = 60ms
#define SEARCH_SPEED      250

/* Sensor weights indexed by GetSen0Val()..GetSen7Val(). */
static const int8_t sensor_pos[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

/* ==================== PID state ==================== */
static float integral     = 0.0f;
static float last_error   = 0.0f;
static float saved_error  = 0.0f;
static int   lost_counter = 0;
static volatile bool straight_test_enabled = false;

volatile int16_t g_line_error_x10 = 0;
volatile int16_t g_line_correction = 0;
volatile int16_t g_line_left_speed = 0;
volatile int16_t g_line_right_speed = 0;
volatile uint8_t g_line_black_count = 0;

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

static void publish_line_debug(float error, float correction, int left_speed, int right_speed, int black_count)
{
    g_line_error_x10 = (int16_t)(error * 10.0f);
    g_line_correction = (int16_t)correction;
    g_line_left_speed = (int16_t)left_speed;
    g_line_right_speed = (int16_t)right_speed;
    g_line_black_count = (uint8_t)black_count;
}

static void reset_line_debug(void)
{
    publish_line_debug(0.0f, 0.0f, 0, 0, 0);
}

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

    if (runFlag == 0) {
        Motor_SetSpeed(&motor_left, 0);
        Motor_SetSpeed(&motor_right, 0);
        integral = 0.0f;
        last_error = 0.0f;
        saved_error = 0.0f;
        lost_counter = 0;
        reset_line_debug();
        return;
    }

    if (straight_test_enabled) {
        int left_speed  = clamp(STRAIGHT_TEST_SPEED + SPEED_COMPENSATION, MIN_SPEED, MAX_SPEED);
        int right_speed = clamp(STRAIGHT_TEST_SPEED - SPEED_COMPENSATION, MIN_SPEED, MAX_SPEED);
        Motor_SetSpeed(&motor_left, left_speed);
        Motor_SetSpeed(&motor_right, right_speed);
        integral = 0.0f;
        last_error = 0.0f;
        saved_error = 0.0f;
        lost_counter = 0;
        publish_line_debug(0.0f, 0.0f, left_speed, right_speed, 0);
        return;
    }

    float error;
    int black_count;
    calc_sensor_error(&error, &black_count);

    if (black_count == 0) {
        lost_counter++;
        if (lost_counter < LOST_LINE_TIMEOUT) {
            error = saved_error;
        } else {
            if (saved_error < 0.0f) {
                Motor_SetSpeed(&motor_left, -SEARCH_SPEED);
                Motor_SetSpeed(&motor_right, SEARCH_SPEED);
                publish_line_debug(saved_error, 0.0f, -SEARCH_SPEED, SEARCH_SPEED, black_count);
            } else {
                Motor_SetSpeed(&motor_left, SEARCH_SPEED);
                Motor_SetSpeed(&motor_right, -SEARCH_SPEED);
                publish_line_debug(saved_error, 0.0f, SEARCH_SPEED, -SEARCH_SPEED, black_count);
            }
            return;
        }
    } else if (black_count == 8) {
        error = 0.0f;
        lost_counter = 0;
    } else {
        saved_error = error;
        lost_counter = 0;
    }

    if (lost_counter == 0) {
        integral += error;
        integral = clampf(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
    }

    float derivative = error - last_error;
    last_error = error;

    float correction = KP * error + KI * integral + KD * derivative + SPEED_COMPENSATION;
    correction = clampf(correction, -MAX_CORRECTION, MAX_CORRECTION);

    int left_speed  = clamp((int)(BASE_SPEED + correction), MIN_SPEED, MAX_SPEED);
    int right_speed = clamp((int)(BASE_SPEED - correction), MIN_SPEED, MAX_SPEED);

    Motor_SetSpeed(&motor_left, left_speed);
    Motor_SetSpeed(&motor_right, right_speed);
    publish_line_debug(error, correction, left_speed, right_speed, black_count);
}
