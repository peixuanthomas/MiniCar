/**
  ******************************************************************************
  * @file           : UsrTimer.c
  * @brief          : Timer callback for high-priority line-following control
  ******************************************************************************
  */

#include "UsrTimer.h"
#include "stdio.h"
#include <stdbool.h>

/* Set true to use simple sensor-state control; set false to use PID control. */
static bool use_sensor_state_control = true;

extern volatile uint8_t runFlag;
extern volatile uint8_t oledProductMode;

/* ==================== PID 参数调试说明 ====================
 * 调参顺序：
 * 1. 先确认传感器方向和电机修正方向。
 *    如果小车越修越远，先改 correction 的符号或左右轮输出映射，
 *    不要先改 KP/KI/KD。
 * 2. 先设置 KI = 0、KD = 0，低 BASE_SPEED 下只调 KP。
 * 3. KP 已经能跟线但开始左右摆动后，再逐步加入 KD。
 * 4. PD 稳定后仍有固定偏差时，最后再加很小的 KI。
 *
 * 现象 -> 调整方法：
 * - 完全没有修正：
 *   看 OLED 上的 g_line_error_x10。如果一直是 0，先修传感器/误差算法。
 *   如果误差变化但轮子不变，检查 correction 符号或电机输出。
 * - 反应太慢、慢慢偏出线：
 *   适当增大 KP，或者调试阶段先降低 BASE_SPEED。
 * - 左右大幅蛇形摆动：
 *   KP 过大，或者 KD 不足。先降低 KP，再逐步增加 KD。
 * - 高频抖动、轮子一顿一顿：
 *   KD 过大，或者传感器噪声太强。降低 KD。
 * - 直线能跟，进弯转不过去：
 *   降低 BASE_SPEED，降低 MIN_SPEED，或者增大 MAX_CORRECTION。
 * - PD 稳定后仍长期偏向一侧：
 *   优先调 SPEED_COMPENSATION。KI 只作为最后的小幅修正。
 * - 修正时慢侧轮降不下来：
 *   MIN_SPEED 太高，降低 MIN_SPEED。
 */
#define BASE_SPEED        400
#define MIN_SPEED         300
#define MAX_SPEED         600
#define STRAIGHT_TEST_SPEED BASE_SPEED
#define SPEED_COMPENSATION 4      // Positive: correct left drift, left wheel + and right wheel -

#define KP                2.0f
#define KI                0.0f
#define KD                0.7f
#define INTEGRAL_MAX      40.0f
#define MAX_CORRECTION    300

#define LOST_LINE_TIMEOUT 30      // 30 * 2ms = 60ms
#define SEARCH_SPEED      250

#define SENSOR_STATE_BASE_SPEED         380
#define SENSOR_STATE_MIN_SPEED          300
#define SENSOR_STATE_MAX_SPEED          480
#define SENSOR_STATE_FINE_CORRECTION    70
#define SENSOR_STATE_SMALL_CORRECTION   140
#define SENSOR_STATE_MEDIUM_CORRECTION  230
#define SENSOR_STATE_HARD_CORRECTION    320
#define SENSOR_STATE_SEARCH_SPEED       220
#define SENSOR_STATE_BACKWARD_SPEED     1
#define POOR_TRACKING_TIMEOUT        100  // 100 * 2ms = 200ms

/* Sensor weights indexed by GetSen0Val()..GetSen7Val(). */
static const int8_t sensor_pos[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

/* ==================== PID state ==================== */
static float integral     = 0.0f;
static float last_error   = 0.0f;
static float saved_error  = 0.0f;
static int   lost_counter = 0;
static int   last_state_correction = 0;
static int   poor_tracking_counter = 0;
static bool  in_backward_mode = false;
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

static void reset_line_control_state(void)
{
    integral = 0.0f;
    last_error = 0.0f;
    saved_error = 0.0f;
    lost_counter = 0;
    last_state_correction = 0;
    poor_tracking_counter = 0;
    in_backward_mode = false;
}

static void read_line_sensors(bool sen[8])
{
    sen[0] = GetSen0Val();  // GetSenXVal() == 1 means black line detected
    sen[1] = GetSen1Val();
    sen[2] = GetSen2Val();
    sen[3] = GetSen3Val();
    sen[4] = GetSen4Val();
    sen[5] = GetSen5Val();
    sen[6] = GetSen6Val();
    sen[7] = GetSen7Val();
}

static int count_line_sensors(const bool sen[8])
{
    int cnt = 0;

    for (int i = 0; i < 8; i++) {
        if (sen[i]) {
            cnt++;
        }
    }

    return cnt;
}

static void calc_sensor_error(float *error, int *count)
{
    bool sen[8];
    read_line_sensors(sen);

    int sum_pos = 0;
    int cnt = count_line_sensors(sen);

    for (int i = 0; i < 8; i++) {
        if (sen[i]) {
            sum_pos += sensor_pos[i];
        }
    }

    *count = cnt;
    if (cnt > 0) {
        *error = (float)sum_pos / (float)cnt;
    } else {
        *error = 0.0f;
    }
}

static int calc_sensor_state_correction(const bool sen[8])
{
    if (sen[3] && sen[4]) return 0;
    if (sen[3]) return -SENSOR_STATE_FINE_CORRECTION;
    if (sen[4]) return SENSOR_STATE_FINE_CORRECTION;

    if (sen[2]) return -SENSOR_STATE_SMALL_CORRECTION;
    if (sen[5]) return SENSOR_STATE_SMALL_CORRECTION;

    if (sen[1]) return -SENSOR_STATE_MEDIUM_CORRECTION;
    if (sen[6]) return SENSOR_STATE_MEDIUM_CORRECTION;

    if (sen[0]) return -SENSOR_STATE_HARD_CORRECTION;
    if (sen[7]) return SENSOR_STATE_HARD_CORRECTION;

    return 0;
}

static float sensor_state_error_from_correction(int correction)
{
    if (correction < 0) {
        return -1.0f;
    }
    if (correction > 0) {
        return 1.0f;
    }
    return 0.0f;
}

static void apply_sensor_state_speeds(int correction, int black_count)
{
    int total_correction = correction + SPEED_COMPENSATION;
    int left_speed = clamp(SENSOR_STATE_BASE_SPEED + total_correction,
                           SENSOR_STATE_MIN_SPEED,
                           SENSOR_STATE_MAX_SPEED);
    int right_speed = clamp(SENSOR_STATE_BASE_SPEED - total_correction,
                            SENSOR_STATE_MIN_SPEED,
                            SENSOR_STATE_MAX_SPEED);

    Motor_SetSpeed(&motor_left, left_speed);
    Motor_SetSpeed(&motor_right, right_speed);
    publish_line_debug(sensor_state_error_from_correction(correction),
                       total_correction,
                       left_speed,
                       right_speed,
                       black_count);
}

static void run_sensor_state_control(void)
{
    bool sen[8];
    read_line_sensors(sen);

    int black_count = count_line_sensors(sen);
    int correction = 0;

    // In backward mode: stay backward until line is clearly re-acquired
    if (in_backward_mode) {
        if (black_count >= 2 || sen[3] || sen[4]) {
            in_backward_mode = false;
            poor_tracking_counter = 0;
            lost_counter = 0;
            correction = (black_count == 8) ? 0 : calc_sensor_state_correction(sen);
            last_state_correction = correction;
            apply_sensor_state_speeds(correction, black_count);
        } else {
            Motor_SetSpeed(&motor_left, -SENSOR_STATE_BACKWARD_SPEED);
            Motor_SetSpeed(&motor_right, -SENSOR_STATE_BACKWARD_SPEED);
            publish_line_debug(0.0f, 0.0f,
                               -SENSOR_STATE_BACKWARD_SPEED,
                               -SENSOR_STATE_BACKWARD_SPEED,
                               black_count);
        }
        return;
    }

    // All sensors lost: enter backward mode immediately
    if (black_count == 0) {
        in_backward_mode = true;
        Motor_SetSpeed(&motor_left, -SENSOR_STATE_BACKWARD_SPEED);
        Motor_SetSpeed(&motor_right, -SENSOR_STATE_BACKWARD_SPEED);
        publish_line_debug(0.0f, 0.0f,
                           -SENSOR_STATE_BACKWARD_SPEED,
                           -SENSOR_STATE_BACKWARD_SPEED,
                           black_count);
        return;
    }

    // Detect poor tracking: only extreme edge sensors, barely on the line
    bool has_center = sen[3] || sen[4];
    bool has_near = sen[2] || sen[5];

    if (!has_center && !has_near && black_count <= 1) {
        poor_tracking_counter++;
        if (poor_tracking_counter > POOR_TRACKING_TIMEOUT) {
            in_backward_mode = true;
            Motor_SetSpeed(&motor_left, -SENSOR_STATE_BACKWARD_SPEED);
            Motor_SetSpeed(&motor_right, -SENSOR_STATE_BACKWARD_SPEED);
            publish_line_debug(0.0f, 0.0f,
                               -SENSOR_STATE_BACKWARD_SPEED,
                               -SENSOR_STATE_BACKWARD_SPEED,
                               black_count);
            return;
        }
    } else {
        poor_tracking_counter = 0;
    }

    lost_counter = 0;
    correction = (black_count == 8) ? 0 : calc_sensor_state_correction(sen);
    last_state_correction = correction;
    apply_sensor_state_speeds(correction, black_count);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM1) return;
    if (oledProductMode) return;

    if (runFlag == 0) {
        Motor_SetSpeed(&motor_left, 0);
        Motor_SetSpeed(&motor_right, 0);
        reset_line_control_state();
        reset_line_debug();
        return;
    }

    if (use_sensor_state_control) {
        integral = 0.0f;
        last_error = 0.0f;
        saved_error = 0.0f;
        run_sensor_state_control();
        return;
    }

    if (straight_test_enabled) {
        int left_speed  = clamp(STRAIGHT_TEST_SPEED + SPEED_COMPENSATION, MIN_SPEED, MAX_SPEED);
        int right_speed = clamp(STRAIGHT_TEST_SPEED - SPEED_COMPENSATION, MIN_SPEED, MAX_SPEED);
        Motor_SetSpeed(&motor_left, left_speed);
        Motor_SetSpeed(&motor_right, right_speed);
        reset_line_control_state();
        publish_line_debug(0.0f, 0.0f, left_speed, right_speed, 0);
        return;
    }

    float error;
    int black_count;
    calc_sensor_error(&error, &black_count);

    if (black_count == 0) {
        Motor_SetSpeed(&motor_left, -SEARCH_SPEED);
        Motor_SetSpeed(&motor_right, -SEARCH_SPEED);
        publish_line_debug(0.0f, 0.0f, -SEARCH_SPEED, -SEARCH_SPEED, black_count);
        return;
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
