/**
  ******************************************************************************
  * @file           : UsrTimer.c
  * @brief          : Timer callback for high-priority line-following control
  ******************************************************************************
  */

#include "UsrTimer.h"
#include "ObstacleAvoidance.h"
#include "stdio.h"
#include <stdbool.h>

/* Set true to use simple sensor-state control; set false to use PID control. */
static bool use_sensor_state_control = false;

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
#define MIN_SPEED         350     /* 降低使转向时内侧轮减速更多 */
#define MAX_SPEED         600
#define STRAIGHT_TEST_SPEED BASE_SPEED

/* 左右轮独立漂移补偿 (正值=该轮加速) */
#define LEFT_DRIFT_COMP   6
#define RIGHT_DRIFT_COMP  0

/* 原 SPEED_COMPENSATION 用于 sensor_state 和直行测试模式 */
#define SPEED_COMPENSATION 4

#define KP                15.0f
#define KI                0.0f    /* 微量积分消除稳态偏差 */
#define KD                1.2f
#define INTEGRAL_MAX      40.0f    /* 积分限幅 */
#define MAX_CORRECTION    300      /* 增大最大修正量 */

/* 微分低通滤波系数: 0=不滤波, 1=完全滤波 */
#define DERIVATIVE_LPF_ALPHA  0.3f

/* 条件积分阈值: |error| 小于此值时积分 */
#define INTEGRATE_THRESHOLD 3.0f

/* 最大加速度: 每 2ms tick 速度最大变化量 */
#define MAX_ACCEL_DELTA   50

#define LOST_LINE_TIMEOUT 30      // 30 * 2ms = 60ms
#define SEARCH_SPEED      250

#define SENSOR_STATE_BASE_SPEED         330
/* Minimum PWM that can move a wheel; use 0 for stop/pivot instead of low PWM. */
#define SENSOR_STATE_MIN_SPEED          420
#define SENSOR_STATE_MAX_SPEED          550
#define SENSOR_STATE_FINE_CORRECTION    90
#define SENSOR_STATE_SMALL_CORRECTION   120
#define SENSOR_STATE_MEDIUM_CORRECTION  210
#define SENSOR_STATE_HARD_CORRECTION    230
#define SENSOR_STATE_SEARCH_SPEED       400
/* Backward recovery uses short 330-PWM pulses to avoid continuous fast reverse. */
#define SENSOR_STATE_BACKWARD_SPEED     340
#define SENSOR_STATE_BACKWARD_ON_TICKS  2
#define SENSOR_STATE_BACKWARD_PERIOD_TICKS 10
#define POOR_TRACKING_TIMEOUT        100  // 100 * 2ms = 200ms

/* Sensor weights indexed by GetSen0Val()..GetSen7Val(). */
static const int8_t sensor_pos[8] = {-7, -5, -5, -1, 1, 5, 5, 7};

/* ==================== PID state ==================== */
static float integral              = 0.0f;
static float last_error            = 0.0f;
static float filtered_derivative   = 0.0f;
static float saved_error           = 0.0f;
static int   lost_counter          = 0;
static int   prev_left_speed       = 0;
static int   prev_right_speed      = 0;

/* ==================== Sensor state machine state ==================== */
static int   last_state_correction = 0;
static int   poor_tracking_counter = 0;
static uint8_t backward_pulse_tick = 0;
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
    filtered_derivative = 0.0f;
    saved_error = 0.0f;
    lost_counter = 0;
    prev_left_speed = 0;
    prev_right_speed = 0;

    last_state_correction = 0;
    poor_tracking_counter = 0;
    backward_pulse_tick = 0;
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

static bool line_needs_immediate_control(const bool sen[8], int black_count)
{
    bool has_center = sen[3] || sen[4];
    bool has_near = sen[2] || sen[5];

    return (black_count == 0) ||
           (!has_center && !has_near && (black_count <= 1));
}

static bool line_should_defer_obstacle_start(void)
{
    bool sen[8];
    int black_count;

    read_line_sensors(sen);
    black_count = count_line_sensors(sen);

    return line_needs_immediate_control(sen, black_count);
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

static int limit_sensor_state_speed(int speed)
{
    if (speed > 0) {
        return clamp(speed, SENSOR_STATE_MIN_SPEED, SENSOR_STATE_MAX_SPEED);
    }
    if (speed < 0) {
        return -clamp(-speed, SENSOR_STATE_MIN_SPEED, SENSOR_STATE_MAX_SPEED);
    }
    return 0;
}

static void calc_sensor_state_speeds(int correction, int *left_speed, int *right_speed)
{
    int magnitude = (correction < 0) ? -correction : correction;
    int inner_speed;
    int outer_speed;

    if (magnitude == 0) {
        *left_speed = SENSOR_STATE_BASE_SPEED + SPEED_COMPENSATION;
        *right_speed = SENSOR_STATE_BASE_SPEED - SPEED_COMPENSATION;
    } else if (magnitude <= SENSOR_STATE_FINE_CORRECTION) {
        inner_speed = SENSOR_STATE_MIN_SPEED;
        outer_speed = SENSOR_STATE_BASE_SPEED + SENSOR_STATE_FINE_CORRECTION;
        if (correction < 0) {
            *left_speed = inner_speed;
            *right_speed = outer_speed;
        } else {
            *left_speed = outer_speed;
            *right_speed = inner_speed;
        }
    } else if (magnitude <= SENSOR_STATE_SMALL_CORRECTION) {
        inner_speed = 0;
        outer_speed = SENSOR_STATE_BASE_SPEED + SENSOR_STATE_FINE_CORRECTION;
        if (correction < 0) {
            *left_speed = inner_speed;
            *right_speed = outer_speed;
        } else {
            *left_speed = outer_speed;
            *right_speed = inner_speed;
        }
    } else if (magnitude <= SENSOR_STATE_MEDIUM_CORRECTION) {
        inner_speed = 0;
        outer_speed = SENSOR_STATE_BASE_SPEED + SENSOR_STATE_SMALL_CORRECTION;
        if (correction < 0) {
            *left_speed = inner_speed;
            *right_speed = outer_speed;
        } else {
            *left_speed = outer_speed;
            *right_speed = inner_speed;
        }
    } else {
        inner_speed = -SENSOR_STATE_MIN_SPEED;
        outer_speed = SENSOR_STATE_MIN_SPEED;
        if (correction < 0) {
            *left_speed = inner_speed;
            *right_speed = outer_speed;
        } else {
            *left_speed = outer_speed;
            *right_speed = inner_speed;
        }
    }

    *left_speed = limit_sensor_state_speed(*left_speed);
    *right_speed = limit_sensor_state_speed(*right_speed);
}

static void apply_sensor_state_speeds(int correction, int black_count)
{
    int left_speed;
    int right_speed;

    calc_sensor_state_speeds(correction, &left_speed, &right_speed);

    Motor_SetSpeed(&motor_left, left_speed);
    Motor_SetSpeed(&motor_right, right_speed);
    publish_line_debug(sensor_state_error_from_correction(correction),
                       correction,
                       left_speed,
                       right_speed,
                       black_count);
}

static void apply_sensor_state_backward(int black_count)
{
    int backward_speed = 0;

    if (backward_pulse_tick < SENSOR_STATE_BACKWARD_ON_TICKS) {
        backward_speed = -SENSOR_STATE_BACKWARD_SPEED;
    }

    backward_pulse_tick++;
    if (backward_pulse_tick >= SENSOR_STATE_BACKWARD_PERIOD_TICKS) {
        backward_pulse_tick = 0;
    }

    Motor_SetSpeed(&motor_left, backward_speed);
    Motor_SetSpeed(&motor_right, backward_speed);
    publish_line_debug(0.0f, 0.0f,
                       backward_speed,
                       backward_speed,
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
            backward_pulse_tick = 0;
            correction = (black_count == 8) ? 0 : calc_sensor_state_correction(sen);
            last_state_correction = correction;
            apply_sensor_state_speeds(correction, black_count);
        } else {
            apply_sensor_state_backward(black_count);
        }
        return;
    }

    // All sensors lost: enter backward mode immediately
    if (black_count == 0) {
        in_backward_mode = true;
        backward_pulse_tick = 0;
        apply_sensor_state_backward(black_count);
        return;
    }

    // Detect poor tracking: only extreme edge sensors, barely on the line
    bool has_center = sen[3] || sen[4];
    bool has_near = sen[2] || sen[5];

    if (!has_center && !has_near && black_count <= 1) {
        poor_tracking_counter++;
        if (poor_tracking_counter > POOR_TRACKING_TIMEOUT) {
            in_backward_mode = true;
            backward_pulse_tick = 0;
            apply_sensor_state_backward(black_count);
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
        ObstacleAvoidance_Reset();
        reset_line_control_state();
        reset_line_debug();
        return;
    }

    if (ObstacleAvoidance_IsActive() || !line_should_defer_obstacle_start()) {
        if (ObstacleAvoidance_Update2ms()) {
            return;
        }
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

    /* ---- 丢线处理: 先直线后退, 再原地旋转寻线 ---- */
    if (black_count == 0) {
        lost_counter++;
        int l, r;
        if (lost_counter <= LOST_LINE_TIMEOUT) {
            /* 阶段1: 直线后退 */
            l = r = -SEARCH_SPEED;
        } else if (saved_error < 0) {
            /* 阶段2: 最后看到线在左边 -> 左转寻线 */
            l = -SEARCH_SPEED; r = SEARCH_SPEED;
        } else {
            /* 阶段2: 最后看到线在右边或未知 -> 右转寻线 */
            l = SEARCH_SPEED; r = -SEARCH_SPEED;
        }
        Motor_SetSpeed(&motor_left, l);
        Motor_SetSpeed(&motor_right, r);
        prev_left_speed = 0;    /* 清除加速限幅状态, 以便重新平稳起步 */
        prev_right_speed = 0;
        publish_line_debug(0.0f, 0.0f, l, r, black_count);
        return;
    }

    lost_counter = 0;

    if (black_count == 8) {
        error = 0.0f;
    } else {
        saved_error = error;
    }

    /* ---- 条件积分: 小误差时积分, 大误差时缓慢衰减 ---- */
    if (error > -INTEGRATE_THRESHOLD && error < INTEGRATE_THRESHOLD) {
        integral += error;
    } else {
        integral *= 0.95f;    /* 防弯道积分饱和 */
    }
    integral = clampf(integral, -INTEGRAL_MAX, INTEGRAL_MAX);

    /* ---- 微分 + 低通滤波 ---- */
    float raw_derivative = error - last_error;
    last_error = error;
    filtered_derivative = DERIVATIVE_LPF_ALPHA * filtered_derivative
                        + (1.0f - DERIVATIVE_LPF_ALPHA) * raw_derivative;

    /* ---- PID 输出 ---- */
    float correction = KP * error + KI * integral + KD * filtered_derivative;
    correction = clampf(correction, -MAX_CORRECTION, MAX_CORRECTION);

    /* ---- 速度映射 + 左右轮独立补偿 ---- */
    int left_speed  = clamp((int)(BASE_SPEED + correction + LEFT_DRIFT_COMP), MIN_SPEED, MAX_SPEED);
    int right_speed = clamp((int)(BASE_SPEED - correction + RIGHT_DRIFT_COMP), MIN_SPEED, MAX_SPEED);

    /* ---- 加速度限幅 (平滑过渡, 避免电机急跳) ---- */
    if (prev_left_speed != 0 || prev_right_speed != 0) {
        left_speed  = clamp(left_speed,  prev_left_speed - MAX_ACCEL_DELTA,  prev_left_speed + MAX_ACCEL_DELTA);
        right_speed = clamp(right_speed, prev_right_speed - MAX_ACCEL_DELTA, prev_right_speed + MAX_ACCEL_DELTA);
    }
    prev_left_speed  = left_speed;
    prev_right_speed = right_speed;

    Motor_SetSpeed(&motor_left, left_speed);
    Motor_SetSpeed(&motor_right, right_speed);
    publish_line_debug(error, correction, left_speed, right_speed, black_count);
}
