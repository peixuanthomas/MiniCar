/**
  ******************************************************************************
  * @file           : UsrTimer.c
  * @brief          : Timer callback for high-priority line-following control
  ******************************************************************************
  */

#include "UsrTimer.h"
#include <stdbool.h>

extern volatile uint8_t runFlag;
extern volatile uint8_t oledProductMode;

#define SENSOR_STATE_FINE_CORRECTION    70
#define SENSOR_STATE_SMALL_CORRECTION   140
#define SENSOR_STATE_MEDIUM_CORRECTION  230
#define SENSOR_STATE_HARD_CORRECTION    320

#define LINE_STEP_TIMER_PERIOD_MS       2       // TIM1 period is 2ms.
#define LINE_STEP_JUDGE_INTERVAL_MS     500
#define LINE_STEP_FORWARD_TIME_MS       1000
#define LINE_STEP_FORWARD_SPEED         400
#define LINE_STEP_TURN_SPEED            260
#define LINE_STEP_TICKS(ms)             ((ms) / LINE_STEP_TIMER_PERIOD_MS)

static int   last_state_correction = 0;

typedef enum {
    LINE_STEP_WAIT_JUDGE = 0,
    LINE_STEP_TURN_LEFT,
    LINE_STEP_TURN_RIGHT,
    LINE_STEP_FORWARD,
} LineStepState;

static LineStepState line_step_state = LINE_STEP_WAIT_JUDGE;
static uint16_t line_step_ticks = 0;

volatile int16_t g_line_error_x10 = 0;
volatile int16_t g_line_correction = 0;
volatile int16_t g_line_left_speed = 0;
volatile int16_t g_line_right_speed = 0;
volatile uint8_t g_line_black_count = 0;

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
    last_state_correction = 0;
    line_step_state = LINE_STEP_WAIT_JUDGE;
    line_step_ticks = 0;
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

static bool is_line_centered(const bool sen[8], int black_count)
{
    if (black_count == 0) {
        return false;
    }
    if (black_count == 8) {
        return true;
    }

    return sen[3] || sen[4];
}

static LineStepState select_line_step_turn_state(const bool sen[8], int black_count)
{
    int correction = 0;

    if (black_count > 0) {
        correction = calc_sensor_state_correction(sen);
        if (correction != 0) {
            last_state_correction = correction;
        }
    } else {
        correction = last_state_correction;
    }

    if (correction < 0) {
        return LINE_STEP_TURN_LEFT;
    }
    if (correction > 0) {
        return LINE_STEP_TURN_RIGHT;
    }

    return (last_state_correction < 0) ? LINE_STEP_TURN_LEFT : LINE_STEP_TURN_RIGHT;
}

static void apply_line_step_stop(int black_count)
{
    Motor_SetSpeed(&motor_left, 0);
    Motor_SetSpeed(&motor_right, 0);
    publish_line_debug(0.0f, 0.0f, 0, 0, black_count);
}

static void apply_line_step_forward(int black_count)
{
    Motor_SetSpeed(&motor_left, LINE_STEP_FORWARD_SPEED);
    Motor_SetSpeed(&motor_right, LINE_STEP_FORWARD_SPEED);
    publish_line_debug(0.0f,
                       0.0f,
                       LINE_STEP_FORWARD_SPEED,
                       LINE_STEP_FORWARD_SPEED,
                       black_count);
}

static void apply_line_step_turn(LineStepState turn_state, int black_count)
{
    int left_speed = LINE_STEP_TURN_SPEED;
    int right_speed = -LINE_STEP_TURN_SPEED;
    float error = 1.0f;

    if (turn_state == LINE_STEP_TURN_LEFT) {
        left_speed = -LINE_STEP_TURN_SPEED;
        right_speed = LINE_STEP_TURN_SPEED;
        error = -1.0f;
    }

    Motor_SetSpeed(&motor_left, left_speed);
    Motor_SetSpeed(&motor_right, right_speed);
    publish_line_debug(error,
                       (float)(right_speed - left_speed),
                       left_speed,
                       right_speed,
                       black_count);
}

static void start_line_step_forward(int black_count)
{
    line_step_state = LINE_STEP_FORWARD;
    line_step_ticks = LINE_STEP_TICKS(LINE_STEP_FORWARD_TIME_MS);
    apply_line_step_forward(black_count);
}

static void run_sensor_state_control(void)
{
    bool sen[8];
    read_line_sensors(sen);

    int black_count = count_line_sensors(sen);

    switch (line_step_state) {
    case LINE_STEP_WAIT_JUDGE:
        apply_line_step_stop(black_count);
        if (line_step_ticks > 0) {
            line_step_ticks--;
            return;
        }

        if (is_line_centered(sen, black_count)) {
            start_line_step_forward(black_count);
        } else {
            line_step_state = select_line_step_turn_state(sen, black_count);
            apply_line_step_turn(line_step_state, black_count);
        }
        break;

    case LINE_STEP_TURN_LEFT:
    case LINE_STEP_TURN_RIGHT:
        if (is_line_centered(sen, black_count)) {
            start_line_step_forward(black_count);
            break;
        }

        if (black_count > 0) {
            line_step_state = select_line_step_turn_state(sen, black_count);
        }
        apply_line_step_turn(line_step_state, black_count);
        break;

    case LINE_STEP_FORWARD:
        apply_line_step_forward(black_count);
        if (line_step_ticks > 0) {
            line_step_ticks--;
            return;
        }

        line_step_state = LINE_STEP_WAIT_JUDGE;
        line_step_ticks = LINE_STEP_TICKS(LINE_STEP_JUDGE_INTERVAL_MS);
        apply_line_step_stop(black_count);
        break;

    default:
        reset_line_control_state();
        apply_line_step_stop(black_count);
        break;
    }
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

    run_sensor_state_control();
}
