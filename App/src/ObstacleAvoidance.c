#include "ObstacleAvoidance.h"

#include "laser_distance.h"

#ifndef OBSTACLE_AVOIDANCE_TEST
#include "Board.h"
#include "TaskMain.h"
#endif

/*
 * Current triangle bypass route:
 * 1) turn out by a small angle;
 * 2) drive the first equal leg;
 * 3) turn back through the obtuse apex, using twice the small turn time;
 * 4) drive the second equal leg back toward the original line;
 * 5) turn by the same small angle to hand control back to line following.
 *
 * Tune OBSTACLE_TURN_TICKS for the small side angle, and tune
 * OBSTACLE_TRIANGLE_LEG_TICKS for how far the car travels before returning.
 */
#define OBSTACLE_TRIGGER_MM 600U
#define OBSTACLE_CLEAR_MM 370U
#define OBSTACLE_MIN_USABLE_MM 40U
#define OBSTACLE_APPROACH_START_MM 650U
#define OBSTACLE_APPROACH_TOLERANCE_MM 15U
#define OBSTACLE_MIN_APPROACH_SAMPLES 3U
#define OBSTACLE_MIN_APPROACH_DECREASES 3U
#define OBSTACLE_NEAR_CONFIRM_SAMPLES 3U
#define OBSTACLE_SIDE_SPEED 360
#define OBSTACLE_FORWARD_SPEED 380
#define OBSTACLE_FORWARD_LEFT_COMP 6
#define OBSTACLE_FORWARD_RIGHT_COMP 0
#define OBSTACLE_IDLE_CHECK_TICKS 5U
#define OBSTACLE_TURN_TICKS 50U
#define OBSTACLE_TRIANGLE_LEG_TICKS 700U
#define OBSTACLE_APEX_TURN_TICKS (OBSTACLE_TURN_TICKS * 2U)

typedef enum {
    OBSTACLE_STATE_IDLE = 0,
    OBSTACLE_STATE_TRIANGLE_TURN_OUT,
    OBSTACLE_STATE_TRIANGLE_FIRST_LEG,
    OBSTACLE_STATE_TRIANGLE_APEX_TURN,
    OBSTACLE_STATE_TRIANGLE_SECOND_LEG,
    OBSTACLE_STATE_TRIANGLE_ALIGN
} ObstacleState_t;

static ObstacleState_t obstacle_state = OBSTACLE_STATE_IDLE;
static uint16_t state_ticks = 0U;
static uint8_t idle_check_ticks = 0U;
static uint8_t trigger_armed = 1U;
static uint32_t last_laser_sequence = 0U;

static uint16_t detector_last_distance = 0U;
static uint8_t detector_has_last = 0U;
static uint8_t detector_valid_samples = 0U;
static uint8_t detector_approach_decreases = 0U;
static uint8_t detector_near_samples = 0U;
static uint8_t detector_saw_far_sample = 0U;

#ifdef OBSTACLE_AVOIDANCE_TEST
static uint8_t obstacle_test_last_buzz_times = 0U;
static int16_t obstacle_test_left_speed = 0;
static int16_t obstacle_test_right_speed = 0;
#endif

static void ObstacleDetector_Reset(void)
{
    detector_last_distance = 0U;
    detector_has_last = 0U;
    detector_valid_samples = 0U;
    detector_approach_decreases = 0U;
    detector_near_samples = 0U;
    detector_saw_far_sample = 0U;
}

static void ObstacleDetector_StartTrack(uint16_t distance_mm)
{
    detector_last_distance = distance_mm;
    detector_has_last = 1U;
    detector_valid_samples = 1U;
    detector_approach_decreases = 0U;
    detector_near_samples = (distance_mm < OBSTACLE_TRIGGER_MM) ? 1U : 0U;
    detector_saw_far_sample = (distance_mm >= OBSTACLE_TRIGGER_MM) ? 1U : 0U;
}

static uint8_t ObstacleDetector_Feed(uint8_t valid, uint16_t distance_mm)
{
    if (!valid ||
        (distance_mm < OBSTACLE_MIN_USABLE_MM) ||
        (distance_mm > OBSTACLE_APPROACH_START_MM)) {
        ObstacleDetector_Reset();
        return 0U;
    }

    if (!detector_has_last) {
        ObstacleDetector_StartTrack(distance_mm);
        return 0U;
    }

    if (distance_mm > (uint16_t)(detector_last_distance + OBSTACLE_APPROACH_TOLERANCE_MM)) {
        ObstacleDetector_StartTrack(distance_mm);
        return 0U;
    }

    if (detector_valid_samples < 255U) {
        detector_valid_samples++;
    }

    if ((uint16_t)(distance_mm + OBSTACLE_APPROACH_TOLERANCE_MM) < detector_last_distance) {
        if (detector_approach_decreases < 255U) {
            detector_approach_decreases++;
        }
    }

    if (distance_mm >= OBSTACLE_TRIGGER_MM) {
        detector_saw_far_sample = 1U;
        detector_near_samples = 0U;
    } else if (detector_near_samples < 255U) {
        detector_near_samples++;
    }

    detector_last_distance = distance_mm;

    if (detector_saw_far_sample &&
        (detector_valid_samples >= OBSTACLE_MIN_APPROACH_SAMPLES) &&
        (detector_approach_decreases >= OBSTACLE_MIN_APPROACH_DECREASES) &&
        (detector_near_samples >= OBSTACLE_NEAR_CONFIRM_SAMPLES)) {
        ObstacleDetector_Reset();
        return 1U;
    }

    return 0U;
}

static void ObstacleAvoidance_SetSpeed(int16_t left_speed, int16_t right_speed)
{
#ifndef OBSTACLE_AVOIDANCE_TEST
    Motor_SetSpeed(&motor_left, left_speed);
    Motor_SetSpeed(&motor_right, right_speed);
#else
    obstacle_test_left_speed = left_speed;
    obstacle_test_right_speed = right_speed;
#endif
}

static void ObstacleAvoidance_SetForwardSpeed(void)
{
    ObstacleAvoidance_SetSpeed(OBSTACLE_FORWARD_SPEED + OBSTACLE_FORWARD_LEFT_COMP,
                               OBSTACLE_FORWARD_SPEED + OBSTACLE_FORWARD_RIGHT_COMP);
}

static void ObstacleAvoidance_StartBuzz(uint8_t times)
{
#ifndef OBSTACLE_AVOIDANCE_TEST
    BuzzStartTimes(times);
#else
    obstacle_test_last_buzz_times = times;
#endif
}

static uint8_t ObstacleAvoidance_ShouldStart(void)
{
    uint16_t distance_mm;
    uint32_t laser_sequence;

    if (!LaserDistance_HasValidDistance()) {
        ObstacleDetector_Reset();
        return 0U;
    }

    laser_sequence = LaserDistance_GetValidSequence();
    if (laser_sequence == last_laser_sequence) {
        return 0U;
    }
    last_laser_sequence = laser_sequence;

    distance_mm = LaserDistance_GetDistanceMm();

    if (!trigger_armed) {
        if (distance_mm > OBSTACLE_CLEAR_MM) {
            trigger_armed = 1U;
            ObstacleDetector_Reset();
        }
        return 0U;
    }

    return ObstacleDetector_Feed(1U, distance_mm);
}

void ObstacleAvoidance_Reset(void)
{
    obstacle_state = OBSTACLE_STATE_IDLE;
    state_ticks = 0U;
    idle_check_ticks = 0U;
    trigger_armed = 1U;
    last_laser_sequence = 0U;
    ObstacleDetector_Reset();
#ifdef OBSTACLE_AVOIDANCE_TEST
    obstacle_test_left_speed = 0;
    obstacle_test_right_speed = 0;
#endif
}

uint8_t ObstacleAvoidance_IsActive(void)
{
    return (obstacle_state != OBSTACLE_STATE_IDLE) ? 1U : 0U;
}

uint8_t ObstacleAvoidance_Update2ms(void)
{
    if (obstacle_state == OBSTACLE_STATE_IDLE) {
        if (idle_check_ticks < (uint8_t)(OBSTACLE_IDLE_CHECK_TICKS - 1U)) {
            idle_check_ticks++;
            return 0U;
        }
        idle_check_ticks = 0U;

        if (!ObstacleAvoidance_ShouldStart()) {
            return 0U;
        }

        obstacle_state = OBSTACLE_STATE_TRIANGLE_TURN_OUT;
        state_ticks = 0U;
        trigger_armed = 0U;
        ObstacleAvoidance_StartBuzz(1U);
    }

    switch (obstacle_state) {
    case OBSTACLE_STATE_TRIANGLE_TURN_OUT:
        /* 1: turn out a small angle from the original line. */
        ObstacleAvoidance_SetSpeed(-OBSTACLE_SIDE_SPEED, OBSTACLE_SIDE_SPEED);
        state_ticks++;
        if (state_ticks >= OBSTACLE_TURN_TICKS) {
            obstacle_state = OBSTACLE_STATE_TRIANGLE_FIRST_LEG;
            state_ticks = 0U;
        }
        return 1U;

    case OBSTACLE_STATE_TRIANGLE_FIRST_LEG:
        /* 2: run the first equal side of the obtuse triangle. */
        ObstacleAvoidance_SetForwardSpeed();
        state_ticks++;
        if (state_ticks >= OBSTACLE_TRIANGLE_LEG_TICKS) {
            obstacle_state = OBSTACLE_STATE_TRIANGLE_APEX_TURN;
            state_ticks = 0U;
        }
        return 1U;

    case OBSTACLE_STATE_TRIANGLE_APEX_TURN:
        /* 3: reverse through the obtuse triangle apex toward the line. */
        ObstacleAvoidance_SetSpeed(OBSTACLE_SIDE_SPEED, -OBSTACLE_SIDE_SPEED);
        state_ticks++;
        if (state_ticks >= OBSTACLE_APEX_TURN_TICKS) {
            obstacle_state = OBSTACLE_STATE_TRIANGLE_SECOND_LEG;
            state_ticks = 0U;
        }
        return 1U;

    case OBSTACLE_STATE_TRIANGLE_SECOND_LEG:
        /* 4: run the second equal side back toward the original line. */
        ObstacleAvoidance_SetForwardSpeed();
        state_ticks++;
        if (state_ticks >= OBSTACLE_TRIANGLE_LEG_TICKS) {
            obstacle_state = OBSTACLE_STATE_TRIANGLE_ALIGN;
            state_ticks = 0U;
        }
        return 1U;

    case OBSTACLE_STATE_TRIANGLE_ALIGN:
        /* 5: restore heading so the normal line follower can take over. */
        ObstacleAvoidance_SetSpeed(-OBSTACLE_SIDE_SPEED, OBSTACLE_SIDE_SPEED);
        state_ticks++;
        if (state_ticks >= OBSTACLE_TURN_TICKS) {
            obstacle_state = OBSTACLE_STATE_IDLE;
            state_ticks = 0U;
            ObstacleAvoidance_StartBuzz(2U);
        }
        return 1U;

    case OBSTACLE_STATE_IDLE:
    default:
        obstacle_state = OBSTACLE_STATE_IDLE;
        state_ticks = 0U;
        return 0U;
    }
}

#ifdef OBSTACLE_AVOIDANCE_TEST
void ObstacleAvoidance_TestResetDetector(void)
{
    ObstacleDetector_Reset();
}

uint8_t ObstacleAvoidance_TestFeedSample(uint8_t valid, uint16_t distance_mm)
{
    return ObstacleDetector_Feed(valid, distance_mm);
}

void ObstacleAvoidance_TestClearBuzz(void)
{
    obstacle_test_last_buzz_times = 0U;
}

uint8_t ObstacleAvoidance_TestGetLastBuzzTimes(void)
{
    return obstacle_test_last_buzz_times;
}

int16_t ObstacleAvoidance_TestGetLeftSpeed(void)
{
    return obstacle_test_left_speed;
}

int16_t ObstacleAvoidance_TestGetRightSpeed(void)
{
    return obstacle_test_right_speed;
}
#endif
