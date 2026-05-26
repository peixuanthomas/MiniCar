#include <stdint.h>
#include <stdio.h>

#include "ObstacleAvoidance.h"

static uint8_t laser_valid = 0U;
static uint16_t laser_distance_mm = 0U;
static uint32_t laser_sequence = 0U;

uint8_t LaserDistance_HasValidDistance(void)
{
    return laser_valid;
}

uint16_t LaserDistance_GetDistanceMm(void)
{
    return laser_distance_mm;
}

uint32_t LaserDistance_GetValidSequence(void)
{
    return laser_sequence;
}

static void set_distance(uint16_t distance_mm)
{
    laser_valid = 1U;
    laser_distance_mm = distance_mm;
    laser_sequence++;
}

static int expect_no_trigger(const char *name, uint8_t valid, uint16_t distance_mm)
{
    if (ObstacleAvoidance_TestFeedSample(valid, distance_mm)) {
        printf("%s: unexpectedly triggered\n", name);
        return 1;
    }
    return 0;
}

static int expect_trigger(const char *name, uint8_t valid, uint16_t distance_mm)
{
    if (!ObstacleAvoidance_TestFeedSample(valid, distance_mm)) {
        printf("%s: did not trigger\n", name);
        return 1;
    }
    return 0;
}

static int expect_motor_speed(const char *name, int16_t expected_left, int16_t expected_right)
{
    int16_t actual_left = ObstacleAvoidance_TestGetLeftSpeed();
    int16_t actual_right = ObstacleAvoidance_TestGetRightSpeed();

    if ((actual_left != expected_left) || (actual_right != expected_right)) {
        printf("%s: expected L=%d R=%d, got L=%d R=%d\n",
               name,
               expected_left,
               expected_right,
               actual_left,
               actual_right);
        return 1;
    }
    return 0;
}

static int expect_active_tick(const char *name, int16_t expected_left, int16_t expected_right)
{
    if (!ObstacleAvoidance_Update2ms()) {
        printf("%s: obstacle mode ended early\n", name);
        return 1;
    }

    return expect_motor_speed(name, expected_left, expected_right);
}

static int expect_active_ticks(const char *name, uint16_t ticks, int16_t expected_left, int16_t expected_right)
{
    uint16_t i;
    int failed = 0;

    for (i = 0U; i < ticks; i++) {
        failed |= expect_active_tick(name, expected_left, expected_right);
    }

    return failed;
}

static int expect_inactive_ticks(const char *name, uint16_t ticks)
{
    uint16_t i;
    int failed = 0;

    for (i = 0U; i < ticks; i++) {
        if (ObstacleAvoidance_Update2ms()) {
            printf("%s: obstacle mode stayed active at tick %u\n", name, (unsigned)i);
            failed = 1;
        }
    }

    return failed;
}

static int expect_buzz_times(const char *name, uint8_t expected_times)
{
    uint8_t actual_times = ObstacleAvoidance_TestGetLastBuzzTimes();
    if (actual_times != expected_times) {
        printf("%s: expected %u buzz times, got %u\n", name, expected_times, actual_times);
        return 1;
    }
    return 0;
}

static int expect_enabled(const char *name, uint8_t expected_enabled)
{
    uint8_t actual_enabled = ObstacleAvoidance_IsEnabled();
    if (actual_enabled != expected_enabled) {
        printf("%s: expected enabled=%u, got %u\n", name, expected_enabled, actual_enabled);
        return 1;
    }
    return 0;
}

int main(void)
{
    int failed = 0;

    ObstacleAvoidance_TestResetDetector();
    failed |= expect_no_trigger("invalid sample", 0U, 0U);
    failed |= expect_no_trigger("single near sample after invalid", 1U, 100U);
    failed |= expect_no_trigger("second near sample without far approach", 1U, 95U);

    ObstacleAvoidance_TestResetDetector();
    failed |= expect_no_trigger("approach 260", 1U, 260U);
    failed |= expect_no_trigger("approach 240", 1U, 240U);
    failed |= expect_no_trigger("approach 220", 1U, 220U);
    failed |= expect_no_trigger("still above 200", 1U, 201U);

    ObstacleAvoidance_TestResetDetector();
    failed |= expect_no_trigger("approach to 260", 1U, 260U);
    failed |= expect_no_trigger("approach to 240", 1U, 240U);
    failed |= expect_no_trigger("approach to 220", 1U, 220U);
    failed |= expect_trigger("confirmed at 200", 1U, 200U);

    ObstacleAvoidance_TestResetDetector();
    failed |= expect_no_trigger("near sample without approach", 1U, 200U);
    failed |= expect_no_trigger("stale resets approach", 0U, 0U);
    failed |= expect_no_trigger("near after stale", 1U, 120U);
    failed |= expect_no_trigger("near after stale again", 1U, 115U);

    ObstacleAvoidance_Reset();
    ObstacleAvoidance_SetEnabled(0U);
    failed |= expect_enabled("disabled switch is readable", 0U);
    ObstacleAvoidance_TestClearBuzz();
    (void)ObstacleAvoidance_TestFeedSample(1U, 260U);
    (void)ObstacleAvoidance_TestFeedSample(1U, 240U);
    (void)ObstacleAvoidance_TestFeedSample(1U, 220U);
    set_distance(200U);
    failed |= expect_inactive_ticks("disabled obstacle avoidance ignores trigger", 5U);
    failed |= expect_buzz_times("disabled obstacle avoidance stays silent", 0U);

    ObstacleAvoidance_Reset();
    failed |= expect_enabled("reset preserves disabled switch", 0U);
    ObstacleAvoidance_SetEnabled(1U);
    failed |= expect_enabled("enabled switch is readable", 1U);

    ObstacleAvoidance_Reset();
    ObstacleAvoidance_TestClearBuzz();
    (void)ObstacleAvoidance_TestFeedSample(1U, 260U);
    (void)ObstacleAvoidance_TestFeedSample(1U, 240U);
    (void)ObstacleAvoidance_TestFeedSample(1U, 220U);
    set_distance(200U);
    failed |= expect_inactive_ticks("laser waits for lower-priority idle slot", 4U);
    failed |= ObstacleAvoidance_Update2ms() ? 0 : 1;
    failed |= expect_buzz_times("enter obstacle mode", 1U);
    failed |= expect_motor_speed("triangle step 1 starts turning out", -360, 360);

    ObstacleAvoidance_TestClearBuzz();
    failed |= expect_active_ticks("triangle step 1 small turn out", 49U, -360, 360);
    failed |= expect_active_ticks("triangle step 2 first equal leg", 700U, 386, 380);
    failed |= expect_active_ticks("triangle step 3 obtuse apex turn", 100U, 360, -360);
    failed |= expect_active_ticks("triangle step 4 second equal leg", 700U, 386, 380);
    failed |= expect_active_ticks("triangle step 5 align with line", 50U, -360, 360);
    failed |= expect_buzz_times("exit obstacle mode", 2U);
    if (ObstacleAvoidance_Update2ms()) {
        printf("triangle sequence: obstacle mode stayed active after final tick\n");
        failed = 1;
    }

    if (failed) {
        return 1;
    }

    printf("obstacle trigger tests passed\n");
    return 0;
}
