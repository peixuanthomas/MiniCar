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

static uint8_t update_with_distance(uint16_t distance_mm)
{
    laser_valid = 1U;
    laser_distance_mm = distance_mm;
    laser_sequence++;
    return ObstacleAvoidance_Update2ms();
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

static int expect_buzz_times(const char *name, uint8_t expected_times)
{
    uint8_t actual_times = ObstacleAvoidance_TestGetLastBuzzTimes();
    if (actual_times != expected_times) {
        printf("%s: expected %u buzz times, got %u\n", name, expected_times, actual_times);
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
    failed |= expect_no_trigger("approach 640", 1U, 640U);
    failed |= expect_no_trigger("approach 620", 1U, 620U);
    failed |= expect_no_trigger("first near under 600", 1U, 585U);
    failed |= expect_no_trigger("second near under 600", 1U, 550U);
    failed |= expect_trigger("confirmed near under 600", 1U, 540U);

    ObstacleAvoidance_TestResetDetector();
    failed |= expect_no_trigger("near sample without approach", 1U, 560U);
    failed |= expect_no_trigger("stale resets approach", 0U, 0U);
    failed |= expect_no_trigger("near after stale", 1U, 120U);
    failed |= expect_no_trigger("near after stale again", 1U, 115U);

    ObstacleAvoidance_Reset();
    ObstacleAvoidance_TestClearBuzz();
    (void)update_with_distance(640U);
    (void)update_with_distance(620U);
    (void)update_with_distance(585U);
    (void)update_with_distance(550U);
    failed |= update_with_distance(540U) ? 0 : 1;
    failed |= expect_buzz_times("enter obstacle mode", 1U);

    ObstacleAvoidance_TestClearBuzz();
    for (uint16_t i = 0U; i < 1000U; i++) {
        (void)ObstacleAvoidance_Update2ms();
    }
    failed |= expect_buzz_times("exit obstacle mode", 2U);

    if (failed) {
        return 1;
    }

    printf("obstacle trigger tests passed\n");
    return 0;
}
