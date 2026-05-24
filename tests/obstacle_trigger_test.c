#include <stdint.h>
#include <stdio.h>

#include "ObstacleAvoidance.h"

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

int main(void)
{
    int failed = 0;

    ObstacleAvoidance_TestResetDetector();
    failed |= expect_no_trigger("invalid sample", 0U, 0U);
    failed |= expect_no_trigger("single near sample after invalid", 1U, 100U);
    failed |= expect_no_trigger("second near sample without far approach", 1U, 95U);

    ObstacleAvoidance_TestResetDetector();
    failed |= expect_no_trigger("approach 320", 1U, 320U);
    failed |= expect_no_trigger("approach 260", 1U, 260U);
    failed |= expect_no_trigger("approach 210", 1U, 210U);
    failed |= expect_no_trigger("approach 170", 1U, 170U);
    failed |= expect_no_trigger("first near under 150", 1U, 145U);
    failed |= expect_trigger("confirmed near under 150", 1U, 138U);

    ObstacleAvoidance_TestResetDetector();
    failed |= expect_no_trigger("far sample", 1U, 260U);
    failed |= expect_no_trigger("stale resets approach", 0U, 0U);
    failed |= expect_no_trigger("near after stale", 1U, 120U);
    failed |= expect_no_trigger("near after stale again", 1U, 115U);

    if (failed) {
        return 1;
    }

    printf("obstacle trigger tests passed\n");
    return 0;
}
