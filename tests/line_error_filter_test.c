#include <stdint.h>
#include <stdio.h>

#include "LineErrorFilter.h"

static int expect_float_x10(const char *name, float got, float expected)
{
    int got_x10 = (int)(got * 10.0f);
    int expected_x10 = (int)(expected * 10.0f);

    if (got_x10 != expected_x10) {
        printf("%s: expected %.1f, got %.1f\n", name, expected, got);
        return 1;
    }
    return 0;
}

static int expect_u8(const char *name, uint8_t got, uint8_t expected)
{
    if (got != expected) {
        printf("%s: expected %u, got %u\n", name, expected, got);
        return 1;
    }
    return 0;
}

int main(void)
{
    int failed = 0;
    LineErrorFilter_t filter;
    LineErrorFilter_Result_t result;

    LineErrorFilter_Reset(&filter);

    result = LineErrorFilter_Update(&filter, 0.0f);
    failed |= expect_float_x10("first sample accepted error", result.error, 0.0f);
    failed |= expect_u8("first sample accepted flag", result.accepted, 1U);
    failed |= expect_u8("first sample ignored flag", result.ignored, 0U);

    result = LineErrorFilter_Update(&filter, 6.0f);
    failed |= expect_float_x10("single large jump reuses previous error", result.error, 0.0f);
    failed |= expect_u8("single large jump not accepted", result.accepted, 0U);
    failed |= expect_u8("single large jump ignored", result.ignored, 1U);

    result = LineErrorFilter_Update(&filter, 0.5f);
    failed |= expect_float_x10("normal sample after ignored jump accepted", result.error, 0.5f);
    failed |= expect_u8("normal sample accepted", result.accepted, 1U);
    failed |= expect_u8("normal sample not ignored", result.ignored, 0U);

    result = LineErrorFilter_Update(&filter, 6.0f);
    failed |= expect_float_x10("first repeated jump still ignored", result.error, 0.5f);
    failed |= expect_u8("first repeated jump not accepted", result.accepted, 0U);

    result = LineErrorFilter_Update(&filter, 6.0f);
    failed |= expect_float_x10("second repeated jump accepted", result.error, 6.0f);
    failed |= expect_u8("second repeated jump accepted flag", result.accepted, 1U);
    failed |= expect_u8("second repeated jump not ignored", result.ignored, 0U);

    LineErrorFilter_Reset(&filter);
    result = LineErrorFilter_Update(&filter, -7.0f);
    failed |= expect_float_x10("reset makes next sample accepted", result.error, -7.0f);
    failed |= expect_u8("reset sample accepted", result.accepted, 1U);

    if (failed) {
        return 1;
    }

    printf("line error filter tests passed\n");
    return 0;
}
