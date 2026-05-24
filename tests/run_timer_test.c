#include <stdint.h>
#include <stdio.h>

#include "RunTimer.h"

static int expect_u32(const char *name, uint32_t got, uint32_t expected)
{
    if (got != expected) {
        printf("%s: expected %lu, got %lu\n",
               name,
               (unsigned long)expected,
               (unsigned long)got);
        return 1;
    }
    return 0;
}

int main(void)
{
    int failed = 0;

    RunTimer_Init();
    failed |= expect_u32("initial elapsed", RunTimer_GetElapsedSeconds(0U), 0U);

    RunTimer_SetRunning(1U, 1000U);
    failed |= expect_u32("running rounds down", RunTimer_GetElapsedSeconds(1999U), 0U);
    failed |= expect_u32("running one second", RunTimer_GetElapsedSeconds(2000U), 1U);
    failed |= expect_u32("running three seconds", RunTimer_GetElapsedSeconds(4500U), 3U);

    RunTimer_SetRunning(0U, 4600U);
    failed |= expect_u32("stopped keeps last run", RunTimer_GetElapsedSeconds(9000U), 3U);

    RunTimer_SetRunning(1U, 10000U);
    RunTimer_SetRunning(0U, 12500U);
    failed |= expect_u32("new run replaces old result", RunTimer_GetElapsedSeconds(13000U), 2U);

    RunTimer_Init();
    failed |= expect_u32("reset clears elapsed", RunTimer_GetElapsedSeconds(13000U), 0U);

    if (failed) {
        return 1;
    }

    printf("run timer tests passed\n");
    return 0;
}
