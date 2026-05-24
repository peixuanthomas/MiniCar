#include "RunTimer.h"

static uint8_t running = 0U;
static uint32_t start_tick_ms = 0U;
static uint32_t last_elapsed_ms = 0U;

void RunTimer_Init(void)
{
    running = 0U;
    start_tick_ms = 0U;
    last_elapsed_ms = 0U;
}

void RunTimer_SetRunning(uint8_t is_running, uint32_t now_ms)
{
    is_running = is_running ? 1U : 0U;

    if (is_running == running) {
        return;
    }

    if (is_running) {
        start_tick_ms = now_ms;
        last_elapsed_ms = 0U;
    } else {
        last_elapsed_ms = (uint32_t)(now_ms - start_tick_ms);
    }

    running = is_running;
}

uint32_t RunTimer_GetElapsedSeconds(uint32_t now_ms)
{
    uint32_t elapsed_ms = last_elapsed_ms;

    if (running) {
        elapsed_ms = (uint32_t)(now_ms - start_tick_ms);
    }

    return elapsed_ms / 1000U;
}
