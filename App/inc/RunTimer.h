#ifndef _RUNTIMER_H_
#define _RUNTIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void RunTimer_Init(void);
void RunTimer_SetRunning(uint8_t is_running, uint32_t now_ms);
uint32_t RunTimer_GetElapsedSeconds(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
