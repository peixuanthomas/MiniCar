#ifndef _OBSTACLE_AVOIDANCE_H_
#define _OBSTACLE_AVOIDANCE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void ObstacleAvoidance_Reset(void);
uint8_t ObstacleAvoidance_Update2ms(void);

#ifdef OBSTACLE_AVOIDANCE_TEST
void ObstacleAvoidance_TestResetDetector(void);
uint8_t ObstacleAvoidance_TestFeedSample(uint8_t valid, uint16_t distance_mm);
void ObstacleAvoidance_TestClearBuzz(void);
uint8_t ObstacleAvoidance_TestGetLastBuzzTimes(void);
#endif

#ifdef __cplusplus
}
#endif

#endif
