#ifndef _OBSTACLE_AVOIDANCE_H_
#define _OBSTACLE_AVOIDANCE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void ObstacleAvoidance_Reset(void);
uint8_t ObstacleAvoidance_Update2ms(void);

#ifdef __cplusplus
}
#endif

#endif
