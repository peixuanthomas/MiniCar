#ifndef _LASER_DISTANCE_H_
#define _LASER_DISTANCE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    uint32_t rx_bytes;
    uint32_t query_count;
    uint32_t frame_count;
    uint32_t valid_count;
    uint32_t checksum_fail_count;
    uint32_t field_fail_count;
    uint32_t range_fail_count;
    uint32_t sync_drop_count;
    uint16_t last_distance_mm;
    uint16_t last_checksum_calc;
    uint16_t last_checksum_recv;
    uint8_t last_status;
    uint8_t last_func;
    uint8_t last_data_len;
    uint8_t last_frame_len;
    uint8_t last_frame[16];
} LaserDistance_Debug_t;

extern volatile LaserDistance_Debug_t g_laser_distance_debug;

void LaserDistance_Init(void);
void LaserDistance_Task(void);
uint8_t LaserDistance_HasValidDistance(void);
uint16_t LaserDistance_GetDistanceMm(void);
uint8_t LaserDistance_IsStale(void);

#ifdef LASER_DISTANCE_TEST
void LaserDistance_TestResetParser(void);
uint8_t LaserDistance_TestParseByte(uint8_t byte, uint16_t *distance_mm);
#endif

#ifdef __cplusplus
}
#endif

#endif
