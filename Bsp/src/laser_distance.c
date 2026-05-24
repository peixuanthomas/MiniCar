#include "laser_distance.h"

#ifndef LASER_DISTANCE_TEST
#include "uart.h"
#include "stm32f1xx_hal.h"
#endif

#define LASER_DISTANCE_FRAME_MAX_LEN 32U
#define LASER_DISTANCE_QUERY_INTERVAL_MS 200U
#define LASER_DISTANCE_STALE_MS 10000U
#define LASER_DISTANCE_MIN_VALID_MM 40U
#define LASER_DISTANCE_MAX_VALID_MM 2000U

static uint8_t rx_frame[LASER_DISTANCE_FRAME_MAX_LEN];
static uint8_t rx_count = 0U;
static uint8_t expected_len = 0U;

static uint16_t latest_distance_mm = 0U;
static uint8_t latest_valid = 0U;

volatile LaserDistance_Debug_t g_laser_distance_debug;

#ifndef LASER_DISTANCE_TEST
static uint32_t last_query_tick = 0U;
static uint32_t last_valid_tick = 0U;
#endif

static void LaserDistance_DebugSaveFrame(const uint8_t *frame, uint8_t frame_len)
{
    uint8_t i;
    uint8_t copy_len = (frame_len > 16U) ? 16U : frame_len;

    g_laser_distance_debug.last_frame_len = frame_len;
    for (i = 0U; i < copy_len; i++) {
        g_laser_distance_debug.last_frame[i] = frame[i];
    }
    for (; i < 16U; i++) {
        g_laser_distance_debug.last_frame[i] = 0U;
    }
}

static uint8_t LaserDistance_ChecksumValid(const uint8_t *frame, uint8_t frame_len)
{
    uint8_t i;
    uint16_t checksum = 0U;
    uint16_t received_checksum;

    if (frame_len < 4U) {
        return 0U;
    }

    for (i = 0U; i < (uint8_t)(frame_len - 2U); i++) {
        checksum = (uint16_t)(checksum + frame[i]);
    }

    received_checksum = ((uint16_t)frame[frame_len - 2U] << 8U) |
                        frame[frame_len - 1U];

    g_laser_distance_debug.last_checksum_calc = checksum;
    g_laser_distance_debug.last_checksum_recv = received_checksum;

    return (checksum == received_checksum) ? 1U : 0U;
}

static uint8_t LaserDistance_DistanceInRange(uint16_t distance_mm)
{
    return ((distance_mm >= LASER_DISTANCE_MIN_VALID_MM) &&
            (distance_mm <= LASER_DISTANCE_MAX_VALID_MM)) ? 1U : 0U;
}

static uint8_t LaserDistance_ParseFrame(const uint8_t *frame, uint8_t frame_len, uint16_t *distance_mm)
{
    uint16_t parsed_distance;

    g_laser_distance_debug.frame_count++;
    LaserDistance_DebugSaveFrame(frame, frame_len);

    if (frame_len != 12U) {
        g_laser_distance_debug.field_fail_count++;
        return 0U;
    }
    if (!LaserDistance_ChecksumValid(frame, frame_len)) {
        g_laser_distance_debug.checksum_fail_count++;
        return 0U;
    }
    g_laser_distance_debug.last_status = frame[5];
    g_laser_distance_debug.last_func = frame[6];
    g_laser_distance_debug.last_data_len = frame[7];
    if ((frame[0] != 0x55U) ||
        (frame[5] != 0x00U) ||
        (frame[6] != 0x05U) ||
        (frame[7] != 0x02U)) {
        g_laser_distance_debug.field_fail_count++;
        return 0U;
    }

    parsed_distance = ((uint16_t)frame[8] << 8U) | frame[9];
    g_laser_distance_debug.last_distance_mm = parsed_distance;
    if (!LaserDistance_DistanceInRange(parsed_distance)) {
        g_laser_distance_debug.range_fail_count++;
    }

    *distance_mm = parsed_distance;
    return 1U;
}

static uint8_t LaserDistance_ProcessByte(uint8_t byte, uint16_t *distance_mm)
{
    if (rx_count == 0U) {
        if (byte != 0x55U) {
            g_laser_distance_debug.sync_drop_count++;
            return 0U;
        }
        rx_frame[rx_count++] = byte;
        expected_len = 0U;
        return 0U;
    }

    rx_frame[rx_count++] = byte;

    if (rx_count == 2U) {
        expected_len = (uint8_t)(rx_frame[1] + 2U);
        if ((expected_len < 4U) || (expected_len > LASER_DISTANCE_FRAME_MAX_LEN)) {
            rx_count = 0U;
            expected_len = 0U;
        }
        return 0U;
    }

    if ((expected_len > 0U) && (rx_count >= expected_len)) {
        uint8_t parsed = LaserDistance_ParseFrame(rx_frame, expected_len, distance_mm);
        rx_count = 0U;
        expected_len = 0U;
        return parsed;
    }

    if (rx_count >= LASER_DISTANCE_FRAME_MAX_LEN) {
        rx_count = 0U;
        expected_len = 0U;
    }

    return 0U;
}

void LaserDistance_Init(void)
{
    uint8_t i;

    rx_count = 0U;
    expected_len = 0U;
    latest_distance_mm = 0U;
    latest_valid = 0U;
    g_laser_distance_debug.rx_bytes = 0U;
    g_laser_distance_debug.query_count = 0U;
    g_laser_distance_debug.frame_count = 0U;
    g_laser_distance_debug.valid_count = 0U;
    g_laser_distance_debug.checksum_fail_count = 0U;
    g_laser_distance_debug.field_fail_count = 0U;
    g_laser_distance_debug.range_fail_count = 0U;
    g_laser_distance_debug.sync_drop_count = 0U;
    g_laser_distance_debug.last_distance_mm = 0U;
    g_laser_distance_debug.last_checksum_calc = 0U;
    g_laser_distance_debug.last_checksum_recv = 0U;
    g_laser_distance_debug.last_status = 0U;
    g_laser_distance_debug.last_func = 0U;
    g_laser_distance_debug.last_data_len = 0U;
    g_laser_distance_debug.last_frame_len = 0U;
    for (i = 0U; i < 16U; i++) {
        g_laser_distance_debug.last_frame[i] = 0U;
    }

#ifndef LASER_DISTANCE_TEST
    last_query_tick = 0U;
    last_valid_tick = 0U;
#endif
}

#ifndef LASER_DISTANCE_TEST
static void LaserDistance_SendQuery(void)
{
    extern UART_HandleTypeDef huart3;
    static const uint8_t query[] = {
        0x51U, 0x0AU, 0x00U, 0x01U, 0x00U, 0x05U, 0x02U, 0x00U, 0x63U
    };

    (void)HAL_UART_Transmit(&huart3, (uint8_t *)query, sizeof(query), 20U);
    g_laser_distance_debug.query_count++;
}

void LaserDistance_Task(void)
{
    uint8_t byte;
    uint16_t distance_mm;
    uint8_t drain_guard = 64U;
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - last_query_tick) >= LASER_DISTANCE_QUERY_INTERVAL_MS) {
        last_query_tick = now;
        LaserDistance_SendQuery();
    }

    while ((drain_guard > 0U) && (uart_fifo_get(&g_uart3, &byte) == 0)) {
        drain_guard--;
        g_laser_distance_debug.rx_bytes++;
        if (LaserDistance_ProcessByte(byte, &distance_mm)) {
            latest_distance_mm = distance_mm;
            latest_valid = 1U;
            last_valid_tick = now;
            g_laser_distance_debug.valid_count++;
        }
    }
}

uint8_t LaserDistance_IsStale(void)
{
    if (!latest_valid) {
        return 1U;
    }

    return ((uint32_t)(HAL_GetTick() - last_valid_tick) > LASER_DISTANCE_STALE_MS) ? 1U : 0U;
}
#else
void LaserDistance_Task(void)
{
}

uint8_t LaserDistance_IsStale(void)
{
    return latest_valid ? 0U : 1U;
}

void LaserDistance_TestResetParser(void)
{
    LaserDistance_Init();
}

uint8_t LaserDistance_TestParseByte(uint8_t byte, uint16_t *distance_mm)
{
    return LaserDistance_ProcessByte(byte, distance_mm);
}
#endif

uint8_t LaserDistance_HasValidDistance(void)
{
    return (latest_valid && !LaserDistance_IsStale()) ? 1U : 0U;
}

uint16_t LaserDistance_GetDistanceMm(void)
{
    return latest_distance_mm;
}
