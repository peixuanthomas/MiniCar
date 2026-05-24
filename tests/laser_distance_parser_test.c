#include <stdint.h>
#include <stdio.h>

#include "laser_distance.h"

static int feed_bytes(const uint8_t *data, unsigned int len, uint16_t *distance_mm)
{
    unsigned int i;

    for (i = 0U; i < len; i++) {
        if (LaserDistance_TestParseByte(data[i], distance_mm)) {
            return 1;
        }
    }

    return 0;
}

int main(void)
{
    static const uint8_t valid_frame[] = {
        0x55U, 0x0AU, 0x00U, 0x01U, 0x00U, 0x00U,
        0x05U, 0x02U, 0x01U, 0x20U, 0x00U, 0x88U
    };
    static const uint8_t bad_checksum[] = {
        0x55U, 0x0AU, 0x00U, 0x01U, 0x00U, 0x00U,
        0x05U, 0x02U, 0x01U, 0x20U, 0x00U, 0x89U
    };
    static const uint8_t status_text[] = "State:0 , Range Valid\r\n";
    static const uint8_t zero_distance_frame[] = {
        0x55U, 0x0AU, 0x00U, 0x01U, 0x00U, 0x00U,
        0x05U, 0x02U, 0x00U, 0x00U, 0x00U, 0x67U
    };
    static const uint8_t over_range_frame[] = {
        0x55U, 0x0AU, 0x00U, 0x01U, 0x00U, 0x00U,
        0x05U, 0x02U, 0x08U, 0x34U, 0x00U, 0xA3U
    };
    uint16_t distance_mm = 0U;

    LaserDistance_TestResetParser();
    if (!feed_bytes(valid_frame, sizeof(valid_frame), &distance_mm)) {
        printf("valid frame did not parse\n");
        return 1;
    }
    if (distance_mm != 288U) {
        printf("expected 288 mm, got %u\n", distance_mm);
        return 1;
    }

    LaserDistance_TestResetParser();
    distance_mm = 0U;
    if (feed_bytes(bad_checksum, sizeof(bad_checksum), &distance_mm)) {
        printf("bad checksum parsed as %u mm\n", distance_mm);
        return 1;
    }

    LaserDistance_TestResetParser();
    distance_mm = 0U;
    if (feed_bytes(status_text, (unsigned int)(sizeof(status_text) - 1U), &distance_mm)) {
        printf("status text parsed as %u mm\n", distance_mm);
        return 1;
    }

    LaserDistance_TestResetParser();
    distance_mm = 1234U;
    if (feed_bytes(zero_distance_frame, sizeof(zero_distance_frame), &distance_mm)) {
        printf("zero distance parsed as %u mm\n", distance_mm);
        return 1;
    }

    LaserDistance_TestResetParser();
    distance_mm = 1234U;
    if (feed_bytes(over_range_frame, sizeof(over_range_frame), &distance_mm)) {
        printf("over range distance parsed as %u mm\n", distance_mm);
        return 1;
    }

    printf("laser distance parser tests passed\n");
    return 0;
}
