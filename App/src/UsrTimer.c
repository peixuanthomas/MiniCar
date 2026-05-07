/**
  ******************************************************************************
  * @file           : UsrTimer.c
  * @brief          : User timer entry and direct 8-sensor line following.
  ******************************************************************************
  */

#include "UsrTimer.h"

extern volatile uint8_t runFlag;
extern volatile uint8_t oledProductMode;

/*
 * Direct 8-sensor line follower.
 *
 * Sensor order below is left to right: Sen0, Sen1, ... Sen7.
 * GetSenxVal() is expected to be 1 when the sensor sees the black line.
 * If your sensor board reports the opposite, change TRACK_LINE_VALUE to 0.
 */
#define TRACK_LINE_VALUE       1U

/* TIM1 is configured as a fast timer; divide it down to avoid over-steering. */
#define TRACK_CONTROL_DIVIDER  4U

/* PWM values are 0..999. These are intentionally conservative because
 * the sensor bar is close to the vehicle body and steering has a strong effect.
 */
#define TRACK_SPEED_STRAIGHT   360
#define TRACK_SPEED_OUTER      340
#define TRACK_SPEED_MID        250
#define TRACK_SPEED_INNER      150
#define TRACK_SPEED_SEARCH     220

#define TRACK_MASK_ALL         0xFFU
#define TRACK_MASK_CENTER      0x18U

static int8_t lastTurnDir = 0; /* -1: left, 1: right */

static uint8_t Track_IsLine(uint8_t sensorValue)
{
    return (sensorValue == TRACK_LINE_VALUE) ? 1U : 0U;
}

static uint8_t Track_ReadMask(void)
{
    uint8_t mask = 0U;

    if (Track_IsLine(GetSen0Val())) mask |= 0x01U;
    if (Track_IsLine(GetSen1Val())) mask |= 0x02U;
    if (Track_IsLine(GetSen2Val())) mask |= 0x04U;
    if (Track_IsLine(GetSen3Val())) mask |= 0x08U;
    if (Track_IsLine(GetSen4Val())) mask |= 0x10U;
    if (Track_IsLine(GetSen5Val())) mask |= 0x20U;
    if (Track_IsLine(GetSen6Val())) mask |= 0x40U;
    if (Track_IsLine(GetSen7Val())) mask |= 0x80U;

    return mask;
}

static int8_t Track_GetZone(uint8_t mask)
{
    if ((mask & TRACK_MASK_CENTER) == TRACK_MASK_CENTER) {
        return 0;
    }
    if (mask & 0x08U) {
        return -1;
    }
    if (mask & 0x10U) {
        return 1;
    }
    if (mask & 0x04U) {
        return -2;
    }
    if (mask & 0x20U) {
        return 2;
    }
    if (mask & 0x02U) {
        return -3;
    }
    if (mask & 0x40U) {
        return 3;
    }
    if (mask & 0x01U) {
        return -4;
    }
    if (mask & 0x80U) {
        return 4;
    }

    return 0;
}

static void Track_SetMotor(uint16_t leftSpeed, uint16_t rightSpeed)
{
    Motor_SetSpeed(&motor_left, (int16_t)leftSpeed);
    Motor_SetSpeed(&motor_right, (int16_t)rightSpeed);
}

static void Track_Stop(void)
{
    Track_SetMotor(0, 0);
}

static void Track_SearchLostLine(void)
{
    if (lastTurnDir < 0) {
        Track_SetMotor(0, TRACK_SPEED_SEARCH);
    } else if (lastTurnDir > 0) {
        Track_SetMotor(TRACK_SPEED_SEARCH, 0);
    } else {
        Track_SetMotor(TRACK_SPEED_SEARCH, TRACK_SPEED_SEARCH);
    }
}

static void Track_FollowDirect(void)
{
    uint8_t mask = Track_ReadMask();
    int8_t zone;

    if (mask == 0U) {
        Track_SearchLostLine();
        return;
    }

    if (mask == TRACK_MASK_ALL) {
        Track_SetMotor(TRACK_SPEED_MID, TRACK_SPEED_MID);
        return;
    }

    zone = Track_GetZone(mask);

    if (zone < 0) {
        lastTurnDir = -1;
    } else if (zone > 0) {
        lastTurnDir = 1;
    }

    switch (zone) {
    case 0:
        Track_SetMotor(TRACK_SPEED_STRAIGHT, TRACK_SPEED_STRAIGHT);
        break;
    case -1:
        Track_SetMotor(TRACK_SPEED_MID, TRACK_SPEED_OUTER);
        break;
    case 1:
        Track_SetMotor(TRACK_SPEED_OUTER, TRACK_SPEED_MID);
        break;
    case -2:
        Track_SetMotor(TRACK_SPEED_INNER, TRACK_SPEED_OUTER);
        break;
    case 2:
        Track_SetMotor(TRACK_SPEED_OUTER, TRACK_SPEED_INNER);
        break;
    case -3:
    case -4:
        Track_SetMotor(0, TRACK_SPEED_OUTER);
        break;
    case 3:
    case 4:
        Track_SetMotor(TRACK_SPEED_OUTER, 0);
        break;
    default:
        Track_SetMotor(TRACK_SPEED_MID, TRACK_SPEED_MID);
        break;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        static uint8_t controlDivider = 0U;

        if (!oledProductMode) {
            OLED_ShowNum(48, 0, runFlag, 1, OLED_8X16);
        }

        if (runFlag == 0) {
            Track_Stop();
            lastTurnDir = 0;
            controlDivider = 0U;
            return;
        }

        controlDivider++;
        if (controlDivider < TRACK_CONTROL_DIVIDER) {
            return;
        }

        controlDivider = 0U;
        Track_FollowDirect();
    }
}
