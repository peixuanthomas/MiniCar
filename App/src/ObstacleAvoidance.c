#include "ObstacleAvoidance.h"

#include "Board.h"

/*
 * 避障参数说明：
 * - OBSTACLE_TRIGGER_MM：触发避障的距离阈值，单位 mm。当前 150 mm，
 *   表示激光测距小于 15 cm 时开始绕障。
 * - OBSTACLE_CLEAR_MM：避障触发后的重新允许触发距离，单位 mm。当前 220 mm。
 *   只有距离重新大于该值后，下一次障碍物检测才会再次触发，避免在同一个障碍物前反复进入避障。
 * - OBSTACLE_MIN_USABLE_MM：测距模块的最小可信距离，单位 mm。当前 40 mm。
 *   小于该值通常是无效值或近距离异常值，不参与避障判断。
 * - OBSTACLE_SIDE_SPEED：原地转向速度。数值越大，转向越快；过大可能打滑或转过头。
 * - OBSTACLE_FORWARD_SPEED：绕过障碍物侧面时的前进速度。数值越大，绕行越快；过大可能来不及回正。
 * - OBSTACLE_TURN_TICKS：第一段转出时间，单位为 2 ms tick。当前 250 约等于 0.5 s。
 *   增大则转出角度更大，减小则转出角度更小。
 * - OBSTACLE_PASS_TICKS：侧向通过障碍物的前进时间，单位为 2 ms tick。当前 450 约等于 0.9 s。
 *   增大可绕过更长的长方体障碍物，减小可更早回到循迹线。
 * - OBSTACLE_RETURN_TICKS：绕过后回正时间，单位为 2 ms tick。当前 250 约等于 0.5 s。
 *   一般应接近 OBSTACLE_TURN_TICKS，保证车身方向大致恢复。
 *
 * 调试建议：
 * 如果小车还没到障碍物就绕行，减小测距误触发可将 OBSTACLE_TRIGGER_MM 调小，
 * 或增大 OBSTACLE_MIN_USABLE_MM 过滤异常近距离值。如果小车撞到障碍物才绕行，
 * 增大 OBSTACLE_TRIGGER_MM。如果绕不过长方体，优先增大 OBSTACLE_PASS_TICKS；
 * 如果转出不够，增大 OBSTACLE_TURN_TICKS 或 OBSTACLE_SIDE_SPEED；
 * 如果绕完后方向偏差明显，微调 OBSTACLE_RETURN_TICKS。
 */
#define OBSTACLE_TRIGGER_MM 150U
#define OBSTACLE_CLEAR_MM 220U
#define OBSTACLE_MIN_USABLE_MM 40U
#define OBSTACLE_SIDE_SPEED 360
#define OBSTACLE_FORWARD_SPEED 380
#define OBSTACLE_TURN_TICKS 250U
#define OBSTACLE_PASS_TICKS 450U
#define OBSTACLE_RETURN_TICKS 250U

typedef enum {
    OBSTACLE_STATE_IDLE = 0,
    OBSTACLE_STATE_TURN_OUT,
    OBSTACLE_STATE_PASS_SIDE,
    OBSTACLE_STATE_TURN_BACK
} ObstacleState_t;

static ObstacleState_t obstacle_state = OBSTACLE_STATE_IDLE;
static uint16_t state_ticks = 0U;
static uint8_t trigger_armed = 1U;

static void ObstacleAvoidance_SetSpeed(int16_t left_speed, int16_t right_speed)
{
    Motor_SetSpeed(&motor_left, left_speed);
    Motor_SetSpeed(&motor_right, right_speed);
}

static uint8_t ObstacleAvoidance_ShouldStart(void)
{
    uint16_t distance_mm;

    if (!LaserDistance_HasValidDistance()) {
        return 0U;
    }

    distance_mm = LaserDistance_GetDistanceMm();
    if (distance_mm < OBSTACLE_MIN_USABLE_MM) {
        return 0U;
    }

    if (!trigger_armed) {
        if (distance_mm > OBSTACLE_CLEAR_MM) {
            trigger_armed = 1U;
        }
        return 0U;
    }

    return (distance_mm < OBSTACLE_TRIGGER_MM) ? 1U : 0U;
}

void ObstacleAvoidance_Reset(void)
{
    obstacle_state = OBSTACLE_STATE_IDLE;
    state_ticks = 0U;
    trigger_armed = 1U;
}

uint8_t ObstacleAvoidance_Update2ms(void)
{
    if (obstacle_state == OBSTACLE_STATE_IDLE) {
        if (!ObstacleAvoidance_ShouldStart()) {
            return 0U;
        }

        obstacle_state = OBSTACLE_STATE_TURN_OUT;
        state_ticks = 0U;
        trigger_armed = 0U;
    }

    switch (obstacle_state) {
    case OBSTACLE_STATE_TURN_OUT:
        /* 第一段：向右原地转出，离开原循迹线，准备从障碍物侧面绕过。 */
        ObstacleAvoidance_SetSpeed(OBSTACLE_SIDE_SPEED, -OBSTACLE_SIDE_SPEED);
        state_ticks++;
        if (state_ticks >= OBSTACLE_TURN_TICKS) {
            obstacle_state = OBSTACLE_STATE_PASS_SIDE;
            state_ticks = 0U;
        }
        return 1U;

    case OBSTACLE_STATE_PASS_SIDE:
        /* 第二段：沿障碍物侧面向前走，时间决定能绕过多长的长方体。 */
        ObstacleAvoidance_SetSpeed(OBSTACLE_FORWARD_SPEED, OBSTACLE_FORWARD_SPEED);
        state_ticks++;
        if (state_ticks >= OBSTACLE_PASS_TICKS) {
            obstacle_state = OBSTACLE_STATE_TURN_BACK;
            state_ticks = 0U;
        }
        return 1U;

    case OBSTACLE_STATE_TURN_BACK:
        /* 第三段：反向原地转回，释放控制权后交还原来的循迹算法。 */
        ObstacleAvoidance_SetSpeed(-OBSTACLE_SIDE_SPEED, OBSTACLE_SIDE_SPEED);
        state_ticks++;
        if (state_ticks >= OBSTACLE_RETURN_TICKS) {
            obstacle_state = OBSTACLE_STATE_IDLE;
            state_ticks = 0U;
        }
        return 1U;

    case OBSTACLE_STATE_IDLE:
    default:
        obstacle_state = OBSTACLE_STATE_IDLE;
        state_ticks = 0U;
        return 0U;
    }
}
