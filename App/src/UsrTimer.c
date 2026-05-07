/**
  ******************************************************************************
  * @file           : UsrTimer.c
  * @brief          : 定时器用户入口
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 上海师范大学 2025-2035
  * All rights reserved.
  *
  * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
  * 上海师范大学 信息与机电工程学院 通信工程专业
  * 开源地址：https://gitee.com/NEagle
  * 修改日期：2025/12/10
  * 版本： V1.0
  * 版权所有，盗版必究
  * V1.0修改说明
  *
  ******************************************************************************
  */

#include "UsrTimer.h"
#include "stdio.h"
#include <stdbool.h>
#include <math.h>

extern volatile uint8_t runFlag;
extern volatile uint8_t oledProductMode;

// 修正传感器映射（根据你之前的PID代码）
// 正确的应该是：Sen0=最左，Sen1=左，Sen2=中，Sen3=右，Sen4=最右
// 但你当前代码的命名是反的，我按正确顺序修正

// 速度参数（根据你之前PID代码的速度范围调整）
#define BASE_SPEED     450
#define MIN_SPEED      250
#define MAX_SPEED      750
#define TURN_SPEED     420
#define SHARP_TURN     480

// 直行微调参数（解决重心不稳问题）
#define STRAIGHT_ADJUST_GAIN   30   // 直行微调增益
#define DEAD_ZONE_THRESHOLD    0.2f // 死区阈值，小于此值不调整
#define MAX_STRAIGHT_ADJUST    80   // 最大直行调整量

// 防偏移记忆
static float history_error[3] = {0, 0, 0}; // 记录最近3次的误差
static int error_index = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)  // 检查是TIM1定时器
    {
        if (oledProductMode) {
            return;
        }

        OLED_ShowNum(48, 0, runFlag, 1, OLED_8X16); // 显示运行状态

        if (runFlag == 0) { // 未启动，保持停止
            Motor_SetSpeed(&motor_left, 0);
            Motor_SetSpeed(&motor_right, 0);
            return;
        }
        
        // 修正传感器命名（根据之前的PID代码）
        // 你之前的PID代码中：SenLL是最左，SenL是左，SenMid是中，SenR是右，SenRR是最右
        // 所以这里应该这样映射：
        bool SenRR = GetSen0Val();  // 最左传感器
        bool SenR  = GetSen1Val();  // 左传感器
        bool SenMid= GetSen2Val();  // 中间传感器
        bool SenL  = GetSen3Val();  // 右传感器
        bool SenLL = GetSen4Val();  // 最右传感器

        char steer[3] = {' ', ' ', '\0'};  // 转向显示
        
        // 计算当前误差（用于直行微调）
        // 传感器值：1=白色（无反射），0=黑色（有反射）
        // 误差计算：负值表示偏左，正值表示偏右
        float current_error = 0.0f;
        int active_count = 0;
        
        if (SenLL == 0) { current_error -= 2.0f; active_count++; } // 最左黑线，车偏右
        if (SenL == 0)  { current_error -= 1.0f; active_count++; } // 左黑线，车稍偏右
        if (SenMid == 0){ current_error += 0.0f; active_count++; } // 中间黑线，居中
        if (SenR == 0)  { current_error += 1.0f; active_count++; } // 右黑线，车稍偏左
        if (SenRR == 0) { current_error += 2.0f; active_count++; } // 最右黑线，车偏左
        
        // 计算平均误差
        if (active_count > 0) {
            current_error = current_error / active_count;
        }
        
        // 存储历史误差
        history_error[error_index] = current_error;
        error_index = (error_index + 1) % 3;
        
        // 计算平均历史误差（用于检测长期偏移趋势）
        float avg_error = 0.0f;
        for (int i = 0; i < 3; i++) {
            avg_error += history_error[i];
        }
        avg_error /= 3.0f;
        
        // 简单阈值巡线算法（增强版，加入直行微调）
        
        // 情况1：理想直行（中间和左右都有黑线）
        if (SenMid == 0 && SenL == 0 && SenR == 0) {
            steer[0] = 'S'; steer[1] = ' ';
            
            // 重心不稳处理：根据误差进行微调
            int left_speed = BASE_SPEED;
            int right_speed = BASE_SPEED;
            
            // 如果误差绝对值超过死区阈值，进行微调
            if (fabsf(current_error) > DEAD_ZONE_THRESHOLD) {
                int adjust = (int)(current_error * STRAIGHT_ADJUST_GAIN);
                
                // 限制调整幅度
                if (adjust > MAX_STRAIGHT_ADJUST) adjust = MAX_STRAIGHT_ADJUST;
                if (adjust < -MAX_STRAIGHT_ADJUST) adjust = -MAX_STRAIGHT_ADJUST;
                
                // 调整左右轮速度（误差为正时车偏左，需要左轮加速/右轮减速）
                left_speed = BASE_SPEED - adjust;  // 注意：current_error正=偏左，所以左轮要减速
                right_speed = BASE_SPEED + adjust; // 右轮要加速
                
                // 显示微调方向
                if (adjust > 0) {
                    steer[0] = 'L'; steer[1] = 'm';  // 微左调
                } else if (adjust < 0) {
                    steer[0] = 'R'; steer[1] = 'm';  // 微右调
                }
            }
            
            // 限幅
            if (left_speed < MIN_SPEED) left_speed = MIN_SPEED;
            if (right_speed < MIN_SPEED) right_speed = MIN_SPEED;
            if (left_speed > MAX_SPEED) left_speed = MAX_SPEED;
            if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
            
            Motor_SetSpeed(&motor_left, left_speed);
            Motor_SetSpeed(&motor_right, right_speed);
        }
        // 情况2：标准直行（左右都有黑线，中间可能没有）
        else if (SenL == 0 && SenR == 0) {
            steer[0] = 'S'; steer[1] = ' ';
            
            // 标准直行，但加入防偏移补偿
            int left_speed = BASE_SPEED;
            int right_speed = BASE_SPEED;
            
            // 使用平均历史误差进行补偿，防止累积偏移
            if (fabsf(avg_error) > DEAD_ZONE_THRESHOLD) {
                int adjust = (int)(avg_error * (STRAIGHT_ADJUST_GAIN / 2));
                
                if (adjust > MAX_STRAIGHT_ADJUST/2) adjust = MAX_STRAIGHT_ADJUST/2;
                if (adjust < -MAX_STRAIGHT_ADJUST/2) adjust = -MAX_STRAIGHT_ADJUST/2;
                
                left_speed -= adjust;
                right_speed += adjust;
                
                if (adjust > 0) {
                    steer[0] = 'L'; steer[1] = 'c';  // 补偿左调
                } else if (adjust < 0) {
                    steer[0] = 'R'; steer[1] = 'c';  // 补偿右调
                }
            }
            
            // 限幅
            if (left_speed < MIN_SPEED) left_speed = MIN_SPEED;
            if (right_speed < MIN_SPEED) right_speed = MIN_SPEED;
            if (left_speed > MAX_SPEED) left_speed = MAX_SPEED;
            if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
            
            Motor_SetSpeed(&motor_left, left_speed);
            Motor_SetSpeed(&motor_right, right_speed);
        }
        // 情况3：左侧偏离（左白右黑），右转
        else if (SenL == 1 && SenR == 0) {
            steer[0] = 'R'; steer[1] = ' ';
            Motor_SetSpeed(&motor_left, TURN_SPEED);
            Motor_SetSpeed(&motor_right, 0);
        }
        // 情况4：右侧偏离（左黑右白），左转
        else if (SenL == 0 && SenR == 1) {
            steer[0] = 'L'; steer[1] = ' ';
            Motor_SetSpeed(&motor_left, 0);
            Motor_SetSpeed(&motor_right, TURN_SPEED);
        }
        // 情况5：只有中间传感器检测到黑线，大左转
        else if (SenMid == 0 && SenL == 1 && SenR == 1) {
            steer[0] = 'L'; steer[1] = 'L';
            Motor_SetSpeed(&motor_left, 0);
            Motor_SetSpeed(&motor_right, SHARP_TURN);
        }
        // 情况6：只有最右传感器检测到黑线，大右转
        else if (SenRR == 0 && SenL == 1 && SenMid == 1 && SenR == 1) {
            steer[0] = 'R'; steer[1] = 'R';
            Motor_SetSpeed(&motor_left, SHARP_TURN);
            Motor_SetSpeed(&motor_right, 0);
        }
        // 情况7：只有最左传感器检测到黑线，大左转
        else if (SenLL == 0 && SenL == 1 && SenMid == 1 && SenR == 1) {
            steer[0] = 'L'; steer[1] = 'L';
            Motor_SetSpeed(&motor_left, 0);
            Motor_SetSpeed(&motor_right, SHARP_TURN);
        }
        // 情况8：所有传感器都是白色（冲出轨道）
        else if (SenLL == 1 && SenL == 1 && SenMid == 1 && SenR == 1 && SenRR == 1) {
            steer[0] = 'E'; steer[1] = ' ';
            
            // 基于历史误差决定恢复方向
            if (avg_error < 0) { // 历史偏右，向左转寻找黑线
                Motor_SetSpeed(&motor_left, -200); // 左轮反转
                Motor_SetSpeed(&motor_right, 300); // 右轮正转
            } else { // 历史偏左，向右转寻找黑线
                Motor_SetSpeed(&motor_left, 300);  // 左轮正转
                Motor_SetSpeed(&motor_right, -200); // 右轮反转
            }
        }
        // 其他情况：保守处理，使用误差进行微调
        else {
            steer[0] = 'C'; steer[1] = ' ';
            
            // 保守速度
            int left_speed = BASE_SPEED - 50;
            int right_speed = BASE_SPEED - 50;
            
            // 根据当前误差微调
            if (fabsf(current_error) > DEAD_ZONE_THRESHOLD) {
                int adjust = (int)(current_error * 20); // 较小增益
                left_speed -= adjust;
                right_speed += adjust;
            }
            
            // 限幅
            if (left_speed < MIN_SPEED) left_speed = MIN_SPEED;
            if (right_speed < MIN_SPEED) right_speed = MIN_SPEED;
            if (left_speed > MAX_SPEED) left_speed = MAX_SPEED;
            if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
            
            Motor_SetSpeed(&motor_left, left_speed);
            Motor_SetSpeed(&motor_right, right_speed);
        }
        
        (void)steer;
    }
}


