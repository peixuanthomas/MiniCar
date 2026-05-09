/**
  ******************************************************************************
  * @file           : TaskMain.c
  * @brief          : 任务列表及主要任务函数
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 上海师范大学 2025-2035
  * All rights reserved.
  *
  * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
  * 上海师范大学 信息与机电工程学院 通信工程专业
  * 开源地址：https://gitee.com/NEagle
  * 修改日期：2025/12/06
  * 版本： V1.0
  * 版权所有，盗版必究
  * V1.0修改说明
  *
  ******************************************************************************
  */
#include "main.h"
#include "TaskMain.h"
#include "Board.h"
#include "UsrTimer.h"

volatile uint8_t runFlag = 0;   // 0 停止，1 运行
volatile uint8_t oledProductMode = 0;
volatile uint8_t key1BuzzSteps = 0;

void Led1Func(void);
void Led2Func(void);
void BuzzFunc(void);
void Uart1Func(void);
void Uart3Func(void);
void PntFunc(void);
void ChkKey0Func(void);
void ChkKey1Func(void);
void ChkSenFunc(void);

#define SENSOR_DISPLAY_COUNT 8
#define SENSOR_DISPLAY_STR_LEN (SENSOR_DISPLAY_COUNT + 1)

static void ReadSensorDisplayString(uint8_t sensorVal[SENSOR_DISPLAY_STR_LEN])
{
	sensorVal[0] = GetSen0Val()? '1':'0';
	sensorVal[1] = GetSen1Val()? '1':'0';
	sensorVal[2] = GetSen2Val()? '1':'0';
	sensorVal[3] = GetSen3Val()? '1':'0';
	sensorVal[4] = GetSen4Val()? '1':'0';
	sensorVal[5] = GetSen5Val()? '1':'0';
	sensorVal[6] = GetSen6Val()? '1':'0';
	sensorVal[7] = GetSen7Val()? '1':'0';
	sensorVal[SENSOR_DISPLAY_COUNT] = 0;
}

static void OledShowStatusPage(void)
{
	uint8_t SensorVal[SENSOR_DISPLAY_STR_LEN];

	ReadSensorDisplayString(SensorVal);

	OLED_ShowString(0, 0, "FLAG:", OLED_8X16);
	OLED_ShowString(0, 16, "SEN:", OLED_8X16);
	OLED_ShowString(0, 32, "BAT:", OLED_8X16);
	OLED_ShowString(72, 32, "mV", OLED_8X16);
	OLED_ShowNum(48, 0, runFlag, 1, OLED_8X16);
	OLED_ShowString(40, 16, (char *)SensorVal, OLED_8X16);
	OLED_ShowNum(40, 32, Adc_GetMilliVolt(), 4, OLED_8X16);
}

static void StartBuzzTwice(void)
{
	key1BuzzSteps = 4;
}

TASK_COMPONENTS TaskComps[TASK_MAX] = {
	{0, 1, 4, ChkKey0Func},			// 检测Key0的任务，预期每5ms执行一次，在第1ms执行
	{0, 1, 4, ChkKey1Func},			// 检测Key1的任务，预期每5ms执行一次，在第2ms执行
	{0, 5, 10, OLED_Update_InPages},	//OLED屏的刷新任务
	{0, 3, 100, BuzzFunc},
	{0, 4, 100, Uart1Func},
	{0, 4, 100, Uart3Func},
	{0, 4, 100, ChkSenFunc},
	{0, 4, 500, AdcReadTask},
	{0, 3, 1000, Led1Func},
	{0, 3, 2000, Led2Func},
	{0, 3, 1000, PntFunc},
};

/** Key0检测函数
  * @brief  检测Key0是否有按下超过20ms
  *         预期每5ms执行一次，在第1ms执行
  * @param  None
  * @retval None
  */
void ChkKey0Func(void)
{
    static uint8_t Cont = 0;

    if (GetKey0Val()) {
        Cont++;
    } else {
        if (Cont > 4) { // >20ms
            runFlag ^= 1;                // 翻转运行状态
            printf("RunFlag = %d\r\n", runFlag);
            if (runFlag == 0) {          // 切到停止时停轮
                Motor_SetSpeed(&motor_left, 0);
                Motor_SetSpeed(&motor_right, 0);
            } 
			// else {                      // 切到启动时,立即给初始速度
            //     Motor_SetSpeed(&motor_left, 500);
            //     Motor_SetSpeed(&motor_right, 500);
            // }
        }
        Cont = 0;
    }
    // 可选:OLED 显示状态
    //OLED_ShowNum(88, 32, runFlag, 1, OLED_8X16);
}

/** Key1检测函数
  * @brief  检测Key1是否有按下超过20ms
  *         预期每5ms执行一次，在第1ms执行
  * @param  None
  * @retval None
  */
void ChkKey1Func(void)
{
    static uint8_t Cont = 0;

    if (GetKey1Val()) {
        if (Cont < 5) {
            Cont++;
        }
        if (Cont == 5) {
            oledProductMode ^= 1;
            OLED_Clear();
            if (oledProductMode) {
                OLED_ShowString(0, 0, "Product of", OLED_8X16);
                OLED_ShowString(0, 16, "Team Seven", OLED_8X16);
                //OLED_ShowString(0, 32, "Seven", OLED_8X16);
            } else {
                OledShowStatusPage();
            }
            OLED_Update();
            StartBuzzTwice();
            Cont++;
        }
    } else {
        Cont = 0;
    }
}

/** 核心板载LED控制函数
  * @brief  控制核心板上的LED的任务函数
  *         预期每1000ms执行一次翻转，在第3ms执行
  * @param  None
  * @retval None
  */
void Led1Func(void)
{
	CoreLedToggle();
}

/** 底板板载LED控制函数
  * @brief  控制底板上的LED的任务函数
  *         预期每2000ms执行一次翻转，在第4ms执行
  * @param  None
  * @retval None
  */
void Led2Func(void)
{
	BrdLedToggle();
}

/** 蜂鸣器控制函数
  * @brief  控制底板上的蜂鸣器的任务函数
  *         预期每100ms执行一次，在第5ms执行
  *			函数内部开启计数，计到第3次，开启蜂鸣器；第4次，关闭蜂鸣器；第40次，从头开始
  * @param  None
  * @retval None
  */
void BuzzFunc(void)
{
	if (key1BuzzSteps == 0) {
		BuzzOFF();
		return;
	}

	if ((key1BuzzSteps & 0x01) == 0) {
		BuzzON();
	} else {
		BuzzOFF();
	}
	key1BuzzSteps--;
}

/** UART1检查函数
  * @brief  UART1接收缓冲区数据检查，然后将收到的数据直接发回原有UART
  *         预期每100ms执行一次，第6ms执行
  * @param  None
  * @retval None
  */
void Uart1Func(void)
{
	int len;
	uint8_t buf[64];
	len = uart_fifo_gets(&g_uart1, buf, 64);
	if(len>0)
		uart_fifo_puts(&g_uart1, buf, len);
}

/** UART3检查函数
  * @brief  UART3接收缓冲区数据检查，然后将收到的数据直接发回原有UART
  *         预期每100ms执行一次，第7ms执行
  * @param  None
  * @retval None
  */
void Uart3Func(void)
{
	int len;
	uint8_t buf[64];
	len = uart_fifo_gets(&g_uart3, buf, 64);
	if(len>0)
		uart_fifo_puts(&g_uart3, buf, len);
}

/** 定期打印函数
  * @brief  定期打印测试
  *         预期每1000ms执行一次，第8ms执行
  * @param  None
  * @retval None
  */
void PntFunc(void)
{
	printf("run=%u,sen=%u%u%u%u%u%u%u%u,batt=%u\r\n",
	       runFlag,
	       GetSen0Val(),
	       GetSen1Val(),
	       GetSen2Val(),
	       GetSen3Val(),
	       GetSen4Val(),
	       GetSen5Val(),
	       GetSen6Val(),
	       GetSen7Val(),
	       Adc_GetMilliVolt());
}

void ChkSenFunc(void)
{
	if (oledProductMode) {
		return;
	}

	uint8_t SensorVal[SENSOR_DISPLAY_STR_LEN];
	ReadSensorDisplayString(SensorVal);
	OLED_ShowNum(48, 0, runFlag, 1, OLED_8X16);
	OLED_ShowString(40, 16, (char *)SensorVal, OLED_8X16);
	OLED_ShowString(0, 48, "e=", OLED_8X16);
	OLED_ShowSignedNum(16, 48, g_line_error_x10, 3, OLED_8X16);
	OLED_ShowString(48, 48, "c=", OLED_8X16);
	OLED_ShowSignedNum(64, 48, g_line_correction, 4, OLED_8X16);
	// 这里可以增加定期检测传感器到蓝牙传输，使用printf()函数即可，这里如果打印，则定时器那里仅需打印决策动作
	// 但这里的是100ms执行一次
}




