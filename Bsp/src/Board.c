/**
  ******************************************************************************
  * @file           : Board.c
  * @brief          : 板级HAL抽象层文件
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
#include "Board.h"
//#include "bmp.h"

static void BoardStartupBeep(void)
{
	BuzzON();
	HAL_Delay(100);
	BuzzOFF();
}

/** 板级外设初始化函数
  * @brief  被Main函数调用的板级初始化函数
  * @param  None
  * @retval None
  */
void BoardInit(void)
{
	BoardStartupBeep();
	InitMwUart1();
	InitMwUart3();
	LaserDistance_Init();
	OLED_Init();
	OLED_ShowString(0, 0, "dist: ---- mm", OLED_8X16);
	OLED_ShowString(0, 16, "SEN:", OLED_8X16);
	OLED_ShowString(0, 32, "BAT:", OLED_8X16);
	OLED_ShowString(72, 32, "mV", OLED_8X16);
	InitMotor();
	OLED_Update();
}

