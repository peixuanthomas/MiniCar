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

/** 板级外设初始化函数
  * @brief  被Main函数调用的板级初始化函数
  * @param  None
  * @retval None
  */
void BoardInit(void)
{
	uint32_t i;
	InitMwUart1();
	InitMwUart3();
	OLED_Init();
	//OLED_ShowPicture(0,0, 128, 64, BMP1, 1);
	//OLED_Reverse();
	for(i=0; i<7; i++)	{
		OLED_ShowChinese((8+(i<<4)),0,i,16);
	}
	OLED_ShowString(8,16, "E-Track Traning", OLED_8X16);
	OLED_ShowString(72, 48, "mV", OLED_8X16);
	InitMotor();
	OLED_Update();
}
