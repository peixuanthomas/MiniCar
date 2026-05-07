/**
  ******************************************************************************
  * @file           : adc_bat.c
  * @brief          : 电池电压采样
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
#include "adc_bat.h"
#include "oled_api.h"

uint8_t Adc_Cplt_Flag = 0;
uint32_t adc_value;
static uint16_t adc_milli_volt;
#define ADCRatio 5.228759765625

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        // 获取ADC转换结果
        adc_value = HAL_ADC_GetValue(hadc);
        
        // 设置转换完成标志
        Adc_Cplt_Flag = 1;
	}
}

extern ADC_HandleTypeDef hadc1;
extern volatile uint8_t oledProductMode;

void AdcReadTask(void)
{
	if(Adc_Cplt_Flag)	{
		adc_milli_volt = (uint16_t)(ADCRatio * adc_value);
		if (!oledProductMode) {
			OLED_ShowNum(40, 32, adc_milli_volt, 4, OLED_8X16);
		}
	}
	HAL_ADC_Start_IT(&hadc1);
	Adc_Cplt_Flag = 0;
}

uint32_t Adc_GetRawValue(void)
{
	return adc_value;
}

uint16_t Adc_GetMilliVolt(void)
{
	return adc_milli_volt;
}


