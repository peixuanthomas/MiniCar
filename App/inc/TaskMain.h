#ifndef _TASKMAIN_H_
#define _TASKMAIN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

//任务结构体定义
typedef struct _TASK_COMPONENTS
{
	unsigned char Run;	//运行标志，0-没有运行，1-运行
	unsigned short Timer;	//当前剩余时间片值
	unsigned short ItvTime;	//任务间隔时间片
	void (*TaskHook)(void);	//任务函数
} TASK_COMPONENTS;

#define TASK_MAX 11	//任务数量

//任务列表
extern TASK_COMPONENTS TaskComps[];

#ifdef __cplusplus
}
#endif

#endif
