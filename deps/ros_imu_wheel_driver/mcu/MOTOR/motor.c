#include "motor.h" 
#include "bsp.h"

#include "config.h"

#include "stdio.h"
#include <math.h>
#include <stdbool.h>

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"

static bool isInit;

//定义驱动轮结构体
WheelSruct_t g_tLeftWheel = {0}; 
WheelSruct_t g_tRightWheel = {0};  

void motorTask(void* param);

void motorInit(void)
{
	if(isInit) return;

	isInit = true;
}
bool motorTest(void)
{
	bool pass = true;
	
	return pass;
}



