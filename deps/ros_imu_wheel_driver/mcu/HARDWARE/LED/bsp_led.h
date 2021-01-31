#ifndef __BSP_LED_H
#define __BSP_LED_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//声明： 版权所有，如用于商业目的请与开发者取得联系
//作者： 芯欣电子科技
//版本：  V1.0
//时间：  2016/9/2    
//Copyright(C) 
//All rights reserved		
////////////////////////////////////////////////////////////////////////////////// 

#define LED1    PBout(3)
#define LED2    PBout(10)
#define LED3    PBout(11)

/*TIM2 频率为10KHz*/
#define TIM2_ARR  (100)
#define TIM2_PSC  (72)
	
/*Cycle 范围0-100 越大led越亮*/
#define REDLED_PWM_CYCLE( Cycle )         TIM_SetCompare2( TIM2, Cycle )
#define WHITELED_PWM_CYCLE( Cycle )       TIM_SetCompare3( TIM2, Cycle )
#define YELLOWLED_PWM_CYCLE( Cycle )      TIM_SetCompare4( TIM2, Cycle )

void LED_Init(void);//初始化
	 				    
#endif
