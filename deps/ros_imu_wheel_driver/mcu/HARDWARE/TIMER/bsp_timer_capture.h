#ifndef __BSP_TIMER_CAPTURE_H
#define __BSP_TIMER_CAPTURE_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//声明： 版权所有，如用于商业目的请与开发者取得联系
//作者： 芯欣电子科技
//版本：  V1.0
//时间：  2016/9/2    
//Copyright(C) 
//All rights reserved		
////////////////////////////////////////////////////////////////////////////////// 

/*TIM1 捕获分辨率100Khz*/
#define TIM1_ARR (50000)
#define TIM1_PSC (720)
/*TIM8 捕获分辨率100Khz*/
#define TIM8_ARR (50000)
#define TIM8_PSC (720)
//#define TIM8_ARR (0XFFFF)
//#define TIM8_PSC (72)

void TIM1_CH1_Capture_Callback(void);
void TIM1_CH2_Capture_Callback(void);
void TIM8_CH2_Capture_Callback(void);
		
void vAll_TIM_CaptureInit(void);

void TIM1_Capture_Init(void);
void TIM8_Capture_Init(void);	

void TIM1_CH12WheelCaptureInit(u16 arr,u16 psc);
void TIM8_CH2CaptureInit(u16 arr,u16 psc);	
#endif
