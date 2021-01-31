#ifndef __BSP_TIMER_CAPTURE_H
#define __BSP_TIMER_CAPTURE_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ ��Ȩ���У���������ҵĿ�����뿪����ȡ����ϵ
//���ߣ� о�����ӿƼ�
//�汾��  V1.0
//ʱ�䣺  2016/9/2    
//Copyright(C) 
//All rights reserved		
////////////////////////////////////////////////////////////////////////////////// 

/*TIM1 ����ֱ���100Khz*/
#define TIM1_ARR (50000)
#define TIM1_PSC (720)
/*TIM8 ����ֱ���100Khz*/
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
