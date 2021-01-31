#ifndef __BSP_TIMER_PWM_H
#define __BSP_TIMER_PWM_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ ��Ȩ���У���������ҵĿ�����뿪����ȡ����ϵ
//���ߣ� о�����ӿƼ�
//�汾��  V1.0
//ʱ�䣺  2016/9/2    
//Copyright(C) 
//All rights reserved		
////////////////////////////////////////////////////////////////////////////////// 

/*TIM3 Ƶ��Ϊ12KHz*/
#define TIM3_ARR (500)
#define TIM3_PSC (12)
/*TIM4 Ƶ��Ϊ12KHz*/
#define TIM4_ARR (500)
#define TIM4_PSC (12)
/*TIM5 Ƶ��Ϊ12KHz*/
#define TIM5_ARR (500)
#define TIM5_PSC (12)

void vAll_PWM_Init(void );

void TIM3_PWM_Init(void);
void TIM4_PWM_Init(void);
void TIM5_PWM_Init(void);

void TIM3_CH34_Init(u16 arr,u16 psc);
void TIM4_CH23_Init(u16 arr,u16 psc);
void TIM5_CH2_Init(u16 arr,u16 psc);
#endif
