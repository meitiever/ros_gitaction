#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ ��Ȩ���У���������ҵĿ�����뿪����ȡ����ϵ
//���ߣ� о�����ӿƼ�
//�汾��  V1.0
//ʱ�䣺  2016/9/2    
//Copyright(C) 
//All rights reserved		
////////////////////////////////////////////////////////////////////////////////// 

#define TIMER6_ENABLE 1
#define TIMER7_ENABLE 1

#if TIMER6_ENABLE
/*TIM6 Ƶ��Ϊ 10Hz*/
#define TIM6_ARR (71)
#define TIM6_PSC (999)
#endif

#if TIMER7_ENABLE
/*TIM7 Ƶ��Ϊ 1MHz*/
#define TIM7_ARR (0xffff)
#define TIM7_PSC (72)
#endif

void vALL_TIM_Init(void);
void TIM2_Int_Init(u16 arr,u16 psc);
void TIM7_Int_Init(u16 arr,u16 psc);

void TIM6_Callback(void);
void TIM7_Callback(void);

#endif
