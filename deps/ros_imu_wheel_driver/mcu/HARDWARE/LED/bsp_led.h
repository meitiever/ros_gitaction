#ifndef __BSP_LED_H
#define __BSP_LED_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ ��Ȩ���У���������ҵĿ�����뿪����ȡ����ϵ
//���ߣ� о�����ӿƼ�
//�汾��  V1.0
//ʱ�䣺  2016/9/2    
//Copyright(C) 
//All rights reserved		
////////////////////////////////////////////////////////////////////////////////// 

#define LED1    PBout(3)
#define LED2    PBout(10)
#define LED3    PBout(11)

/*TIM2 Ƶ��Ϊ10KHz*/
#define TIM2_ARR  (100)
#define TIM2_PSC  (72)
	
/*Cycle ��Χ0-100 Խ��ledԽ��*/
#define REDLED_PWM_CYCLE( Cycle )         TIM_SetCompare2( TIM2, Cycle )
#define WHITELED_PWM_CYCLE( Cycle )       TIM_SetCompare3( TIM2, Cycle )
#define YELLOWLED_PWM_CYCLE( Cycle )      TIM_SetCompare4( TIM2, Cycle )

void LED_Init(void);//��ʼ��
	 				    
#endif
