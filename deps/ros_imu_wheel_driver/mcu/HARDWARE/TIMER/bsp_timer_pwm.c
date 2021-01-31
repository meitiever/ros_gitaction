/**
******************************************************************************
* �ļ���    		bsp_timer_pwm.c
* ������			  ��ʱ��PWM�������
*	˵��:         PWM���IO���ã��Ƽ�ʹ�ö�ʱ��2��3��4��5
* ���ߣ�		    о�����ӿƼ�
* �汾�� 			  V1.0
* ʱ�䣺    		2020/7/28
******************************************************************************
*/
#include "bsp_timer_pwm.h"    

void vAll_PWM_Init(void )
{
    TIM3_PWM_Init();
	  TIM4_PWM_Init();
	  TIM5_PWM_Init();
}
void TIM3_PWM_Init(void)
{
    TIM3_CH34_Init(TIM3_ARR,TIM3_PSC);
}
void TIM3_CH34_Init(u16 arr,u16 psc)
{
    GPIO_InitTypeDef         GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//ʹ��PB�˿�ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//ʹ�ܶ�ʱ��3ʱ��
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 		 //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    TIM_TimeBaseStructure.TIM_Period = arr-1; 					//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	  TIM_TimeBaseStructure.TIM_Prescaler =psc-1; 					//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 	
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ
	  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 		
    

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
 	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ե�
    TIM_OCInitStructure.TIM_Pulse=0;
	  TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM5 OC2
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM5 OC2
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM5��CCR2�ϵ�Ԥװ�ؼĴ���
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM5��CCR2�ϵ�Ԥװ�ؼĴ���
    
    TIM_Cmd(TIM3, ENABLE);    
}

void TIM5_PWM_Init(void)
{
    TIM5_CH2_Init(TIM5_ARR,TIM5_PSC);
}
void TIM5_CH2_Init(u16 arr,u16 psc)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��PA�˿�ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);	//ʹ�ܶ�ʱ��5ʱ��
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 		 //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    TIM_TimeBaseStructure.TIM_Period = arr-1; 					//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
    TIM_TimeBaseStructure.TIM_Prescaler =psc-1; 					//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 	
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 		


    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
    TIM_OCInitStructure.TIM_Pulse=0;
    TIM_OC2Init(TIM5, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM5 OC2

    TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);  //ʹ��TIM5��CCR2�ϵ�Ԥװ�ؼĴ���

    TIM_Cmd(TIM5, ENABLE);    
}
void TIM4_PWM_Init(void)
{
    TIM4_CH23_Init(TIM4_ARR,TIM4_PSC);
}
void TIM4_CH23_Init(u16 arr,u16 psc)
{
    GPIO_InitTypeDef         GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);	//ʹ��PD�˿�ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//ʹ�ܶ�ʱ��4ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	//
    
    GPIO_PinRemapConfig(GPIO_Remap_TIM4,ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 		 //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    TIM_TimeBaseStructure.TIM_Period = arr-1; 					//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	  TIM_TimeBaseStructure.TIM_Prescaler =psc-1; 					//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 	
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ
	  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 		  

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
   	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
    TIM_OCInitStructure.TIM_Pulse=0;
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);  
		TIM_OC3Init(TIM4, &TIM_OCInitStructure);  
		TIM_OC4Init(TIM4, &TIM_OCInitStructure);  

    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
		TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); 
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable); 		
    
    TIM_Cmd(TIM4, ENABLE);    
}

