/**
******************************************************************************
* �ļ���    		bsp_timer.c
* ������			  ��ʱ��ʱ��
*	˵��:         ʵ�ֶ�ʱ����,TIM6��TIM7ͨ�ö�ʱ��
* ���ߣ�		    о�����ӿƼ�
* �汾�� 			  V1.0
* ʱ�䣺    		2020/7/28
******************************************************************************
*/
#include "bsp_timer.h" 

//
//ͨ�ö�ʱ����ʼ��
//
void vALL_TIM_Init(void)
{
	#if TIMER6_ENABLE
	TIM2_Int_Init(TIM6_ARR,TIM6_PSC);
	#endif
	#if TIMER7_ENABLE
	TIM7_Int_Init(TIM7_ARR,TIM7_PSC);
	#endif
}
//
//��ʱ��6��ʼ������
//
u32 time_test =0;
void TIM2_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //ʱ��ʹ��
	
	//��ʱ��TIM6��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; 					//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ 1M/(4999+1) = 200HZ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 					//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 72M/(71+1) = 1M
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 	//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 			//����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); 					//ʹ��ָ����TIM6�ж�,��������ж�

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  			//TIM6�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  	//��ռ���ȼ�5��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  		//�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 			//IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  							//��ʼ��NVIC�Ĵ���

	TIM_Cmd(TIM2, ENABLE);  									//ʹ��TIM6					 
}
void TIM2_IRQHandler(void)   
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) 
	{
		TIM6_Callback();
		time_test++;
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update); 	
	}
}
void TIM7_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);  
	
	TIM_TimeBaseInitStructure.TIM_Period = arr; 	
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc; 
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE); 
	TIM_Cmd(TIM7,ENABLE); 
	#if 0
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x09; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; 
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
}
void TIM7_IRQHandler(void)   
{
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET) 
	{
		TIM7_Callback();
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update); 	
	}
}
void __attribute__((weak)) TIM6_Callback(void) { }
void __attribute__((weak)) TIM7_Callback(void) { }
