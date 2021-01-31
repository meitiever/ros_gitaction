/**
******************************************************************************
* 文件：    		bsp_timer.c
* 描述：			  计时定时器
*	说明:         实现定时功能,TIM6、TIM7通用定时器
* 作者：		    芯欣电子科技
* 版本： 			  V1.0
* 时间：    		2020/7/28
******************************************************************************
*/
#include "bsp_timer.h" 

//
//通用定时器初始化
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
//定时器6初始化配置
//
u32 time_test =0;
void TIM2_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //时钟使能
	
	//定时器TIM6初始化
	TIM_TimeBaseStructure.TIM_Period = arr; 					//设置在下一个更新事件装入活动的自动重装载寄存器周期的值 1M/(4999+1) = 200HZ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 					//设置用来作为TIMx时钟频率除数的预分频值 72M/(71+1) = 1M
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 	//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 			//根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); 					//使能指定的TIM6中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  			//TIM6中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  	//先占优先级5级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  		//从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 			//IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  							//初始化NVIC寄存器

	TIM_Cmd(TIM2, ENABLE);  									//使能TIM6					 
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
