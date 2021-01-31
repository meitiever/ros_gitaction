/**
******************************************************************************
* 文件：    		bsp_timer_pwm.c
* 描述：			  定时器PWM输出配置
*	说明:         PWM输出IO配置，推荐使用定时器2、3、4、5
* 作者：		    芯欣电子科技
* 版本： 			  V1.0
* 时间：    		2020/7/28
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
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//使能PB端口时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//使能定时器3时钟
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 		 //复用输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    TIM_TimeBaseStructure.TIM_Period = arr-1; 					//设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	  TIM_TimeBaseStructure.TIM_Prescaler =psc-1; 					//设置用来作为TIMx时钟频率除数的预分频值
	  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 	
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 		
    

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
 	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
    TIM_OCInitStructure.TIM_Pulse=0;
	  TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM5 OC2
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM5 OC2
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM5在CCR2上的预装载寄存器
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM5在CCR2上的预装载寄存器
    
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
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//使能PA端口时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);	//使能定时器5时钟
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 		 //复用输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    TIM_TimeBaseStructure.TIM_Period = arr-1; 					//设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
    TIM_TimeBaseStructure.TIM_Prescaler =psc-1; 					//设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 	
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 		


    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
    TIM_OCInitStructure.TIM_Pulse=0;
    TIM_OC2Init(TIM5, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM5 OC2

    TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);  //使能TIM5在CCR2上的预装载寄存器

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
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);	//使能PD端口时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//使能定时器4时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	//
    
    GPIO_PinRemapConfig(GPIO_Remap_TIM4,ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 		 //复用输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    TIM_TimeBaseStructure.TIM_Period = arr-1; 					//设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	  TIM_TimeBaseStructure.TIM_Prescaler =psc-1; 					//设置用来作为TIMx时钟频率除数的预分频值
	  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 	
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 		  

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
   	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
    TIM_OCInitStructure.TIM_Pulse=0;
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);  
		TIM_OC3Init(TIM4, &TIM_OCInitStructure);  
		TIM_OC4Init(TIM4, &TIM_OCInitStructure);  

    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
		TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); 
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable); 		
    
    TIM_Cmd(TIM4, ENABLE);    
}

