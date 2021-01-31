/**
******************************************************************************
* 文件：    		bsp_led.c
* 描述：			  LED驱动代码
*	说明:         初始化IO口输出模式
* 作者：		    正点原子@ALIENTEK
* 版本： 			  V1.0
* 时间：    		2012/9/2
******************************************************************************
*/
#include "bsp_gpio.h"    
//
//
//
void vAllOutputPinInit(void)
{
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);	
 
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	    		    
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
 GPIO_Init(GPIOB, &GPIO_InitStructure);	  				    
 GPIO_ResetBits(GPIOB,GPIO_Pin_9); 				

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;	    		    
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
 GPIO_Init(GPIOC, &GPIO_InitStructure);	  				    
 GPIO_SetBits(GPIOC,GPIO_Pin_13); 	
}
void vAll_EXTI_PinInit(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef  GPIO_InitStructure;
 	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE|RCC_APB2Periph_AFIO, ENABLE);	 //使能端口时钟 

	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		     //上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOD, &GPIO_InitStructure);//根据设定参数初始化GPIOD
	
	//set PC7 EXIT
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;	    		    
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource7);
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;

	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line7);
	//set PD10 EXIT
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	    		    
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 //推挽输出
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource10);
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_Line10;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;

	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line10);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	    		    
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 //推挽输出
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource8);
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;

	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line8);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	    		    
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 //推挽输出
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource3);
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_Line3;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;

	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line3);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;			
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	
	
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	    		    
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 //推挽输出
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource5);
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;

	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line5);
	
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	    		    
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 //推挽输出
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource5);
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;

	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line5);
	
	
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;	    		    
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		 //推挽输出
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource7);
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;

	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line7);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	    		    
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		 //推挽输出
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource4);
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_Line4;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;

	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line4);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;			
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
}
uint8_t vGPIO_ReadInput_PB4(void)
{
	return GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4);
} 
uint8_t vGPIO_ReadInput_PD7(void)
{
	return GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_7);
} 
uint8_t vGPIO_ReadInput_PD8(void)
{
	return GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_8);
}
uint8_t vGPIO_ReadInput_PD10(void)
{
	return GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_10);
} 

