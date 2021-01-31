#include "direct.h"	
#include "bsp.h"
#include "motor.h" 

// 局部全局变量
static DirectState_t g_DirectLeftIN1State;
static DirectState_t g_DirectLeftIN2State;
static DirectState_t g_DirectRightIN1State;
static DirectState_t g_DirectRightIN2State;
//
//
//
//
u8 usGetEncoderLeftDirectIN1(void)
{
	switch((int)g_DirectLeftIN1State)
	{
		case DirectNor:
			
		break;
		case DirectFirstRise:
			
		break;
		case DirectFirstFall:
			
		break;
		case DirectSecondRise:
			
		break;
		case DirectSecondFall:
			
		break;
		case DirectComplete:
			
		break;											
	}
}
u8 usGetEncoderLeftDirectIN2(void)
{
	switch((int)g_DirectLeftIN2State)
	{
		case DirectNor:
			
		break;
		case DirectFirstRise:
			
		break;
		case DirectFirstFall:
			
		break;
		case DirectSecondRise:
			
		break;
		case DirectSecondFall:
			
		break;
		case DirectComplete:
			
		break;											
	}
}
u8 usGetEncoderRightDirectIN1(void)
{
	switch((int)g_DirectRightIN1State)
	{
		case DirectNor:
			
		break;
		case DirectFirstRise:
			
		break;
		case DirectFirstFall:
			
		break;
		case DirectSecondRise:
			
		break;
		case DirectSecondFall:
			
		break;
		case DirectComplete:
			
		break;											
	}
}
u8 usGetEncoderRightDirectIN2(void)
{
	switch((int)g_DirectRightIN2State)
	{
		case DirectNor:
			
		break;
		case DirectFirstRise:
			
		break;
		case DirectFirstFall:
			
		break;
		case DirectSecondRise:
			
		break;
		case DirectSecondFall:
			
		break;
		case DirectComplete:
			
		break;											
	}
}

//
//传感器IRQ回调函数
// 
void __attribute__((used)) EXTI8_Callback()
{
  g_tRightWheel.Direction =WheelForward;
}
//
//传感器IRQ回调函数
//
void __attribute__((used)) EXTI10_Callback()
{
	g_tRightWheel.Direction =WheelRetreat;
}
void __attribute__((used)) EXTI3_Callback()
{
  g_tLeftWheel.Direction =WheelRetreat;
}
//
//传感器IRQ回调函数
//
void __attribute__((used)) EXTI5_Callback()
{
	g_tLeftWheel.Direction = WheelForward;
}
u8 Collision_Left =0 , Collision_Right = 0;
void __attribute__((used)) EXTI7_Callback()
{
//	printf("exit 7\r\n");
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==1) {Collision_Left = 1;}
		else if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==0) {Collision_Left = 0;}
}
void __attribute__((used)) EXTI4_Callback()
{
//	printf("exit 4\r\n");
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==1) {Collision_Right = 1;}
		else if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==0) {Collision_Right = 0;}
}
