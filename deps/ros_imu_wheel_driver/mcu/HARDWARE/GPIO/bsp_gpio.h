#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//声明： 版权所有，如用于商业目的请与开发者取得联系
//作者： 芯欣电子科技
//版本：  V1.0
//时间：  2016/9/2    
//Copyright(C) 
//All rights reserved		
////////////////////////////////////////////////////////////////////////////////// 

#define   VCC3V3M_EN PCout(8)
#define   VCC5VM_EN  PEout(8)
#define   VCC5VG_EN  PAout(11)
#define   PD9     PDout(9)
#define   PD11    PDout(11)
#define   PD15    PDout(3)
#define   PD3     PDout(15)
#define   PB2     PBout(9)
#define   PC13    PCout(13)


uint8_t vGPIO_ReadInput_PB4(void);
uint8_t vGPIO_ReadInput_PD7(void);
uint8_t vGPIO_ReadInput_PD8(void);
uint8_t vGPIO_ReadInput_PD10(void);
		
void vAllOutputPinInit(void);
void vAll_EXTI_PinInit(void);	 				    
#endif
