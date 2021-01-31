/**
******************************************************************************
* �ļ���    		bsp_timer_capture.c
* ������			  ��ʱ�����벶������
*	˵��:         ���벶��IO����
* ���ߣ�		    о�����ӿƼ�
* �汾�� 			  V1.0
* ʱ�䣺    		2020/7/28
******************************************************************************
*/
#include "bsp_timer_capture.h"    

 
void vAll_TIM_CaptureInit(void)
{
    TIM1_Capture_Init();
//	  TIM8_Capture_Init();

}
void TIM1_Capture_Init(void)
{
   TIM1_CH12WheelCaptureInit(TIM1_ARR,TIM1_PSC);
}
void TIM8_Capture_Init(void)
{
	 TIM8_CH2CaptureInit(TIM8_ARR,TIM8_PSC);
}
void TIM1_CH12WheelCaptureInit(u16 arr,u16 psc)
{
    GPIO_InitTypeDef         GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef         NVIC_InitStructure;
    TIM_ICInitTypeDef        TIM_ICInitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);	//ʹ��PE�˿�ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	//ʹ�ܶ�ʱ��1ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	//ʹ�ܶ�ʱ��1ʱ��
  
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);      //Timer1��ȫ��ӳ��   
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		 //�������
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    
    TIM_TimeBaseStructure.TIM_Period = arr-1; 					//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
		TIM_TimeBaseStructure.TIM_Prescaler =psc-1; 					//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 	
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ
		TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 		
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;                    //CC1S=01 IC1 connect to TI1
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;	
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 
    TIM_ICInitStructure.TIM_ICFilter = 0x05;                
    TIM_ICInit( TIM1, &TIM_ICInitStructure );
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit( TIM1, &TIM_ICInitStructure );
    
    TIM_ClearITPendingBit( TIM1,TIM_IT_CC1|TIM_IT_CC2 );
    TIM_ITConfig( TIM1,TIM_IT_CC1, ENABLE );  
    TIM_ITConfig( TIM1,TIM_IT_CC2, ENABLE );
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;           
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		            
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			            
    NVIC_Init( &NVIC_InitStructure );	
    
    TIM_Cmd(TIM1, ENABLE);    
}
void TIM8_CH2CaptureInit(u16 arr,u16 psc)
{
//    GPIO_InitTypeDef         GPIO_InitStructure;
//    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//    NVIC_InitTypeDef         NVIC_InitStructure;
//    TIM_ICInitTypeDef        TIM_ICInitStructure;
//    
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	//ʹ��PC�˿�ʱ��
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);	//ʹ�ܶ�ʱ��8ʱ��
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	//ʹ�ܶ�ʱ��1ʱ��
//      
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		 //�������
//    GPIO_Init(GPIOC, &GPIO_InitStructure);
//    
//    TIM_TimeBaseStructure.TIM_Period = arr-1; 					//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
//		TIM_TimeBaseStructure.TIM_Prescaler =psc-1; 					//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
//		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 	
//		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ
//		TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); 		
//    
//    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;                    //CC1S=01 IC1 connect to TI1
//    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;	
//    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_TRC; 
//    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 
//    TIM_ICInitStructure.TIM_ICFilter = 0x05;                
//    TIM_ICInit( TIM8, &TIM_ICInitStructure );
//		
////	 TIM1->CCER &= (uint16_t)~((uint16_t)TIM_CCER_CC1E);
////   TIM1->CCMR1 |= 0x03;
//   TIM1->SMCR |= (1 << 6);
//   TIM1->SMCR &= ~((1 << 5) | (1 << 4));
////   TIM1->CCER |= ((uint16_t)TIM_CCER_CC1E);
//	 
//    
//    TIM_ClearITPendingBit( TIM8,TIM_IT_CC2 );
//    TIM_ITConfig( TIM8,TIM_IT_CC2, ENABLE );
//    
//    NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;           
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		            
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			            
//    NVIC_Init( &NVIC_InitStructure );	
//    
//    TIM_Cmd(TIM8, ENABLE);    
}

//void TIM1_CC_IRQHandler( void )
//{
//    if(SET == TIM_GetITStatus(TIM1,TIM_IT_CC1) )
//    {
//			  TIM1_CH1_Capture_Callback();
//        TIM_ClearITPendingBit( TIM1,TIM_IT_CC1 ); //����жϱ�־λ    
//        
//    }

//    if(SET == TIM_GetITStatus(TIM1,TIM_IT_CC2) )
//    {
//			  TIM1_CH2_Capture_Callback();
//        TIM_ClearITPendingBit( TIM1,TIM_IT_CC2 ); //����жϱ�־λ
//       

//    }
//}
void TIM8_CC_IRQHandler( void )
{

    if(SET == TIM_GetITStatus(TIM8,TIM_IT_CC2) )
    {
			  TIM8_CH2_Capture_Callback();
        TIM_ClearITPendingBit( TIM8,TIM_IT_CC2 ); //����жϱ�־λ
       

    }
}
void __attribute__((weak)) TIM1_CH1_Capture_Callback(void) { }
void __attribute__((weak)) TIM1_CH2_Capture_Callback(void) { }
void __attribute__((weak)) TIM8_CH2_Capture_Callback(void) { }

