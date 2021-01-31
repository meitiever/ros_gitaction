/**
******************************************************************************
* �ļ���    		bsp_usart3.c
* ������			  ����3��������
*	˵��:         ����3�������ļ�������GPIO���á����ڽ����жϡ�
* ���ߣ�		    wildfire team 
* �汾�� 			  V1.0
* ʱ�䣺    		2012
******************************************************************************
*/
#include "bsp_usart3.h"                                                                                                                                                                                                                      

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"


//static xSemaphoreHandle waitUntilSendDone;
static xSemaphoreHandle uart3Busy;

void UART3_DMA_Init(void);
/*********************************************************************************************/
/*
 * ��������USART3_Config
 * ����  ��USART3 GPIO ����,����ģʽ����
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */
void USART3_Config( void )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* config USART3 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* USART2 GPIO config */
  /* Configure USART2 Tx (PB.10) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	    
  /* Configure USART2 Rx (PA.03) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	  
	/* USART3 mode config */
	USART_InitStructure.USART_BaudRate =230400;               
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure); 
  UART3_DMA_Init();
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* ʹ�ܴ���3�����ж� */
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	/* ʹ�ܴ���3���߿����ж� */
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
	
	USART_Cmd(USART3, ENABLE);
	
}
u8 Uart3_Tx_Buffer1[256];
static   DMA_InitTypeDef DMA_InitStructure;

void UART3_DMA_Init(void)
{
	uart3Busy = xSemaphoreCreateBinary();			/*����æ ��ֵ�ź���*/
	xSemaphoreGive(uart3Busy); 
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);		
  NVIC_InitTypeDef NVIC_InitStructure;

	DMA_DeInit(DMA1_Channel2);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART3->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Uart3_Tx_Buffer1;  
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);
	
	DMA_Init(DMA1_Channel2,&DMA_InitStructure);
	DMA_ClearFlag(DMA1_FLAG_GL2);  // clear all DMA flags
	DMA_ITConfig(DMA1_Channel2,DMA_IT_TC,ENABLE); 
//	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
////	
//	isUart2TxDmaInitialized=true;

}
#include <string.h>

void uarts3SendDataDmaBlocking(u32 size, u8* data)
{
	{
		xSemaphoreTake(uart3Busy, portMAX_DELAY);
		memcpy(Uart3_Tx_Buffer1, data, size);		/*�������ݵ�DMA������*/
		DMA_InitStructure.DMA_BufferSize = size;
		//initialDMACount = size;
		DMA_Init(DMA1_Channel2, &DMA_InitStructure);	/*���³�ʼ��DMA������*/
		DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);/*����DMA��������ж�*/	
		USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);/* ʹ��USART DMA TX���� */
//		USART_ClearFlag(USART3, USART_FLAG_TC);		/* �����������жϱ�־λ */
		DMA_Cmd(DMA1_Channel2, ENABLE);	/* ʹ��DMA USART TX������ */
		//xSemaphoreGive(uart3Busy);
	}
}

void  DMA1_Channel2_IRQHandler(void)	
{
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
//	printf("aa\r\n");
	if(DMA_GetITStatus(DMA1_FLAG_TC2)==SET)
	{
//		printf("bb\r\n");
		DMA_ClearFlag(DMA1_FLAG_GL2);        
		DMA_Cmd(DMA1_Channel2, DISABLE);  
		DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, DISABLE);
	  USART_DMACmd(USART3, USART_DMAReq_Tx, DISABLE);
	}
	xSemaphoreGiveFromISR(uart3Busy, &xHigherPriorityTaskWoken);
}

/*
 * ��������USART3_IRQHandler
 * ����  ������3�жϺ���
 * ����  ����
 * ���  ����
 * ����  ���� 
 * ����  ��
 */
u8 test_flags1 =0;
u8 test_flags2 =0;
u8 test_flags[12];
u8 test_cnt =0;
u8 g_flag ;
u8 g_val ;
void USART3_IRQHandler( void )
{	
	u8 ch;	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
//		USART_ClearITPendingBit(USART2,USART_IT_RXNE); 

		test_flags[test_cnt]  = USART_ReceiveData( USART3 );
//		printf("%x,",test_flags[test_cnt]);
//		if(test_flags1 ==1)
//		{
//			g_val =test_flags[8];
//			g_flag =1;
//			
//		}
//		if(test_flags[0]==0x59&&test_flags[1]==0x53)
//		{
//			if(test_flags[6]==0x53 && test_flags[7]==0x01) {test_flags1 =1;}
//		}
		if(test_cnt>=10) {g_flag =1; test_cnt =0;}
		test_cnt ++;
			
	}
	if ( USART_GetITStatus( USART3, USART_IT_IDLE ) == SET )                                         
	{
		USART_ClearFlag(USART3,USART_FLAG_ORE);  //
		USART_ReceiveData(USART3); // 
		ch = USART_ReceiveData( USART3 );                                                              
  }
}



static void send_byte(unsigned char byte)
{
  USART_SendData(USART3, (unsigned char) byte);
 	while( USART_GetFlagStatus(USART3,USART_FLAG_TC)!= SET);	
}


void UART3_SEND_STRING(u8 *pDATA, u8 NUM_OF_DATA)
{
  u8 i;
	for(i=0;i<NUM_OF_DATA;i++)
	{
	 send_byte(pDATA[i]);
	}
}



