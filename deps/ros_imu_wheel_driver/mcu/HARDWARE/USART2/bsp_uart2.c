#include "bsp_uart2.h"
#include "usart1.h"
#include "string.h"

uint8_t ucUSART2TrainsmitBuffer[USART2_TRANSMIT_GROOVE_SIZE][USART2_TRANSMIT_BUFFER_SIZE] = {0}; 
uint8_t ucUSART2TrainsmitLength[USART2_TRANSMIT_GROOVE_SIZE];    
uint8_t ucDMA1_Stream7TransmitGrooveIndex = 0;                   
uint8_t ucUSART2_TransmitMessageNumber = 0;                        
uint8_t ucWtiteDataToUSART2TransmitGrooveIndex = 0;                                                            
uint8_t ucUSART2_TransmitWritingBytePointer = 0;                 

uint8_t ucUSART2ReceiveBuffer[USART2_RECEIVE_GROOVE_SIZE][USART2_RECEIVE_BUFFER_SIZE] = {0};    
uint8_t ucUSART2ReceiveBufferLength[USART2_RECEIVE_GROOVE_SIZE];                                  
uint8_t ucUSART2ReceiveGrooveIndex = 0;                          
uint8_t ucUSART2ReadBufferIndex = 0;                             
uint8_t ucUSART2ReceiveWritingBytePointer = 0;                    
uint8_t ucDMA1_Stream6ReceiveGrooveIndex = 0;                    
uint8_t ucUSART2_ReceiveMessageNumber = 0;                       

void vUart2_Init(void)
{
    vUart2PinFunction_Init(USART2_BOUND);
    vDma1_7_Usart2_TX_Init(ucUSART2TrainsmitBuffer[ucDMA1_Stream7TransmitGrooveIndex],USART2_TRANSMIT_BUFFER_SIZE);
    vDma1_6_Usart2_RX_Init(ucUSART2ReceiveBuffer[ucUSART2ReceiveGrooveIndex],USART2_RECEIVE_BUFFER_SIZE);
}

void vUsart2Interactive( void )
{  
    static USART_DMAState DMA1_Stream7Flag = USART_DMA_DONE;     
    if( ucUSART2_TransmitMessageNumber ) 
    {        
        if( DMA_GetFlagStatus( DMA1_FLAG_TC7 )== SET )
        {
            DMA_ClearFlag( DMA1_FLAG_TC7 );
            DMA1_Stream7TransmitDone(  );     	
            DMA1_Stream7Flag = USART_DMA_DONE;
        }    
        if( DMA1_Stream7Flag == USART_DMA_DONE )
        {   	
            DMA1_Stream7Flag = USART_DMA_UNDONE;
            DMA_Cmd( DMA1_Channel7, DISABLE );
            ucUSART2_TransmitMessageNumber--;                                                          
            DMA1_Channel7->CMAR = (uint32_t)ucUSART2TrainsmitBuffer[ucDMA1_Stream7TransmitGrooveIndex]; 
            DMA1_Channel7->CNDTR = ucUSART2TrainsmitLength[ucDMA1_Stream7TransmitGrooveIndex];           
            DMA_Cmd( DMA1_Channel7, ENABLE );                                                           
        }   
    }                    
}

/*******************************************************************************
* Function Name  : vUart2PinFunction_Init(u32 bound)
* Description    : 串口2和对应IO的初始化
* Input          : 波特率
* Output         : None
* Return         : None
*******************************************************************************/
void vUart2PinFunction_Init(u32 bound)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    
//  GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);
   
	//USART2_RX	  GPIOD6初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  
  
	//USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

	USART_Init(USART2, &USART_InitStructure); //初始化串口2

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_DMACmd(USART2,USART_DMAReq_Tx, ENABLE);
	USART_DMACmd(USART2,USART_DMAReq_Rx, ENABLE);
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
	USART_Cmd(USART2, ENABLE);                    
}
void vDma1_7_Usart2_TX_Init(u8 *MemoryAddr,u32 DataSize)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	

    DMA_DeInit(DMA1_Channel7);   

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)( &( USART2->DR ) );  
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)MemoryAddr;  
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  
    DMA_InitStructure.DMA_BufferSize = DataSize;  
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
    DMA_Init(DMA1_Channel7, &DMA_InitStructure);  
    DMA_ClearFlag( DMA1_FLAG_TC7 );
}
void vDma1_6_Usart2_RX_Init(u8 *MemoryAddr,u32 DataSize)
{
    DMA_InitTypeDef DMA_InitStructure;
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	

    DMA_DeInit(DMA1_Channel6);   

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)( &( USART2->DR ) );  
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)MemoryAddr;  
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  
    DMA_InitStructure.DMA_BufferSize = DataSize; 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
    DMA_Init(DMA1_Channel6, &DMA_InitStructure);  
    
    DMA_Cmd(DMA1_Channel6, ENABLE);  
}
//
void USART2_IRQHandler(void)                	
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) 
    {

    }
    else if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        DMA_Cmd( DMA1_Channel6, DISABLE ); 
        DMA_ClearFlag( DMA1_FLAG_TC6 );                         
        USART2->SR;
        USART2->DR;
        ucUSART2ReceiveBufferLength[ucDMA1_Stream6ReceiveGrooveIndex] = USART2_RECEIVE_BUFFER_SIZE - DMA_GetCurrDataCounter( DMA1_Channel6 );     
        DMA1_Stream6ReceiveDone(  );
        DMA1_Channel6->CMAR = (uint32_t)ucUSART2ReceiveBuffer[ucDMA1_Stream6ReceiveGrooveIndex];
        DMA1_Channel6->CNDTR = USART2_RECEIVE_BUFFER_SIZE;                        
        DMA_Cmd( DMA1_Channel6, ENABLE );         
    }
} 
/*******************************************************************************
* Function Name  : vUart2_SendString(char * buf,int len)
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void vUart2_SendString(char * buf,int len)
{
		memcpy(ucUSART2TrainsmitBuffer[ucWtiteDataToUSART2TransmitGrooveIndex],buf,len);//写入信息
		ucUSART2TrainsmitLength[ucWtiteDataToUSART2TransmitGrooveIndex]=len;
		WriteDataToUSART2TraismitBufferDone();
}
