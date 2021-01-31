#include "syslog.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "usart.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include <string.h>

#define USART1_TX_QUEUE_SIZE		(0xFFU)
#define USART1_RX_QUEUE_SIZE		(0xFFU)


QueueHandle_t xUsart1_TxQueue = NULL; 
QueueHandle_t xUsart1_RxQueue = NULL;

SemaphoreHandle_t xUsart1_TxMutex = NULL;
SemaphoreHandle_t xUsart1_RxMutex = NULL;


static void prvUsart1_QueueInit(void)
{
	static uint8_t Usart1_QueueSetFlag = 0x00;
	
	if (Usart1_QueueSetFlag == 0x00)
	{
		/* Create the queue of chars that are waiting to be sent to console. */
		xUsart1_TxQueue = xQueueCreate(USART1_TX_QUEUE_SIZE, sizeof(char));
		/* Create the queue used to hold characters received from console. */
		xUsart1_RxQueue = xQueueCreate(USART1_RX_QUEUE_SIZE, sizeof(char));
		if ((xUsart1_TxQueue && xUsart1_RxQueue) == NULL)
		{
			for (;;)
			{
				;
			}
		}
		Usart1_QueueSetFlag = 0x01;
	}
}
static void prvUsart1_MutexInit(void)
{
	static uint8_t Usart1_MutexSetFlag = 0x00;
	
	if (Usart1_MutexSetFlag == 0x00)
	{
		xUsart1_TxMutex = xSemaphoreCreateMutex();
		xUsart1_RxMutex = xSemaphoreCreateMutex();
		if ((xUsart1_TxMutex && xUsart1_RxMutex) == NULL)
		{
			for (;;)
			{
				;
			}
		}
		Usart1_MutexSetFlag = 0x01;
	}
}
void prvUsart1_Init(void)
{
	prvUsart1_QueueInit();
	prvUsart1_MutexInit();
}
uint8_t ucUsart1_GetChar(char *pcRxedChar, const uint32_t xBlockTime)
{
	if (xQueueReceive(xUsart1_RxQueue, pcRxedChar, xBlockTime) == pdPASS)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}


uint8_t ucUsart1_PutChar(const char cOutChar, const uint32_t xBlockTime)
{
	if (xQueueSend(xUsart1_TxQueue, &cOutChar, xBlockTime) == pdPASS)
	{
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		return 0;
	}
	else
	{
		return 1;
	}
}


uint8_t ucUsart1_PutString(const char *const pcString, const uint32_t ulStringLength, const uint32_t xBlockTime)
{
	uint8_t ucPutRes = 0;

	for (uint32_t i = 0; i < ulStringLength; i++)
	{
		ucPutRes &= ucUsart1_PutChar(pcString[i], xBlockTime);
	}
	return ucPutRes;
}

 char prvGetChar(void)
{
	char rxtemp = 0;
	xSemaphoreTake(xUsart1_RxMutex, portMAX_DELAY);
	ucUsart1_GetChar(&rxtemp, portMAX_DELAY);
	xSemaphoreGive(xUsart1_RxMutex);
	return rxtemp;
}

 void prvPutChar(char cChar)
{
	xSemaphoreTake(xUsart1_TxMutex, portMAX_DELAY);
	ucUsart1_PutChar(cChar, portMAX_DELAY);
	xSemaphoreGive(xUsart1_TxMutex);
}

 void prvPutString(const char * const pcChar)
{
	xSemaphoreTake(xUsart1_TxMutex, portMAX_DELAY);
	ucUsart1_PutString(pcChar, strlen(pcChar), portMAX_DELAY);
	xSemaphoreGive(xUsart1_TxMutex);
}

void USART_SendStr(USART_TypeDef *USARTx, uint32_t ulLen, const char *pcData)
{
	uint32_t i;
	for(i=0; i<ulLen; i++)
	{
		USART_SendData(USARTx, (uint8_t)*(pcData++));
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
		{
			;
		}
	}
}



static char logbuf[1024] = {0};

int log_record(const char *format, ...)
{
	va_list args;
  int result;
	
	memset(logbuf,0,sizeof (logbuf));
	va_start(args, format);
	result = vsprintf(logbuf,format, args);
	va_end(args);
	
	USART_SendStr(USART1, strlen(logbuf), logbuf);
	
	return result;
}


int ts_printf(const char *format, ...)
{
	va_list args;
	int result;
	memset(logbuf,0,sizeof (logbuf));
	xSemaphoreTake(xUsart1_TxMutex, portMAX_DELAY);
	va_start(args, format);
	result = vprintf(format, args);
	va_end(args);
	xSemaphoreGive(xUsart1_TxMutex);
	return result;
}

void __attribute__((used)) Usart1_Rx_Callback()
{
	BaseType_t HigherPriorityTaskWoken = pdTRUE;
	/*·ÀÖ¹±àÒëÆ÷ÓÅ»¯*/
	volatile uint32_t temp;
	char cChar;
	cChar = (uint8_t)USART_ReceiveData(USART1);
	xQueueSendFromISR(xUsart1_RxQueue, &cChar, &HigherPriorityTaskWoken);	
}
void __attribute__((used)) Usart1_Tx_Callback()
{
	BaseType_t HigherPriorityTaskWoken = pdTRUE;
	/*·ÀÖ¹±àÒëÆ÷ÓÅ»¯*/
	volatile uint32_t temp;
	char cChar;
		/* The interrupt was caused by the THR becoming empty.  Are there any
	more characters to transmit? */
	if (xQueueReceiveFromISR(xUsart1_TxQueue, &cChar, &HigherPriorityTaskWoken) == pdPASS)
	{
		/* A character was retrieved from the buffer so can be sent to the
		THR now. */
		USART_SendData(USART1, (uint8_t)cChar);
	}
	else
	{
		USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	}
}
void __attribute__((used)) Usart1_Idle_Callback()
{
	
}
