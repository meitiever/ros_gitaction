#ifndef __USART3_H
#define	__USART3_H

#include "stm32f10x.h"
#include <stdio.h>

void USART3_Config( void );
extern void USART3_printf(USART_TypeDef* USARTx, char *Data, ...);
extern  void UART3_SEND_STRING(u8 *pDATA, u8 NUM_OF_DATA);
void uarts3SendDataDmaBlocking(u32 size, u8* data);
#endif /* __USART2_H */
