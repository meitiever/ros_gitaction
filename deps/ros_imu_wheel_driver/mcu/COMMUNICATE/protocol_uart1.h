#ifndef PROTOCOL_UART_H_
#define PROTOCOL_UART_H_

#include "sys.h"
#include "stdbool.h"
#include "string.h"
#include "protocol.h"


extern Rx_FrameRecord FR_uart1_Rx;
extern Tx_FrameRecord FR_uart1_Tx;

FrameErr_e eFrame_uart1_Analy(u8 *buf,u8 length,Rx_FrameRecord *RFR);

void uart1ReceiveAnl(Rx_FrameRecord *anlPacket);
void encoder_open(void);
void encoder_close(void);
#endif

