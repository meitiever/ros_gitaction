#ifndef __FRAMEHANDLE_H
#define __FRAMEHANDLE_H

#include "sys.h"
#include "protocol.h"
#include "protocol_uart1.h"



void vFrameDataHandlerTask(void *p);
void vFrameDataHandlerUsart1Task(void *p);
void vFrameRxPackageTask(void *p);
void vUart1FrameRxPackageTask(void *p);
#endif

