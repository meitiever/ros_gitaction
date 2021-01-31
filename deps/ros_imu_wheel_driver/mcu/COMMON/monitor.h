#ifndef __MONITOR_H
#define __MONITOR_H
#include <stdbool.h>
#include <stdint.h>

extern uint8_t Monitor_Thread_Log_Open_Flag;
void monitorTask(void *param);

#endif 
