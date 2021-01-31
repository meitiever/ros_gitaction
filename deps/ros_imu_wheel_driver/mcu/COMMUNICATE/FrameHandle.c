#include "framehandle.h"
#include "bsp.h"
#include "bsp_uart2.h"	
#include "usart1.h"	  

/*FreeRtos includes*/
#include "FreeRTOS.h" 
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

static xQueueHandle vFrameRxPackage_MsgQueue;
static xQueueHandle vUart1FrameRxPackage_MsgQueue;
//static xQueueHandle vFrameTxPackage_MsgQueue;
bool vFrameRxPackageBlocking(Rx_FrameRecord *p);
bool vUart1FrameRxPackageBlocking(Rx_FrameRecord *p);
/*
 * Filename   ：framehandle.c
 * Author     : PuSheng 
 * Version    : 1.0
 * Date       : 2019.12.25 
 * Discription : 协议帧处理函数
*/
/*
* Function Name  : vFrameDataHandlerTask
* Description    : 协议帧数据处理任务函数
* Input          : 参数 
* Output         : None 
* Return         : None
*/

/*
* Function Name  : vFrameDataHandlerTask
* Description    : 帧处理函数任务 将串口接受到的数据分析为协议帧数据 
* Input          : p 参数
* Output         : None 
* Return         : None
*/
void vFrameDataHandlerTask(void *p)
{
	u8 length=0;
	
    while(1)
    {
        /*串口接收到数据 并且帧数据已经全部处理完毕*/
        if(ucUSART2_ReceiveMessageNumber)
        {
            length=ucUSART2ReceiveBufferLength[ucUSART2ReadBufferIndex];//获取接收数据的长度
            FrameErr_e err=eFrame_Analy(ucUSART2ReceiveBuffer[ucUSART2ReadBufferIndex],length,&FR_Rx);//解析数据 
					  if(err == FrameSuccess) vFrameRxPackageBlocking(&FR_Rx);
            USART2ReceiveDataReadDone( );//串口数据读取完毕             
        }                         
		vTaskDelay(5);
    }
}

void vFrameDataHandlerUsart1Task(void *p)
{
	u8 length=0;
	
    while(1)
    {
        /*串口接收到数据 并且帧数据已经全部处理完毕*/
        if(ucUSART1_ReceiveMessageNumber)
        {
            length=ucUSART1ReceiveBufferLength[ucUSART1ReadBufferIndex];//获取接收数据的长度
            FrameErr_e err=eFrame_uart1_Analy(ucUSART1ReceiveBuffer[ucUSART1ReadBufferIndex],length,&FR_uart1_Rx);//解析数据 
					  if(err == FrameSuccess) vUart1FrameRxPackageBlocking(&FR_uart1_Rx);
            USART1ReceiveDataReadDone( );//串口数据读取完毕             
        }                         
		vTaskDelay(5);
    }
}
/*
* Function Name  : Init_FrameRxPackageMsgQueue
* Description    : 初始化
* Input          : None
* Output         : None 
* Return         : int
*/
int Init_FrameRxPackageMsgQueue (void) {
 
	vFrameRxPackage_MsgQueue = xQueueCreate(5, sizeof(Rx_FrameRecord));
	vUart1FrameRxPackage_MsgQueue = xQueueCreate(5, sizeof(Rx_FrameRecord));
  if (vFrameRxPackage_MsgQueue == NULL) {
		printf("Message Queue object not created, handle failure \r\n");
  }
  if (vUart1FrameRxPackage_MsgQueue == NULL) {
		printf("vUart1FrameRxPackage_MsgQueue object not created, handle failure \r\n");
  } 	
  return(0);
}
/*
* Function Name  : vFrameRxPackageTask
* Description    : 帧处理函数任务 
* Input          : p 参数
* Output         : None 
* Return         : None
*/
void vFrameRxPackageTask(void *p)
{
  Rx_FrameRecord msg;

	Init_FrameRxPackageMsgQueue();
  while (1) {
    xQueueReceive(vFrameRxPackage_MsgQueue, &msg, portMAX_DELAY);
		atkpReceiveAnl(&msg);
  }
}

void vUart1FrameRxPackageTask(void *p)
{
  Rx_FrameRecord msg;

  while (1) {
    xQueueReceive(vUart1FrameRxPackage_MsgQueue, &msg, portMAX_DELAY);
		uart1ReceiveAnl(&msg);
  }
}
/*
* Function Name  : vFrameRxPackageBlocking
* Description    : 帧处理函数任务 
* Input          : p 参数
* Output         : None 
* Return         : None
*/
bool vFrameRxPackageBlocking(Rx_FrameRecord *p)
{
	xQueueSend(vFrameRxPackage_MsgQueue, p, portMAX_DELAY);
	return true;
}
bool vUart1FrameRxPackageBlocking(Rx_FrameRecord *p)
{
	xQueueSend(vUart1FrameRxPackage_MsgQueue, p, portMAX_DELAY);
	return true;
}
