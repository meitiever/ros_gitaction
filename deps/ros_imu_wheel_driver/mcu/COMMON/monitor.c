#include "monitor.h"
#include "bsp_gpio.h"    

#include "delay.h"
#include "config.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "encoder.h"
#include "syslog.h"
#include "math.h"
#include "usart1.h"	  
#include "string.h"

#define  PERIOD_STATUS		        100
#if  MONITOR_DEBUG_ENABLE
#define Monitor_LOG(...)   { \
		    printf("%d -%s,", __LINE__,__FUNCTION__); \
        printf(__VA_ARGS__); \
      }
#else
#define Monitor_LOG(...) ((void)0)
#endif
uint8_t Monitor_Thread_Log_Open_Flag=1;
//
//
//
extern short yaw,pitch,roll,x,y,z;
extern 			uint16_t encoder1 ,encoder2 ,direct1 ,direct2 ;
extern 			float dis_x,dis_y;
extern 			u32 time_test ;
extern 			float ang_dt ;
extern      u8 Collision_Left  , Collision_Right ;
u32 s_test = 0;
void monitor_system_status(void)
{
	static u16 count_ms = 1;
	//if(Monitor_Thread_Log_Open_Flag==0){
	if(0){
	#if MONITOR_PERIOD_STATUS
	if(!(count_ms % PERIOD_STATUS))
	{
		  int16_t MonitorWheelLeftEncoder=0;
		  int16_t MonitorWheelRightEncoder=0;
		  getWheelEncoderData(&MonitorWheelLeftEncoder,&MonitorWheelRightEncoder);	


//		u8 val =GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7);
		char buf[50];
//		sprintf(buf,"%f,%f,%f,\r\n",dis_y/1000,dis_x/1000,(float)yaw/100);
		//sprintf(buf,"%d,%d,%f,\r\n",Collision_Left,Collision_Right,(float)yaw/100);
		//sprintf(buf,"yaw:%d,%d %d,%d,\r\n",direct1,direct2,encoder1,encoder2);
		sprintf(buf,"time:%d\r\n",time_test);
//		vUart1_SendString(buf,strlen(buf));
		uarts1SendDataDmaBlocking(strlen(buf),buf);

	}
	#endif
  }
	if(++count_ms>=65535) 
		count_ms = 1;	
}
void monitorTask(void *param)
{
	while(1)
	{
		monitor_system_status();
		vTaskDelay(1);
	}
}
