#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "bsp.h"
#include "monitor.h"
#include "motor.h"    
#include "encoder.h"
#include "config.h"
#include "main.h"
#include "shell.h" 
#include "syslog.h"
#include "framehandle.h"
#include "usart1.h" //stdbool.h
#include "BMI088_TASK.h"

u32 vUTC_Base_Time =0x5FC735EF;
volatile  RobotState_t g_tRobotState = {0};//机器人全局状态
volatile thread_t  g_tRobotTaskState;
//定义版本信息
static const char verision[] = "\r\nSinva Robot v0.10\r\n";

void testTask(void *param);

TaskHandle_t startTaskHandle;

xTaskHandle pvCreatedmonitorTask;
xTaskHandle pvCreatedvShell_Task;
xTaskHandle pvCreatedtestTask;
xTaskHandle pvCreatedradiolinkTask;
xTaskHandle pvCreatedatkpTxTask;
xTaskHandle pvCreatedimuTask;
xTaskHandle pvCreatedstabilizerTask;
xTaskHandle pvCreatedsensorTask;
xTaskHandle pvCreatedultrasoundTask;
xTaskHandle pvCreatedbuttonTask;
xTaskHandle pvCreatedencoderTask;
xTaskHandle pvCreatedmotorTask;
xTaskHandle pvCreatedbmsTask;
xTaskHandle pvCreatedledTask;
xTaskHandle pvCreatedatkpRxAnlTask;
//

#define Montor_STK_SIZE           200
#define Shell_STK_SIZE            500
#define Test_STK_SIZE             50
#define Radiolink_STK_SIZE        150
#define AtkpTx_STK_SIZE           200
#define Imu_STK_SIZE              200
#define Stabilizer_STK_SIZE       200
#define Sensor_STK_SIZE           100
#define Ultrasound_STK_SIZE       150
#define Button_STK_SIZE           100
#define Encoder_STK_SIZE          300
#define Motor_STK_SIZE            64
#define Bms_STK_SIZE              200
#define Led_STK_SIZE              200
#define AtkpRxAnl_STK_SIZE        200

#define Montor_TASK_PRIO		      11
#define Shell_TASK_PRIO		        11
#define Test_TASK_PRIO		        10
#define Radiolink_TASK_PRIO		    2
#define AtkpTx_TASK_PRIO		      8
#define AtkpRxAnl_TASK_PRIO       8
#define Imu_TASK_PRIO		          4
#define Stabilizer_TASK_PRIO		  5
#define Sensor_TASK_PRIO		      8
#define Ultrasound_TASK_PRIO		  7
#define Button_TASK_PRIO		      9
#define Encoder_TASK_PRIO		      7
#define Motor_TASK_PRIO		        6
#define Bms_TASK_PRIO		          8
#define Led_TASK_PRIO		          12

//定义定时器句柄
TimerHandle_t 	AutoReloadTimer_Handle;				
void AutoReloadCallback(TimerHandle_t xTimer); 	

void vAllThredTotalSta(void)
{
	unsigned portBASE_TYPE uxHighWaterMark;
	
  #if MONITOR_DEBUG_ENABLE
	uxHighWaterMark=uxTaskGetStackHighWaterMark( pvCreatedmonitorTask );
	g_tRobotTaskState.monitor.name="monitor";
	g_tRobotTaskState.monitor.pri=Montor_TASK_PRIO;
	g_tRobotTaskState.monitor.tol=Montor_STK_SIZE;
	g_tRobotTaskState.monitor.left=uxHighWaterMark;
	g_tRobotTaskState.monitor.used=((float)(Montor_STK_SIZE-uxHighWaterMark)/Montor_STK_SIZE)*100;
	#endif
	
	uxHighWaterMark=uxTaskGetStackHighWaterMark( pvCreatedvShell_Task );
	g_tRobotTaskState.Shell.name="Shell";
	g_tRobotTaskState.Shell.pri=Shell_TASK_PRIO;
	g_tRobotTaskState.Shell.tol=Shell_STK_SIZE;
	g_tRobotTaskState.Shell.left=uxHighWaterMark;
	g_tRobotTaskState.Shell.used=((float)(Shell_STK_SIZE-uxHighWaterMark)/Shell_STK_SIZE)*100;
	//Test
	uxHighWaterMark=uxTaskGetStackHighWaterMark( pvCreatedtestTask );
	g_tRobotTaskState.test.name="Test";
	g_tRobotTaskState.test.pri=Test_TASK_PRIO;
	g_tRobotTaskState.test.tol=Test_STK_SIZE;
	g_tRobotTaskState.test.left=uxHighWaterMark;
	g_tRobotTaskState.test.used=((float)(Test_STK_SIZE-uxHighWaterMark)/Test_STK_SIZE)*100;
	//link
	uxHighWaterMark=uxTaskGetStackHighWaterMark( pvCreatedradiolinkTask );
	g_tRobotTaskState.radiolink.name="radiolink";
	g_tRobotTaskState.radiolink.pri=Radiolink_TASK_PRIO;
	g_tRobotTaskState.radiolink.tol=Radiolink_STK_SIZE;
	g_tRobotTaskState.radiolink.left=uxHighWaterMark;
	g_tRobotTaskState.radiolink.used=((float)(Radiolink_STK_SIZE-uxHighWaterMark)/Radiolink_STK_SIZE)*100;
	//atkpTx
	uxHighWaterMark=uxTaskGetStackHighWaterMark( pvCreatedatkpTxTask );
	g_tRobotTaskState.atkpTx.name="atkpTx";
	g_tRobotTaskState.atkpTx.pri=AtkpTx_TASK_PRIO;
	g_tRobotTaskState.atkpTx.tol=AtkpTx_STK_SIZE;
	g_tRobotTaskState.atkpTx.left=uxHighWaterMark;
	g_tRobotTaskState.atkpTx.used=((float)(AtkpTx_STK_SIZE-uxHighWaterMark)/AtkpTx_STK_SIZE)*100;
	//imu
	uxHighWaterMark=uxTaskGetStackHighWaterMark( pvCreatedimuTask );
	g_tRobotTaskState.imu.name="imu";
	g_tRobotTaskState.imu.pri=Imu_TASK_PRIO;
	g_tRobotTaskState.imu.tol=Imu_STK_SIZE;
	g_tRobotTaskState.imu.left=uxHighWaterMark;
	g_tRobotTaskState.imu.used=((float)(Imu_STK_SIZE-uxHighWaterMark)/Imu_STK_SIZE)*100;	
	//stabilizer
	uxHighWaterMark=uxTaskGetStackHighWaterMark( pvCreatedstabilizerTask );
	g_tRobotTaskState.stabilizer.name="stabilizer";
	g_tRobotTaskState.stabilizer.pri=Stabilizer_TASK_PRIO;
	g_tRobotTaskState.stabilizer.tol=Stabilizer_STK_SIZE;
	g_tRobotTaskState.stabilizer.left=uxHighWaterMark;
	g_tRobotTaskState.stabilizer.used=((float)(Stabilizer_STK_SIZE-uxHighWaterMark)/Stabilizer_STK_SIZE)*100;	
	//sensor
	uxHighWaterMark=uxTaskGetStackHighWaterMark( pvCreatedsensorTask );
	g_tRobotTaskState.sensor.name="sensor";
	g_tRobotTaskState.sensor.pri=Sensor_TASK_PRIO;
	g_tRobotTaskState.sensor.tol=Sensor_STK_SIZE;
	g_tRobotTaskState.sensor.left=uxHighWaterMark;
	g_tRobotTaskState.sensor.used=((float)(Sensor_STK_SIZE-uxHighWaterMark)/Sensor_STK_SIZE)*100;	
	//ultrasound
	uxHighWaterMark=uxTaskGetStackHighWaterMark( pvCreatedsensorTask );
	g_tRobotTaskState.ultrasound.name="ultrasound";
	g_tRobotTaskState.ultrasound.pri=Ultrasound_TASK_PRIO;
	g_tRobotTaskState.ultrasound.tol=Ultrasound_STK_SIZE;
	g_tRobotTaskState.ultrasound.left=uxHighWaterMark;
	g_tRobotTaskState.ultrasound.used=((float)(Ultrasound_STK_SIZE-uxHighWaterMark)/Ultrasound_STK_SIZE)*100;	
	//button
	uxHighWaterMark=uxTaskGetStackHighWaterMark( pvCreatedbuttonTask );
	g_tRobotTaskState.button.name="button";
	g_tRobotTaskState.button.pri=Button_TASK_PRIO;
	g_tRobotTaskState.button.tol=Button_STK_SIZE;
	g_tRobotTaskState.button.left=uxHighWaterMark;
	g_tRobotTaskState.button.used=((float)(Button_STK_SIZE-uxHighWaterMark)/Button_STK_SIZE)*100;	
	//encoder
	uxHighWaterMark=uxTaskGetStackHighWaterMark( pvCreatedencoderTask );
	g_tRobotTaskState.encoder.name="encoder";
	g_tRobotTaskState.encoder.pri=Encoder_TASK_PRIO;
	g_tRobotTaskState.encoder.tol=Encoder_STK_SIZE;
	g_tRobotTaskState.encoder.left=uxHighWaterMark;
	g_tRobotTaskState.encoder.used=((float)(Encoder_STK_SIZE-uxHighWaterMark)/Encoder_STK_SIZE)*100;	
	//motor
	uxHighWaterMark=uxTaskGetStackHighWaterMark( pvCreatedmotorTask );
	g_tRobotTaskState.motor.name="motor";
	g_tRobotTaskState.motor.pri=Motor_TASK_PRIO;
	g_tRobotTaskState.motor.tol=Motor_STK_SIZE;
	g_tRobotTaskState.motor.left=uxHighWaterMark;
	g_tRobotTaskState.motor.used=((float)(Motor_STK_SIZE-uxHighWaterMark)/Motor_STK_SIZE)*100;		
	//bms
	uxHighWaterMark=uxTaskGetStackHighWaterMark( pvCreatedbmsTask );
	g_tRobotTaskState.bms.name="bms";
	g_tRobotTaskState.bms.pri=Bms_TASK_PRIO;
	g_tRobotTaskState.bms.tol=Bms_STK_SIZE;
	g_tRobotTaskState.bms.left=uxHighWaterMark;
	g_tRobotTaskState.bms.used=((float)(Bms_STK_SIZE-uxHighWaterMark)/Bms_STK_SIZE)*100;	
	//led
	uxHighWaterMark=uxTaskGetStackHighWaterMark( pvCreatedledTask );
	g_tRobotTaskState.led.name="led";
	g_tRobotTaskState.led.pri=Led_TASK_PRIO;
	g_tRobotTaskState.led.tol=Led_STK_SIZE;
	g_tRobotTaskState.led.left=uxHighWaterMark;
	g_tRobotTaskState.led.used=((float)(Led_STK_SIZE-uxHighWaterMark)/Led_STK_SIZE)*100;	
}

void xTimersCreat(	const char * const pcTimerName,
								const TickType_t xTimerPeriodInTicks,
								const UBaseType_t uxAutoReload,
								void * const pvTimerID,
								TimerCallbackFunction_t pxCallbackFunction ){									
	AutoReloadTimer_Handle=xTimerCreate(pcTimerName,xTimerPeriodInTicks,uxAutoReload,(void*)pvTimerID,pxCallbackFunction); 	
	xTimerStart(AutoReloadTimer_Handle,0);
}
//周期定时器的回调函数
void AutoReloadCallback(TimerHandle_t xTimer)
{
    vAllThredTotalSta();
}
static void startTask(void *arg);
//
int main(void)
{ 
  bsp_Init();
	
	xTaskCreate(startTask, "START_TASK", 300, NULL, 2, &startTaskHandle);                                    	 /*创建起始任务*/            
  vTaskStartScheduler();	//开启任务调度  	
}
void vSystemHardwareWorkTask(void *p)
{   
    while(1)
    {
      vUsart1Interactive();// 串口1发送处理函数
			vUsart2Interactive();// 串口2发送和接受处理函数        
			vTaskDelay(5);
    }
}
/*创建任务*/
void startTask(void *arg)
{
	taskENTER_CRITICAL();                                            	                                         /*进入临界区*/
//	#if MONITOR_DEBUG_ENABLE
//	xTaskCreate(monitorTask,    "monitorTask",     Montor_STK_SIZE,      NULL, Montor_TASK_PRIO,      &pvCreatedmonitorTask);		     /*创建监测任务*/
//	#endif	
	xTaskCreate(vFrameDataHandlerTask,  "radiolinkTask",   100,   NULL, Radiolink_TASK_PRIO,   &pvCreatedradiolinkTask);	     /*创建串口接收任务*/
	
	xTaskCreate(vFrameRxPackageTask,  "atkpRxAnlTask",   200,   NULL, AtkpRxAnl_TASK_PRIO,   &pvCreatedatkpRxAnlTask);	     /*创建串口接收任务*/
	
	xTaskCreate(vFrameDataHandlerUsart1Task,  "radiolinkTask",   150,   NULL, Radiolink_TASK_PRIO,   NULL);	     /*创建串口接收任务*/
	
	xTaskCreate(vUart1FrameRxPackageTask,  "atkpRxAnlTask",   200,   NULL, AtkpRxAnl_TASK_PRIO,   NULL);	     /*创建串口接收任务*/

  xTaskCreate(encoderTask,    "encoderTask",     100,     NULL, Encoder_TASK_PRIO,     &pvCreatedencoderTask);		     /*创建里程计任务*/

	xTaskCreate(imuTask,        "imuTask",         100,         NULL, Imu_TASK_PRIO,         &pvCreatedimuTask);		         /*创建imu任务*/ 
	
	xTaskCreate(vSystemHardwareWorkTask,    "bspTask",     100,     NULL, 2,     NULL);	
	
	//printf("\r\n%scompile at %s %s\r\n\r\n", verision, __DATE__, __TIME__);
			
	vTaskDelete(startTaskHandle);										                                                           /*删除开始任务*/
	
	taskEXIT_CRITICAL();	                                                                                     /*退出临界区*/
} 

