#include "BMI088_TASK.h"
#include "BMI088.h"
#include "delay.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "read_sensor_data.h"
#include "myiic.h"
#include "protocol.h"
#include "bsp_gpio.h"    

float bmi088_ax = 0, bmi088_ay = 0, bmi088_az = 0;
float bmi088_gx = 0, bmi088_gy = 0, bmi088_gz = 0;
int16_t bmi088_TEMP =0;
extern u8 bmi088_imu_flasgs;
/*传感器任务*/
void imuTask(void *param)
{
	static portTickType xLastWakeTime;  
  const portTickType xFrequency = pdMS_TO_TICKS(10);
  xLastWakeTime = xTaskGetTickCount();   
	drv_BMI088();
	if(BMI088.isConnection())
		{
				BMI088.initialize();
				printf("BMI088 is connected \r\n");
		}
		else printf("BMI088 is not connected \r\n");
	vTaskDelay(100);
	while (1)
	{
		BMI088.getAcceleration(&bmi088_ax, &bmi088_ay, &bmi088_az);
    BMI088.getGyroscope(&bmi088_gx, &bmi088_gy, &bmi088_gz);
	  bmi088_TEMP = BMI088.getTemperature();
		//if(bmi088_imu_flasgs == 1)
			//{PC13 = !PC13;}
//		Sensor_bmi088_Send(ax*9.8/1000,ay*9.8/1000,az*9.8/1000,gx*3.14/180,gy*3.14/180,gz*3.14/180,TEMP);
//		printf("raw data : %0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f ,%d \r\n",ax*9.8/1000,ay*9.8/1000,az*9.8/1000,gx*3.14/180,gy*3.14/180,gz*3.14/180,TEMP);
		
		vTaskDelayUntil( &xLastWakeTime,xFrequency );  
				//vTaskDelay(10);
	}	
}
