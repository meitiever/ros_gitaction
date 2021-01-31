#include "encoder.h"
#include "motor.h" 
#include "main.h"

#include "bsp.h"
#include "config.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"

u32 Left_encoder_cnt = 0,Right_cncoder_cnt =0;
#if 1
void __attribute__((used)) TIM1_CH1_Capture_Callback()
{
	Left_encoder_cnt ++;
}
void __attribute__((used)) TIM1_CH2_Capture_Callback()
{
  Right_cncoder_cnt ++;
}
#else 
void __attribute__((used)) TIM1_CH1_Capture_Callback()
{
	  g_tLeftWheel.TimeOut=0;
		g_tLeftWheel.usNew = TIM_GetCapture1( TIM1 );  

		if( g_tLeftWheel.usNew < g_tLeftWheel.usHistory )
		{
				g_tLeftWheel.usEncodeBuffer[g_tLeftWheel.ucPoint] = g_tLeftWheel.usNew+TIM1_ARR-g_tLeftWheel.usHistory+1;
		}
		else
		{
				g_tLeftWheel.usEncodeBuffer[g_tLeftWheel.ucPoint] = g_tLeftWheel.usNew-g_tLeftWheel.usHistory;
		} 
    g_tLeftWheel.ucPoint = (g_tLeftWheel.ucPoint+1 ==  EncodeNumber)? 0 : (g_tLeftWheel.ucPoint+1);
    g_tLeftWheel.usHistory = g_tLeftWheel.usNew;		
}
void __attribute__((used)) TIM1_CH2_Capture_Callback()
{
		g_tRightWheel.TimeOut=0;        
		g_tRightWheel.usNew = TIM_GetCapture2( TIM1 );  

		if( g_tRightWheel.usNew < g_tRightWheel.usHistory )
		{
				g_tRightWheel.usEncodeBuffer[g_tRightWheel.ucPoint] = g_tRightWheel.usNew+TIM1_ARR-g_tRightWheel.usHistory+1;
		}
		else
		{
				g_tRightWheel.usEncodeBuffer[g_tRightWheel.ucPoint] = g_tRightWheel.usNew-g_tRightWheel.usHistory;
		} 
		g_tRightWheel.ucPoint = (g_tRightWheel.ucPoint+1 ==  EncodeNumber)? 0 : (g_tRightWheel.ucPoint+1);
    g_tRightWheel.usHistory = g_tRightWheel.usNew;
}
#endif
 uint16_t usGetMiddleValue( uint16_t *pusBuffer, uint8_t ucLength )
{
    uint8_t i;
    uint16_t usMiddle,usMax,usCount = 0,usPoint;
    
    while( usCount <= (ucLength/2) )
    {
        usPoint = 0;
        usMax = pusBuffer[0];
        for( i=1;i<ucLength;i++ ) 
        {      
            if( pusBuffer[i] > usMax ) 
            {
                usMax = pusBuffer[i];
                usPoint = i;
            }                           
        }
        usMiddle = usMax;
        pusBuffer[usPoint] = 0;   
        usCount++;       
    }
    
    return usMiddle;
}
void vDataMemoryCopy( uint16_t *pusTarget, uint16_t *pusSource, uint8_t ucLength )
{
    uint8_t i;
    
    for(i=0;i<ucLength;i++)
    {
        pusTarget[i] = pusSource[i];
    }
    
}
int16_t usGetWheelSpeed( uint16_t usBuffer[], WheelDirection_e Direction ) 
{ 

    int16_t usSpeed;     
    uint16_t usArray[EncodeNumber];
        
    vDataMemoryCopy( usArray, usBuffer, EncodeNumber );               
    usSpeed = SINGLE_ENCODE_DISTANCE/usGetMiddleValue( usArray, EncodeNumber );

    if( Direction == WheelRetreat )
    {
        usSpeed = -usSpeed;
    }
    else if( Direction == WheelStop )
    {
        usSpeed = 0;
    }
    
    return usSpeed;
}
int16_t usGetWheelEncoder( uint16_t usBuffer[])
{
	  int16_t usSpeed;     
    uint16_t usArray[EncodeNumber];
        
    vDataMemoryCopy( usArray, usBuffer, EncodeNumber );               
    usSpeed = usGetMiddleValue( usArray, EncodeNumber );
	
	return usSpeed;
}
uint16_t encoder1 =0,encoder2 =0,direct1 =0,direct2 =0;
extern TickType_t xEncoder_Time;	
extern short yaw;
#include "protocol.h"
#include "math.h"
#include "protocol_uart1.h"

extern u8 test_flags1 ;
extern u8 test_flags2 ;
extern u8 test_flags[10];
extern u8 test_cnt ;
extern u8 g_flag ;
extern u8 g_val ;

u8 g_val_flags =0;
u8 imu_flasgs = 0;
u8 bmi088_imu_flasgs = 0;
void encoderTask(void *param)
{
	static uint32_t camera_cnt = 0;
//	delay_ms(1000);
	static portTickType xLastWakeTime;  
  const portTickType xFrequency = pdMS_TO_TICKS(2*50);  
   
    // 使用当前时间初始化变量xLastWakeTime ,注意这和vTaskDelay()函数不同 
    xLastWakeTime = xTaskGetTickCount(); 
    while(1)
    {
			//g_val_flags =1;imu_flasgs =1;bmi088_imu_flasgs = 1;
			if(g_flag ==1)
			{
				for(int i=0;i<12;i++) printf("%x,",test_flags[i]);
				if(test_flags[0]==0x59&&test_flags[1]==0x53&&test_flags[2]==0x10&&test_flags[3]==0x04&&test_flags[4]==0x16&&test_flags[5]==0x74&&test_flags[6]==0x54&&test_flags[7]==0x01)
				{
				g_val = test_flags[8];
				printf(":g_val=%d \r\n",g_val);
				if(g_val ==1){delay_ms(1000);g_val_flags =1;imu_flasgs =1;bmi088_imu_flasgs = 1; encoder_open();}
				else if(g_val ==0) {g_val_flags =0;imu_flasgs =0;bmi088_imu_flasgs = 0; encoder_close();}
				}					

				  test_flags1 = 0;
				  test_flags2 = 0;
				  for(int o =0;o<12;o++)test_flags[0] =0;
				  test_cnt = 0;
				  g_flag = 0;
				  g_val = 0;
					camera_cnt++;				
			}
			else {camera_cnt =0;}
			
			if(g_val_flags ==1){PB2=1;Sensor_Camer_Send();delay_ms(1);PB2= 0;Sensor_Collision_Send();}
			else if(0){}						
			//delay_ms(50);
			vTaskDelayUntil( &xLastWakeTime,xFrequency );  

    }        
}

void getWheelEncoderData(int16_t* get0,int16_t* get1)
{
	*get0 = g_tLeftWheel.RealSpeed ;
	*get1 = g_tRightWheel.RealSpeed;
}
void getRobotWheelData(int16_t* linSpeed,float* AngSpeed)
{
	*linSpeed = g_tRobotState.motion.RealLineSpeed ;
	*AngSpeed = g_tRobotState.motion.RealAngSpeed ;
}






