#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"

#define prd    500
#define Vbreak 800

#define PWMPeriod   500

#define Motor_R_Forward  0
#define Motor_L_Forward  0
#define Motor_R_Back     1
#define Motor_L_Back     1

#define MotorSpdZero     0		

#define motor_R_Forward     PEout(4)=1
#define motor_R_Reverse     PEout(4)=0
#define motor_L_Forward     PEout(7)=1
#define motor_L_Reverse     PEout(7)=0

#define   WHEEL_PERIMETER           (220)   //轮子周长 按照l=3.14*7 计算，单位mm 
#define   WHEEL_ENCODE_COUNT        (263)   //编码器计数个数  //255-265测试
#define   SINGLE_ENCODE_DISTANCE    (83650) 
#define   WHEEL_SPACEING            (220)   //左右驱动轮间距 单位mm
   
#define EncodeNumber 5 

typedef enum{
    WheelRetreat = 0,
    WheelForward ,
    WheelStop,
}WheelDirection_e;

/*驱动轮结构体*/
typedef struct{
    
    uint16_t usEncodeBuffer[EncodeNumber];         
    uint16_t usHistory;                                                 
    uint16_t usNew;                                       
    uint8_t ucPoint;                             
    WheelDirection_e Direction;                   
    uint16_t TimeOut;                                
    int16_t RealSpeed;                           
    int16_t ExpectSpeed;                          
    int16_t SoftSpeed;                           
    int32_t iSignalCount;                         
    
}WheelSruct_t;

/*轮子结构体*/
extern WheelSruct_t g_tLeftWheel ;
extern WheelSruct_t g_tRightWheel ;

#endif
