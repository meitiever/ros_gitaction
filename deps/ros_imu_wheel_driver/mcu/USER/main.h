#ifndef __MAIN_H
#define __MAIN_H
#include "sys.h"
//机器人运动线速度和角速度
typedef struct
{
    int16_t RealLineSpeed;                           
    int16_t ExpectLineSpeed;  
	  float RealAngSpeed;                           
    float ExpectAngSpeed;  //单位：rad/s
}RobotWheelState_t;
//机器人运动状态枚举
typedef enum{
    zStop      =0,           
    zForward   =1,                
    zTurnLeft  =2,          
    zTurnRight =3,          
    zRetreat   =4,          
}RobotMotionState_t;
//机器人LED状态
typedef enum{
	   zOff          =0,
	   zOn           =1,
	   zFlash_50Ms   =2, 
}LedState_t;

typedef struct{
	LedState_t state;
}LEDColorState_t;

typedef struct{
      LEDColorState_t Red;
	    LEDColorState_t White;
	    LEDColorState_t Yellow;
}RobotLedState_t;
//机器人按键状态
typedef enum{
	   zKeyNorPress = 0,
	   zKey1Down ,
	   zKey1Up  ,
		 zKey2Down ,
	   zKey2Up  ,
	   zKey1Press_Hold   ,
	   zKey2Press_Hold   ,
	   zKey12Press_Down  ,
	   zKey12Press_Up  ,
	   zKey12Press_Hold  ,
//	   zPress_1S      ,
//	   zPress_2S      ,
//	   zPress_5S      ,
//	   zPress_10S     ,
//}KeyState_t;

//typedef struct{
//	   KeyState_t key0;
//	   KeyState_t key1;	
}KeyNumberState_t;

typedef enum{
	zEnable =0,
	zDisable =1,
}SwitchState_t;
typedef enum{   
	   zNor    =0,
	   zAbn    =1,
	   zErr    =2,
}SensorState_t;
typedef struct{
	SwitchState_t switching;
	SensorState_t state;
}SensorAtt_t;
typedef struct{
	SensorAtt_t LFstate;
	SensorAtt_t LBstate;
	SensorAtt_t RFstate;
	SensorAtt_t RBstate;
}DropState_t;
typedef struct{
	SensorAtt_t state;
}DMSState_t;
typedef struct{
	SensorAtt_t Left;
	SensorAtt_t Right;
}CollisionState_t;
typedef enum
{
	BmsNor = 0,
	BmsLowPower,
	BmsCharging,
	BmsFullyCharge,
	BmsUndervoltage,
	BmsLowvoltage,
}BmsState_t;
//机器人所有信息 全局变量
typedef struct
{
	RobotWheelState_t  motion;
	RobotMotionState_t motionState;
	RobotLedState_t    led;
	KeyNumberState_t   key;
	DropState_t        drop;
	DMSState_t         DMS;
	CollisionState_t   collision;
	BmsState_t         bms;
}RobotState_t;
extern volatile RobotState_t  g_tRobotState;
extern volatile RobotWheelState_t  g_tRobotWheelState;
typedef struct 
{
	char *name;
	uint8_t pri;
	uint8_t status;
	uint16_t tol;
	uint16_t left;
	uint8_t  used;
}thread_Att_t;
typedef struct 
{
	thread_Att_t  monitor;
	thread_Att_t  Shell;
	thread_Att_t  test;
	thread_Att_t  radiolink;
	thread_Att_t  atkpTx;
	thread_Att_t  imu;
	thread_Att_t  stabilizer;
	thread_Att_t  sensor;
	thread_Att_t  ultrasound;
	thread_Att_t  button;
	thread_Att_t  encoder;
	thread_Att_t  motor;	
	thread_Att_t  bms;
	thread_Att_t  led;
}thread_t;
extern volatile thread_t  g_tRobotTaskState;

void vAllThreadStateLog(void);
#endif
