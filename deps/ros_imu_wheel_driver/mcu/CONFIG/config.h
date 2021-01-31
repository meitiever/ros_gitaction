#ifndef __CONFIG_H
#define __CONFIG_H

#include "stdio.h"	/*printf 调用*/


#define DEG2RAD		0.017453293f	
#define RAD2DEG		57.29578f		

#define P_NAME "Sweeping robot MCU"
#define HAEDVER "V0.0.1"
#define SOFTVER "V0.0.1"

#define SENSOR_IMU_DEBUG_ENABLE      1                     //IMU线程调试信息开关                        0：关闭，1：打开
#define MONITOR_DEBUG_ENABLE         1                     //监测线程调试信息开关                       0：关闭，1：打开
#define ATKPT_UART2_DEBUG_ENABLE     1                     //串口2通信开关                              0：关闭，1：打开
#define KEY_DEBUG_ENABLE             1                     //按键线程调试信息开关                       0：关闭，1：打开
#define USMART_DEBUG_ENABLE          0                     //Shell线程启动开关                          0：关闭，1：打开
#define SHELL_DEBUG_ENABLE           1                     //Shell线程启动开关                          0：关闭，1：打开
#define SENSOR_DEBUG_ENABLE          1                     //传感器驱动逻辑信息开关                     0：关闭，1：打开
#define BMS_BQ24773_DEBUG_ENABLE     1                     //电池电源管理信息开关                       0：关闭，1：打开

#if MONITOR_DEBUG_ENABLE                                   //如果打开监测线程开关                       
#define MONITOR_PERIOD_IMU_RAE       0                     //周期监测IMU原始数据开关                    0：关闭，1：打开
#define MONITOR_PERIOD_STATUS        1                     //周期监测状态信息开关                       0：关闭，1：打开
#define MONITOR_PERIOD_ENCODER       0                     //周期监测左右驱动轮里程计反馈速度开关       0：关闭，1：打开
#define MONITOR_PERIOD_SRNSOR        0                     //周期监测沿边传感器数据                     0：关闭，1：打开
#define MONITOR_PERIOD_MOTOR_CURRENT 1                     //周期监测所有电机电流数据                   0：关闭，1：打开
#endif

#endif /* __CONFIG_H */
