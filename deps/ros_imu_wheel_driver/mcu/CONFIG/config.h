#ifndef __CONFIG_H
#define __CONFIG_H

#include "stdio.h"	/*printf ����*/


#define DEG2RAD		0.017453293f	
#define RAD2DEG		57.29578f		

#define P_NAME "Sweeping robot MCU"
#define HAEDVER "V0.0.1"
#define SOFTVER "V0.0.1"

#define SENSOR_IMU_DEBUG_ENABLE      1                     //IMU�̵߳�����Ϣ����                        0���رգ�1����
#define MONITOR_DEBUG_ENABLE         1                     //����̵߳�����Ϣ����                       0���رգ�1����
#define ATKPT_UART2_DEBUG_ENABLE     1                     //����2ͨ�ſ���                              0���رգ�1����
#define KEY_DEBUG_ENABLE             1                     //�����̵߳�����Ϣ����                       0���رգ�1����
#define USMART_DEBUG_ENABLE          0                     //Shell�߳���������                          0���رգ�1����
#define SHELL_DEBUG_ENABLE           1                     //Shell�߳���������                          0���رգ�1����
#define SENSOR_DEBUG_ENABLE          1                     //�����������߼���Ϣ����                     0���رգ�1����
#define BMS_BQ24773_DEBUG_ENABLE     1                     //��ص�Դ������Ϣ����                       0���رգ�1����

#if MONITOR_DEBUG_ENABLE                                   //����򿪼���߳̿���                       
#define MONITOR_PERIOD_IMU_RAE       0                     //���ڼ��IMUԭʼ���ݿ���                    0���رգ�1����
#define MONITOR_PERIOD_STATUS        1                     //���ڼ��״̬��Ϣ����                       0���رգ�1����
#define MONITOR_PERIOD_ENCODER       0                     //���ڼ��������������̼Ʒ����ٶȿ���       0���رգ�1����
#define MONITOR_PERIOD_SRNSOR        0                     //���ڼ���رߴ���������                     0���رգ�1����
#define MONITOR_PERIOD_MOTOR_CURRENT 1                     //���ڼ�����е����������                   0���رգ�1����
#endif

#endif /* __CONFIG_H */
