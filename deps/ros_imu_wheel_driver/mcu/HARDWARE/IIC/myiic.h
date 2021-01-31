#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"
#include "delay.h"


//IO��������
 
#define SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)8<<28;}
#define SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)3<<28;}

//IO��������	 
#define IIC_SCL    PBout(6) //SCL
#define IIC_SDA    PBout(7) //SDA	 
#define READ_SDA   PBin(7)  //����SDA 

//IIC���в�������
void vIIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void vIIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 ucIIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 ucIIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�
  
u8 uc_i2c_read_byte(uint8_t reg_addr,uint8_t ReadAddr);
void uc_i2c_write_byte(uint8_t reg_addr,u8 WriteAddr,u8 DataToWrite);
u8  uc_i2c_read_Len(u8 addr,u8 reg,u8 len,u8 *buf);
#endif
















