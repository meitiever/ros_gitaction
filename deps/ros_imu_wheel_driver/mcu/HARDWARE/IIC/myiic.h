#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"
#include "delay.h"


//IO方向设置
 
#define SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)8<<28;}
#define SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)3<<28;}

//IO操作函数	 
#define IIC_SCL    PBout(6) //SCL
#define IIC_SDA    PBout(7) //SDA	 
#define READ_SDA   PBin(7)  //输入SDA 

//IIC所有操作函数
void vIIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void vIIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 ucIIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 ucIIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号
  
u8 uc_i2c_read_byte(uint8_t reg_addr,uint8_t ReadAddr);
void uc_i2c_write_byte(uint8_t reg_addr,u8 WriteAddr,u8 DataToWrite);
u8  uc_i2c_read_Len(u8 addr,u8 reg,u8 len,u8 *buf);
#endif
















