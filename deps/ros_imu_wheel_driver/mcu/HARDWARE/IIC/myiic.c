#include "myiic.h"

//初始化IIC
void vIIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	//使能GPIOB时钟
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7); 	//PB6,PB7 输出高
}
//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;   
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
	delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 ucIIC_Wait_Ack(void)
{
  u8 ucErrTime=0;
  SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void vIIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        //IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
			IIC_SDA=1;
		else
			IIC_SDA=0;
		txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 ucIIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}

u8 uc_i2c_read_byte(uint8_t reg_addr,uint8_t ReadAddr)
{				  
	u8 temp=0;		  	    																 
  IIC_Start();  
  vIIC_Send_Byte(reg_addr<<1|0);   //发送器件地址0XD4,伪写数据 	 
	ucIIC_Wait_Ack(); 
  vIIC_Send_Byte(ReadAddr);   //发送寄存器地址
	ucIIC_Wait_Ack();	 
	IIC_Stop();
	IIC_Start();  	 	    
	vIIC_Send_Byte(reg_addr<<1|1);           //进入接收模式			   
	ucIIC_Wait_Ack();	 
  temp=ucIIC_Read_Byte(0);		   
  IIC_Stop();//产生一个停止条件	    
	return temp;
}
void uc_i2c_write_byte(uint8_t reg_addr,u8 WriteAddr,u8 DataToWrite)
{				   	  	    																 
  IIC_Start();  
	vIIC_Send_Byte(reg_addr<<1|0);   //发送器件地址写数据 
	ucIIC_Wait_Ack();	   
  vIIC_Send_Byte(WriteAddr);   //发送低地址
	ucIIC_Wait_Ack(); 	 										  		   
	vIIC_Send_Byte(DataToWrite);     //发送字节							   
	ucIIC_Wait_Ack();  		    	   
  IIC_Stop();//产生一个停止条件 

}
u8  uc_i2c_read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	IIC_Start(); 
	vIIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
  ucIIC_Wait_Ack();
    vIIC_Send_Byte(reg);	//写寄存器地址
    ucIIC_Wait_Ack();		//等待应答
    IIC_Start();
	vIIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
    ucIIC_Wait_Ack();		//等待应答 
	while(len)
	{
		if(len==1)*buf=ucIIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=ucIIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
    IIC_Stop();	//产生一个停止条件 
	return 0;	
}
























