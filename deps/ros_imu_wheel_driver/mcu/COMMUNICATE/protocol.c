#include "protocol.h"
#include "bsp.h"
/*FreeRtos includes*/
#include "FreeRTOS.h" 
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "usart1.h"	  


extern u32 time_test ;
#if 1
Rx_FrameRecord FR_Rx={0};
Tx_FrameRecord FR_Tx={0};
/*
* Function Name  : eFrame_Analy
* Description    : 将数据BUF解析为帧数据
* Input          : buf 接收原始数据 length 数据长度 RFR 接收帧结构体
* Output         : None 
* Return         : FrameErr_e 错误类型
*/
FrameErr_e eFrame_Analy(u8 *buf,u8 length,Rx_FrameRecord *RFR)
{
	  u8 cksum = 0 , dataIndex = 0;
    u16 i;
    FrameErr_e FErr = FrameFromatErr;
    FrameState_e FState = WaitForStartByte1;
	  //printf("analysis is start ,len=%d \r\n",length);
    for(i=0;i<length;i++)
    {  
			switch(FState)
			{
				case WaitForStartByte1 :
					FState = (buf[i] == DOWN_BYTE1) ? WaitForStartByte2 : WaitForStartByte1;
					cksum = buf[i];
				  break;
				case WaitForStartByte2 :
					FState = (buf[i] == DOWN_BYTE2) ? WaitForMsgID : WaitForStartByte1;
					cksum += buf[i];
				  break;
				case WaitForMsgID :
					RFR->tFrame.FrameFunction = buf[i];
					FState = WaitForData;
					cksum += buf[i];
				  dataIndex = 0;
				  break;
				case WaitForData:
					RFR->DataBuf[dataIndex] = buf[i];
					dataIndex++;
					cksum += buf[i];
					if (dataIndex == 12) FState = WaitForChksum1;
					break;
				case WaitForChksum1:
					  FState = WaitForStartByte1;
				    return FrameSuccess;
					break;
				default:
					break;
			}
		}
		RFR->ContinueErrNum++;
    RFR->FrameErrType=FErr;
		
    return FErr;
}
#include "math.h"
#include "main.h"
#include "encoder.h"
#include "main.h"
#include "motor.h" 
extern 			uint16_t encoder1 ,encoder2 ,direct1 ,direct2 ;
			
short yaw,pitch,roll,x,y,z;
TickType_t xIMU_Uart_Time ,xIMU_Send_Time,xEncoder_Time;			
static Yesense_t anlPacket;
u16 Ysense_Serial_Num =0;



float dis_x=0,dis_y=0;
float last_ang =0,now_ang =0;
float ang_dt = 0;
float lineSpeed =0;

u32 left_encoder_cnt_last =0;
u32 right_encoder_cnt_last =0;
u16 left_encoder_dt =0,right_encoder_dt =0;
s16 dir_left = 0,dir_right = 0;
extern u32 Left_encoder_cnt ,Right_cncoder_cnt ;


extern float bmi088_ax , bmi088_ay , bmi088_az ;
extern float bmi088_gx , bmi088_gy , bmi088_gz ;
extern int16_t bmi088_TEMP ;
uint8_t imu_buff[47];
uint8_t encoder_buff[37];
uint8_t bmi088_buff[53];
uint8_t	sendbuf[137];
void encoder_dis_cal(void)
{
	float now_ang=(float)yaw/100;
  ang_dt =now_ang-last_ang;
	if(ang_dt<0) ang_dt =-ang_dt;
	last_ang = now_ang;
	lineSpeed =(0.83650)*((left_encoder_dt+right_encoder_dt)/2);	
	u8 time =1;
	if(now_ang>0&&now_ang<90) {dis_x+=lineSpeed*time*cos(ang_dt);dis_y+=lineSpeed*time*sin(ang_dt); }
	else if(now_ang>90&&now_ang<180) {dis_x+=-lineSpeed*time*cos(ang_dt);dis_y+=lineSpeed*time*sin(ang_dt); }
	else if(now_ang>-90&&now_ang<0) {dis_x+=lineSpeed*time*cos(ang_dt);dis_y+= -lineSpeed*time*sin(ang_dt); }
	else if(now_ang>-180&&now_ang<-90) {dis_x+=-lineSpeed*time*cos(ang_dt);dis_y+=-lineSpeed*time*sin(ang_dt); }
	else {}
}
extern u8 imu_flasgs;
void atkpReceiveAnl(Rx_FrameRecord *anlPacket)
{
	//short yaw,pitch,roll,x,y,z
	#if 1
	yaw=((u16)anlPacket->DataBuf[1]<<8)|anlPacket->DataBuf[0];  
	pitch = (anlPacket->DataBuf[3]<<8)|anlPacket->DataBuf[2];
	roll = (anlPacket->DataBuf[5]<<8)|anlPacket->DataBuf[4];
	
	x=(anlPacket->DataBuf[7]<<8)|anlPacket->DataBuf[6];
	y=(anlPacket->DataBuf[9]<<8)|anlPacket->DataBuf[8];
	z=(anlPacket->DataBuf[11]<<8)|anlPacket->DataBuf[10];
	if( imu_flasgs == 1){
	xIMU_Uart_Time = xTaskGetTickCount();
	Sensor_Send();
	left_encoder_dt =Left_encoder_cnt-left_encoder_cnt_last; left_encoder_cnt_last =Left_encoder_cnt;
	right_encoder_dt =Right_cncoder_cnt-right_encoder_cnt_last;right_encoder_cnt_last = Right_cncoder_cnt;
	Sensor_Encoder_Send();
	encoder_dis_cal();
	lineSpeed =0.83650*((left_encoder_dt+left_encoder_dt)/2);
	Sensor_bmi088_Send(bmi088_ax*9.8/1000,bmi088_ay*9.8/1000,bmi088_az*9.8/1000,bmi088_gx*3.14/180,bmi088_gy*3.14/180,bmi088_gz*3.14/180,bmi088_TEMP);
	PC13 = !PC13;
//	for(int send_c =0;send_c <137;send_c++)sendbuf[send_c] =0 ;
//	for(int i =0;i<47;i++)	sendbuf[i]=imu_buff[i];
//	for(int i =0;i<37;i++)	sendbuf[i+46]=imu_buff[i];	
//	for(int i =0;i<53;i++)	sendbuf[i+46+36]=imu_buff[i];	
//	uarts3SendDataDmaBlocking(137,sendbuf);
	}
	#endif
//	encoder_di	s_cal();
//	printf("yaw=%f,%f,%f,%f,%f,%f \r\n",(float)yaw/100,(float)pitch/100,(float)roll/100,(float)x/100,(float)y/100,(float)z/100);
} 

void Sensor_IMU_Data_Make(void)
{
	anlPacket.data[0]=0x10;
	anlPacket.data[1]=0x0c;
	u32 temp;
	temp = x *10000;
	
	anlPacket.data[2]=(uint8_t)temp; //acc_x
	anlPacket.data[3]=(uint8_t)(temp>>8);
	anlPacket.data[4]=(uint8_t)(temp>>16);
	anlPacket.data[5]=(uint8_t)(temp>>24);

	temp = y *10000;
	anlPacket.data[6]=(uint8_t)temp; //acc_y
	anlPacket.data[7]=(uint8_t)(temp>>8);
	anlPacket.data[8]=(uint8_t)(temp>>16);
	anlPacket.data[9]=(uint8_t)(temp>>24);
	temp = z *10000;
	anlPacket.data[10]=(uint8_t)temp; //acc_z
	anlPacket.data[11]=(uint8_t)(temp>>8);
	anlPacket.data[12]=(uint8_t)(temp>>16);
	anlPacket.data[13]=(uint8_t)(temp>>24);
	//欧拉角
	anlPacket.data[14]=0x40;
	anlPacket.data[15]=0x0c;
	temp = yaw *10000; //
	anlPacket.data[16]=(uint8_t)temp; //pitch
	anlPacket.data[17]=(uint8_t)(temp>>8);
	anlPacket.data[18]=(uint8_t)(temp>>16);
	anlPacket.data[19]=(uint8_t)(temp>>24);
	temp = pitch *10000;
	anlPacket.data[20]=(uint8_t)temp; //roll
	anlPacket.data[21]=(uint8_t)(temp>>8);
	anlPacket.data[22]=(uint8_t)(temp>>16);
	anlPacket.data[23]=(uint8_t)(temp>>24);
	temp = roll *10000;
	anlPacket.data[24]=temp; //yaw
	anlPacket.data[25]=(uint8_t)(temp>>8);
	anlPacket.data[26]=(uint8_t)(temp>>16);
	anlPacket.data[27]=(uint8_t)(temp>>24);
	//采样时间
	anlPacket.data[28]=0x51;
	anlPacket.data[29]=0x04;
	//temp = xIMU_Uart_Time *1000;
	temp = 0;
	anlPacket.data[30]=temp; //采样时间
	anlPacket.data[31]=(uint8_t)(temp>>8);
	anlPacket.data[32]=(uint8_t)(temp>>16);
	anlPacket.data[33]=(uint8_t)(temp>>24);
	
//	xIMU_Send_Time = xTaskGetTickCount();
//	temp = xIMU_Send_Time *1000;
	temp =time_test;
	//printf("time= %d \r\n", temp);	
	//输出同步时间
	anlPacket.data[34]=0x52;
	anlPacket.data[35]=0x04;
	
	anlPacket.data[36]=temp; //输出同步时间
	anlPacket.data[37]=(uint8_t)(temp>>8);
	anlPacket.data[38]=(uint8_t)(temp>>16);
	anlPacket.data[39]=(uint8_t)(temp>>24);
}
void Sensor_Send(void)
{
	uint8_t buff[47];

	anlPacket.YS_Header1 = 0x59;
	anlPacket.YS_Header2 = 0x53;
	if(Ysense_Serial_Num>60000) Ysense_Serial_Num =0;
	
	anlPacket.TID =Ysense_Serial_Num;
	Ysense_Serial_Num++;
	anlPacket.LEN = 0x00;
	Sensor_IMU_Data_Make();
	buff[0]=anlPacket.YS_Header1;
	buff[1]=anlPacket.YS_Header2;
	buff[2]=anlPacket.TID>>8;
	buff[3]=anlPacket.TID;
	buff[4]=40;
	for(int i=0;i<40;i++)
	buff[5+i]=anlPacket.data[i];
	

	uint8_t CK1 = 0 ,CK2 = 0;
	for(int i=2;i<45;i++)
	{
	 CK1 = CK1 + buff[i];
	 CK2 = CK2 + CK1;
	}
	buff[45]=CK1;
	buff[46]=CK2;
  for(int cnt =0 ;cnt<47 ;cnt++)imu_buff[cnt] = buff[cnt];

	uarts3SendDataDmaBlocking(47,buff);
}

extern u16 left_encoder_dt ,right_encoder_dt ;
static u16 test_left_encoder_cnt,test_right_encoder_cnt ,test_encoder_cnt1,test_encoder_cnt2;

extern u8 uart1_Collision_Left  ,  uart1_Collision_Right , uart1_encoder_left_dir , uart1_encoder_right_dir;
extern u16 uart1_encoder_left , uart1_encoder_right ;

void Sensor_Encoder_Send(void)
{
	#define abs(x) (x>0?x:-x)
	uint8_t buff[37];
		  int16_t MonitorWheelLeftEncoder=0;
		  int16_t MonitorWheelRightEncoder=0;
		  getWheelEncoderData(&MonitorWheelLeftEncoder,&MonitorWheelRightEncoder);

	
	anlPacket.YS_Header1 = 0x59;
	anlPacket.YS_Header2 = 0x53;
	if(Ysense_Serial_Num>60000) Ysense_Serial_Num =0;
	anlPacket.TID =Ysense_Serial_Num;
	Ysense_Serial_Num++;
	anlPacket.LEN = 0x00;

	
	buff[0]=anlPacket.YS_Header1;
	buff[1]=anlPacket.YS_Header2;
	buff[2]=anlPacket.TID>>8;
	buff[3]=anlPacket.TID;
	buff[4]=30;
	
	anlPacket.data[0]=0x72;
	anlPacket.data[1]=0x10;
//	uint16_t temp3= left_encoder_dt;uart1_Collision_Left
	uint16_t temp3= uart1_encoder_left;
	test_left_encoder_cnt = temp3;
	anlPacket.data[2]= temp3;
	anlPacket.data[3]=(temp3>>8)&0XFF;
	anlPacket.data[4]= 0 ;
  anlPacket.data[5]= 0 ;
	if(g_tLeftWheel.Direction ==WheelForward) direct1 =1;
	else if(g_tLeftWheel.Direction ==WheelRetreat) direct1 =0;
	test_encoder_cnt1 = direct1;
	anlPacket.data[6]= uart1_encoder_left_dir;
	anlPacket.data[7]=0;
	anlPacket.data[8]=0;
	anlPacket.data[9]=0;
	uint16_t temp4= uart1_encoder_right;
	test_right_encoder_cnt = temp4;
	anlPacket.data[10]= temp4;
	anlPacket.data[11]=(temp4>>8)&0XFF ;
	anlPacket.data[12]= 0 ;
	anlPacket.data[13]= 0 ;
//		printf("sdsd:%d,%d,%x,%x \r\n",temp3,temp4,anlPacket.data[10],anlPacket.data[11]);
  if(g_tRightWheel.Direction ==WheelForward) direct2 =1;
	else if(g_tRightWheel.Direction ==WheelRetreat) direct2 =0;
	test_encoder_cnt2 = direct2;
	anlPacket.data[14]= uart1_encoder_right_dir;
	anlPacket.data[15]=0;
	anlPacket.data[16]=0;
	anlPacket.data[17]=0;
	u32 temp;
	//采样时间
	anlPacket.data[18]=0x51;
	anlPacket.data[19]=0x04;
	
	//temp = xEncoder_Time *1000;
	temp =0;
	anlPacket.data[20]=temp; //采样时间
	anlPacket.data[21]=(uint8_t)(temp>>8);
	anlPacket.data[22]=(uint8_t)(temp>>16);
	anlPacket.data[23]=(uint8_t)(temp>>24);
	
//	xIMU_Send_Time = xTaskGetTickCount();
//	temp = xIMU_Send_Time *1000;
  temp =time_test;
  //输出同步时间
	anlPacket.data[24]=0x52;
	anlPacket.data[25]=0x04;
	
	anlPacket.data[26]=(uint8_t)(temp); //输出同步时间
	anlPacket.data[27]=(uint8_t)(temp>>8);
	anlPacket.data[28]=(uint8_t)(temp>>16);
	anlPacket.data[29]=(uint8_t)(temp>>24);
	
	for(int i=0;i<0+30;i++)
	buff[5+i]=anlPacket.data[i];

	uint8_t CK1 = 0 ,CK2 = 0;
	for(int i=2;i<2+30+3;i++)
	{
	 CK1 = CK1 + buff[i];
	 CK2 = CK2 + CK1;
	}
	buff[35]=CK1;
	buff[36]=CK2;
	
	char buf[100];
//	sprintf(buf,"en:%d,%d,%d,%d,%f,%f\r\n",left_encoder_dt ,direct1 ,right_encoder_dt ,direct2 ,dis_x,dis_y);
//	u8 buf[50];
//	//sprintf(buf,"en:%d,%d,%d,%d ,%d\r\n",test_left_encoder_cnt ,test_encoder_cnt1 ,test_right_encoder_cnt ,test_encoder_cnt2 ,temp );
//	sprintf(buf,"en:%d,%d,%d,%d ,%d\r\n",buff[7] ,buff[11] ,buff[15] ,buff[19],temp );
////	vUart1_SendString(buf,strlen(buf));
//	uarts1SendDataDmaBlocking(strlen(buf),buf);
	//printf("en:%d,%d,%d,%d ,%d\r\n",buff[7] ,buff[11] ,buff[15] ,buff[19] ,temp);
	//printf("en:%d,%d,%d,%d ,%d\r\n",buff[7] ,buff[8] ,buff[9] ,buff[10] ,temp);

	for(int cnt =0 ;cnt<37 ;cnt++)encoder_buff[cnt] = buff[cnt];
	uarts3SendDataDmaBlocking(37,buff);
}
extern u8 Collision_Left  , Collision_Right ;

void Sensor_Collision_Send(void)
{
	uint8_t buff[37];
	
	anlPacket.YS_Header1 = 0x59;
	anlPacket.YS_Header2 = 0x53;
	if(Ysense_Serial_Num>60000) Ysense_Serial_Num =0;
	anlPacket.TID =Ysense_Serial_Num;
	Ysense_Serial_Num++;
	anlPacket.LEN = 0x00;

	
	buff[0]=anlPacket.YS_Header1;
	buff[1]=anlPacket.YS_Header2;
	buff[2]=anlPacket.TID>>8;
	buff[3]=anlPacket.TID;
	buff[4]=22;
	
	anlPacket.data[0]=0x73;
	anlPacket.data[1]=0x08;
	uint16_t temp3= uart1_Collision_Left;
	anlPacket.data[2]= temp3;
	anlPacket.data[3]=(temp3>>8)&0XFF;
	anlPacket.data[4]= 0 ;
  anlPacket.data[5]= 0 ;
  uint16_t temp4= uart1_Collision_Right;
	test_right_encoder_cnt = temp4;
	anlPacket.data[6]= temp4;
	anlPacket.data[7]=(temp4>>8)&0XFF ;
	anlPacket.data[8]=0;
	anlPacket.data[9]=0;
	
	u32 temp;
	//采样时间
	anlPacket.data[10]=0x51;
	anlPacket.data[11]=0x04;
	temp =0;
	anlPacket.data[12]=temp; //采样时间
	anlPacket.data[13]=(uint8_t)(temp>>8);
	anlPacket.data[14]=(uint8_t)(temp>>16);
	anlPacket.data[15]=(uint8_t)(temp>>24);
	
  temp =time_test;
  //输出同步时间
	anlPacket.data[16]=0x52;
	anlPacket.data[17]=0x04;
	
	anlPacket.data[18]=(uint8_t)(temp); //输出同步时间
	anlPacket.data[19]=(uint8_t)(temp>>8);
	anlPacket.data[20]=(uint8_t)(temp>>16);
	anlPacket.data[21]=(uint8_t)(temp>>24);
	
	for(int i=0;i<0+22;i++)
	buff[5+i]=anlPacket.data[i];

	uint8_t CK1 = 0 ,CK2 = 0;
	for(int i=2;i<2+22+3;i++)
	{
	 CK1 = CK1 + buff[i];
	 CK2 = CK2 + CK1;
	}
	buff[27]=CK1;
	buff[28]=CK2;
	
	uarts3SendDataDmaBlocking(29,buff);
}
void Sensor_Camer_Send(void)
{
	uint8_t buff[37];
	
	anlPacket.YS_Header1 = 0x59;
	anlPacket.YS_Header2 = 0x53;
	if(Ysense_Serial_Num>60000) Ysense_Serial_Num =0;
	anlPacket.TID =Ysense_Serial_Num;
	Ysense_Serial_Num++;
	anlPacket.LEN = 0x00;

	
	buff[0]=anlPacket.YS_Header1;
	buff[1]=anlPacket.YS_Header2;
	buff[2]=anlPacket.TID>>8;
	buff[3]=anlPacket.TID;
	buff[4]=22;
	
	anlPacket.data[0]=0x74;
	anlPacket.data[1]=0x06;
	
	u32 temp;
	
  temp =time_test;
  //输出同步时间
	anlPacket.data[2]=0x52;
	anlPacket.data[3]=0x04;
	
	anlPacket.data[4]=(uint8_t)(temp); //输出同步时间
	anlPacket.data[5]=(uint8_t)(temp>>8);
	anlPacket.data[6]=(uint8_t)(temp>>16);
	anlPacket.data[7]=(uint8_t)(temp>>24);
	
	for(int i=0;i<0+8;i++)
	buff[5+i]=anlPacket.data[i];

	uint8_t CK1 = 0 ,CK2 = 0;
	for(int i=2;i<2+8+3;i++)
	{
	 CK1 = CK1 + buff[i];
	 CK2 = CK2 + CK1;
	}
	buff[13]=CK1;
	buff[14]=CK2;
	
	uarts3SendDataDmaBlocking(15,buff);
}

void Sensor_bmi088_Send(float ax,float ay,float az,float gx ,float gy,float gz,float t)
{
	uint8_t buff[53];
	
	anlPacket.YS_Header1 = 0x59;
	anlPacket.YS_Header2 = 0x53;
	if(Ysense_Serial_Num>60000) Ysense_Serial_Num =0;
	anlPacket.TID =Ysense_Serial_Num;
	Ysense_Serial_Num++;
	anlPacket.LEN = 0x00;

	
	buff[0]=anlPacket.YS_Header1;
	buff[1]=anlPacket.YS_Header2;
	buff[2]=anlPacket.TID>>8;
	buff[3]=anlPacket.TID;
	buff[4]=22;
	
	anlPacket.data[0]=0x10;
	anlPacket.data[1]=0x0c;
	u32 temp;
	s32 temp1;
	
	temp1 = ax *1000000;
	
	anlPacket.data[2]=temp1; //acc_x
	anlPacket.data[3]=(temp1>>8);
	anlPacket.data[4]=(temp1>>16);
	anlPacket.data[5]=(uint8_t)(temp1>>24);

	temp1 = ay *1000000;
	anlPacket.data[6]=temp1; //acc_y
	anlPacket.data[7]=(temp1>>8);
	anlPacket.data[8]=(temp1>>16);
	anlPacket.data[9]=(temp1>>24);
	
//	s16 data = az * 100;
//	u16 dd = data>>8|(uint8_t)data;
	temp1 = az *1000000;
	anlPacket.data[10]=temp1; //acc_z
	anlPacket.data[11]=(temp1>>8);
	anlPacket.data[12]=(temp1>>16);
	anlPacket.data[13]=(temp1>>24);

//	printf("ax =%f ,%d ,%x,%x,%x,%x\r\n",az,temp1,anlPacket.data[10],anlPacket.data[11],anlPacket.data[12],anlPacket.data[13]);
	//欧拉角
	anlPacket.data[14]=0x40;
	anlPacket.data[15]=0x0c;
	temp1 = gx *1000000;
	anlPacket.data[16]=temp1; //pitch
	anlPacket.data[17]=(temp1>>8);
	anlPacket.data[18]=(temp1>>16);
	anlPacket.data[19]=(temp1>>24);
	temp1 = gy *1000000;
	anlPacket.data[20]=temp1; //roll
	anlPacket.data[21]=(temp1>>8);
	anlPacket.data[22]=(temp1>>16);
	anlPacket.data[23]=(temp1>>24);
	temp1 = gz *1000000;
	anlPacket.data[24]=temp; //yaw
	anlPacket.data[25]=(temp1>>8);
	anlPacket.data[26]=(temp1>>16);
	anlPacket.data[27]=(temp1>>24);
	
	anlPacket.data[28]=0x53;
	anlPacket.data[29]=0x04;
	temp = t;
	anlPacket.data[30]=temp; //采样时间
	anlPacket.data[31]=(uint8_t)(temp>>8);
	anlPacket.data[32]=(uint8_t)(temp>>16);
	anlPacket.data[33]=(uint8_t)(temp>>24);
	
	
	anlPacket.data[34]=0x51;
	anlPacket.data[35]=0x04;
	temp = 0;
	anlPacket.data[36]=temp; //采样时间
	anlPacket.data[37]=(uint8_t)(temp>>8);
	anlPacket.data[38]=(uint8_t)(temp>>16);
	anlPacket.data[39]=(uint8_t)(temp>>24);

	
	//输出同步时间
	anlPacket.data[40]=0x52;
	anlPacket.data[41]=0x04;
	temp =time_test;
	anlPacket.data[42]=temp; //输出同步时间
	anlPacket.data[43]=(uint8_t)(temp>>8);
	anlPacket.data[44]=(uint8_t)(temp>>16);
	anlPacket.data[45]=(uint8_t)(temp>>24);

	
	for(int i=0;i<0+46;i++)
	buff[5+i]=anlPacket.data[i];

	uint8_t CK1 = 0 ,CK2 = 0;
	for(int i=2;i<2+46+3;i++)
	{
	 CK1 = CK1 + buff[i];
	 CK2 = CK2 + CK1;
	}
	buff[51]=CK1;
	buff[52]=CK2;
	for(int cnt =0 ;cnt<37 ;cnt++)bmi088_buff[cnt] = buff[cnt];
	uarts3SendDataDmaBlocking(53,buff);
}

#else
#endif
