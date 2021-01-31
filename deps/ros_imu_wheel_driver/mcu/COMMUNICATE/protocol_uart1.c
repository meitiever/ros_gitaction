#include "protocol_uart1.h"
#include "protocol.h"
#include "bsp.h"
#include "FreeRTOS.h" 
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "usart1.h"	  

Rx_FrameRecord FR_uart1_Rx={0};
Tx_FrameRecord FR_uart1_Tx={0};
/*
* Function Name  : eFrame_Analy
* Description    : 将数据BUF解析为帧数据
* Input          : buf 接收原始数据 length 数据长度 RFR 接收帧结构体
* Output         : None 
* Return         : FrameErr_e 错误类型
*/
FrameErr_e eFrame_uart1_Analy(u8 *buf,u8 length,Rx_FrameRecord *RFR)
{
	  u8 cksum = 0 , dataIndex = 0;
    u16 i;
    FrameErr_e FErr = FrameFromatErr;
    FrameState_e FState = WaitForStartByte1;
    for(i=0;i<length;i++)
    {  
			switch(FState)
			{
				case WaitForStartByte1 :
					FState = (buf[i] == 0x59) ? WaitForStartByte2 : WaitForStartByte1;
					cksum = buf[i];
				  break;
				case WaitForStartByte2 :
					FState = (buf[i] == 0x53) ? WaitForMsgID : WaitForStartByte1;
					cksum += buf[i];
				  break;
				case WaitForMsgID :
					RFR->tFrame.FrameFunction = buf[i];
					FState = WaitForMsgID2;
					cksum += buf[i];
				  break;
			  case WaitForMsgID2 :
					RFR->tFrame.FrameFunction = buf[i];
					FState = WaitForDataLength;
					cksum += buf[i];
				  break;
			  case WaitForDataLength :
					RFR->tFrame.DataLength = buf[i];
					FState = WaitForData;
					cksum += buf[i];
				  dataIndex = 0;
				  break;
				case WaitForData:
					RFR->DataBuf[dataIndex] = buf[i];
					dataIndex++;
					cksum += buf[i];
					if (dataIndex == RFR->tFrame.DataLength) FState = WaitForChksum1;
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
u8 uart1_Collision_Left  ,  uart1_Collision_Right , uart1_encoder_left_dir , uart1_encoder_right_dir;
u16 uart1_encoder_left , uart1_encoder_right ;

void uart1ReceiveAnl(Rx_FrameRecord *anlPacket)
{
	switch(anlPacket->DataBuf[0])
	{
		case 0x73:
			uart1_Collision_Left = anlPacket->DataBuf[2];
		  uart1_Collision_Right = anlPacket->DataBuf[6];
			break;
		
		case 0x72:
			uart1_encoder_left = anlPacket->DataBuf[3]<<8|anlPacket->DataBuf[2];
		  uart1_encoder_left_dir = anlPacket->DataBuf[6];
		  uart1_encoder_right = anlPacket->DataBuf[11]<<8|anlPacket->DataBuf[10];
		  uart1_encoder_right_dir = anlPacket->DataBuf[14];
			break;
		default :
			
		break;
	}static int ccn=0;
	if(ccn>=9){
	printf("%x,%x,%x,%x,%x,%x \r\n",anlPacket->DataBuf[0],anlPacket->DataBuf[1],anlPacket->DataBuf[2],anlPacket->DataBuf[3],anlPacket->DataBuf[4],anlPacket->DataBuf[5]);
		ccn =0;
	}
	ccn ++;
	#if 0
	yaw=((u16)anlPacket->DataBuf[1]<<8)|anlPacket->DataBuf[0];  
	pitch = (anlPacket->DataBuf[3]<<8)|anlPacket->DataBuf[2];
	roll = (anlPacket->DataBuf[5]<<8)|anlPacket->DataBuf[4];
	x=(anlPacket->DataBuf[7]<<8)|anlPacket->DataBuf[6];
	y=(anlPacket->DataBuf[9]<<8)|anlPacket->DataBuf[8];
	z=(anlPacket->DataBuf[11]<<8)|anlPacket->DataBuf[10];
	}
	#endif
} 
void encoder_open(void)
{
	char buf[11] ={0x59,0x53,0x10,0x04,0x16,0x74,0x54,0x01,0x1,0x5c,0x69};
	vUart1_SendString(buf,11);
}
void encoder_close(void)
{
	char buf[11] ={0x59,0x53,0x10,0x04,0x16,0x74,0x54,0x01,0x0,0x5c,0x69};
	vUart1_SendString(buf,11);
}