#ifndef PROTOCOL__H_
#define PROTOCOL__H_

#include "sys.h"
#include "stdbool.h"
#include "string.h"



#define FRAME_DATA_MAX_LENGTH   (50)//֡�����ֽ���󳤶�

#define FRAME_HEAD_POS         (0)//֡ͷ�ֽ���һ֡�е�λ��
#define FRAME_FUNCTION_POS     (1)//֡�����ֽ���һ֡�е�λ��
#define FRAME_SEQUENCE_POS     (2)//֡�����ֽ���һ֡�е�λ��
#define FRAME_DATA_LENGTH_POS  (3)//֡���ݳ����ֽ���һ֡�е�λ��
#define FRAME_DATA_POS         (4)//֡�����ֽ���һ֡�е�λ��

#define UP_BYTE1 0xAA /*����֡ͷ*/
#define UP_BYTE2 0x55

#define DOWN_BYTE1 0xAA /*����֡ͷ*/
#define DOWN_BYTE2 0xAA
//20201109���ݽṹ
#define Yesense_ID_ACC              0X10      //12
#define Yesense_ID_EULER            0X40      //16
#define Yesense_ID_SAMPLE_TIME      0X51      //4
#define Yesense_ID_SYNCOUT_TIME     0X52      //4
#define Yesense_ID_SPEED            0X60      //12
typedef struct {
	uint8_t YS_Header1;
	uint8_t YS_Header2;
	uint16_t TID;
	uint8_t LEN;
	uint8_t data[50];
	uint8_t CK1;
	uint8_t CK2;
}Yesense_t;
/*���ֽ� ֡����ö��*/
typedef enum
{
    UpdateFirmwareStart  =0XC0,//��ʼ�̼���������
    UpdateFirmwareDone   =0XC1,//�̼������������
    BehaviorControl      =0xC2,//��Ϊ��������
    RequestState         =0xC3,//����״̬����
    RouteCoord           =0xC4,//��·��������
}fCommand_e;
/*�����ֽ�ö��*/
typedef enum
{
    ffRespond=0xb0,//��Ӧ
    ffCommand=0xb1,//����
    ffData   =0xb2,//����
    ffIdle   =0xb3,//����
    ffMax,//�����ֽڽ�β
}FrameFunction_e;
/* ����֡״̬ */
typedef enum
{
	WaitForStartByte1,
	WaitForStartByte2,
	WaitForMsgID,
	WaitForMsgID2,
	WaitForPid,
	WaitForDataLength,
	WaitForData,
	WaitForChksum1,
}FrameState_e;

/*֡��������ö��*/
typedef enum
{
    FrameFromatErr=0x1,//��ʽ����
    FrameCheckErr =0x2,//У�����
    FrameSeqErr   =0x3,//���д���
    FrameLengthErr=0x4,//���ȴ���
    FrameSuccess  =0x5,//�޴���
}FrameErr_e;
/*ͨѶ���ݽṹ*/
typedef struct
{
	u8 MsgID;
	u8 Pid;
	u8 DataLen;
	u8 Data[FRAME_DATA_MAX_LENGTH];
}FramePackage_t;
/*֡�ṹ��*/
__packed  typedef struct
{
    u8 FrameHead;//֡ͷ �̶�Ϊ0XAA
    FrameFunction_e FrameFunction;//֡���ܱ�ʶ�ֽ�
    u8 FrameSequence;//֡���� 0-255 ѭ����1
    u8 DataLength;//֡���ݳ��� 0-50
    u8 *FrameData;//֡���ݵ�ַ
    u8 FrameChecksum;//֡У��
}Frame_t;

/*֡���ռ�¼*/
__packed typedef struct
{
    Frame_t tFrame;//֡��Ϣ
    u8 DataBuf[FRAME_DATA_MAX_LENGTH];//֡����
    u8 NextFrameSeq;//�´�Ӧ�õ�����
    
    u8 ContinueErrNum;//�����������
    bool IsStartReceive;//�Ƿ�ʼ����
    bool IsFrameNotProcess;//֡���ݻ�δ����
    FrameErr_e FrameErrType;//֡��������
}Rx_FrameRecord;


/*֡���ͼ�¼*/
__packed typedef struct
{
    Frame_t tFrame;//֡��Ϣ
    u8 DataBuf[FRAME_DATA_MAX_LENGTH];//֡����
    u8 NextFrameSeq;//�´�Ӧ�õ�����
    bool IsSendOut;//�Ƿ�����һ֡���ݣ�
    bool IsReceiveRespond;//�Ƿ���ܵ���Ӧ
}Tx_FrameRecord;

extern Rx_FrameRecord FR_Rx;
extern Tx_FrameRecord FR_Tx;

bool bJudgeChecksum(u8 *buf,u8 length);
u8 ucCodeChecksum(u8 *buf,u8 length);

FrameErr_e eFrame_Analy(u8 *buf,u8 length,Rx_FrameRecord *RFR);

u8 ucCode_RespondFrame(Tx_FrameRecord *Tx,Rx_FrameRecord *Rx,u8 *buf);
u8 ucCode_CommandFrame(Tx_FrameRecord *Tx,u8 *Commandbuf,u8 length,u8 *buf);
u8 ucCode_DataFrame(Tx_FrameRecord *Tx,u8 *Databuf,u8 length,u8 *buf);
u8 ucCode_IdleFrame(Tx_FrameRecord *Tx,u8 *buf);

void Sensor_Encoder_Send(void);
void Sensor_Collision_Send(void);
void Sensor_Send(void);
void Sensor_Camer_Send(void);
void Sensor_bmi088_Send(float ax,float ay,float az,float gx ,float gy,float gz,float t);
void atkpReceiveAnl(Rx_FrameRecord *anlPacket);
#endif

