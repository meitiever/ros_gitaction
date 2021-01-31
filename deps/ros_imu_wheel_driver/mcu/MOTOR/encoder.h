#ifndef __ENCODER_H
#define __ENCODER_H

#include "sys.h" 

void getWheelEncoderData(int16_t* get0,int16_t* get1);
void getRobotWheelData(int16_t* linSpeed,float* AngSpeed);
void encoderTask(void *param);

#endif
