#ifndef __DIRECT_H
#define __DIRECT_H

#include "sys.h" 

typedef enum{
    DirectNor = 0,
    DirectFirstRise,
    DirectFirstFall,
	  DirectSecondRise,
	  DirectSecondFall,
	  DirectComplete
}DirectState_t;

typedef uint8_t (*GPIO_ReadInput)(void);
typedef struct{
	GPIO_ReadInput ReadExit;
	unsigned char id;
	DirectState_t state;
}EncoderDirectState_t;
#endif
