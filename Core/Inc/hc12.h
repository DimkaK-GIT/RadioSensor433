#ifndef HC12_H
#define HC12_H

#define SPEED1200 1200
#define SPEED2400 2400
#define SPEED4800 4800
#define SPEED9600 9600
#define SPEED19200 19200
#define SPEED38400 38400
#define SPEED57600 57600
#define SPEED115200 115200

#define FU1 1
#define FU2 2
#define FU3 3
#define FU4 4

#define P1 1 //-1 dBM (0,8 mW)
#define P2 2 //+2 dBM (1,6 mW)
#define P3 3 //+5 dBM (3,2 mW)
#define P4 4 //+8 dBM (6,3 mW)
#define P5 5 //+11 dBM (12 mW)
#define P6 6 //+14 dBM (25 mW)
#define P7 7 //+17 dBM (50 mW)
#define P8 8 //+20 dBM (100 mW)

#include <stdint.h>
#include "string.h"

void modeAT (void);
void modeModem (void);
void sendATCommand (char* command);
void sendTest( void);
void setSpeed (uint16_t speed);
void setMode (uint8_t mode);
void setChannel (uint8_t channel);
void setPower (uint8_t power);
void setSleep (void);
void modeSleep(void);
uint8_t initHC12(uint16_t speed, uint8_t mode, uint8_t cannel, uint8_t power);
			
#endif // HC12_H
