#ifndef HDLC_H
#define HDLC_H

#include <stdint.h>
#include "string.h"

#define MAX_HDLC_TX_BUFFER 200  				// размер бувера на передачу по мастеру
#define MAX_HDLC_RX_BUFFER 200  				//размер бувера на прием по мастеру

typedef struct HDLC_Header
      {
       unsigned char ADR_DST;             		//Адрес назначения посылки по Master
       unsigned char ADR_SRC;             		//Адрес отправителя посылки по Master
      }struct_HDLC_Header; 

	  
typedef struct
			{
				uint16_t count_TX_Master;											// индекс передаваемого байта в порт Master
				uint16_t TX_Master_Max;     									// количество передаваемых байт в порт Master
				uint16_t Master_TX_Flag;                      // флаг передачи нанных в Master
				uint8_t master_buff_tx[MAX_HDLC_TX_BUFFER]; 	// буфер на передачу Master
			}interfaceHDLC_TX;

typedef struct
			{
				uint16_t count_RX;														// количество принятых байт
				uint8_t RX_StartFlag;                         // флаг передачи нанных в Master
				uint8_t RX_7EFlag;														// флаг приема байта 7E
				uint8_t RX_7DFlag;														// флаг приема байта 7D
				uint8_t ADR_DSC;															// адрес получателя сообщения
				uint8_t ADR_SRC;															// адрес источника сообщения
				uint8_t Command;															// принятая команда.
				uint8_t arg[MAX_HDLC_RX_BUFFER];						 	// аргументы принятой команды
				uint8_t master_buff_rx[MAX_HDLC_RX_BUFFER]; 	// буфер на прием Master
				uint16_t argCount;														// количество принятых аргументов
			}interfaceHDLC_RX;



// прототипы функций:
void recHDLC (uint8_t data);
unsigned int send_Master_HDLC (struct_HDLC_Header MasterHDLC,char Command, char* data, int count_data);			
uint16_t CRC16(uint8_t * data, int count);
			
#endif // HDLC_H
