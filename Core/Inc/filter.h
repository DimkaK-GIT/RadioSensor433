#ifndef __FILTER_H
#define __FILTER_H
#include "main.h"

typedef struct
{
	uint8_t count;											// индекс передаваемого байта в порт Master
	float fillbuff[5]; 	// буфер на передачу Master
	float result;
}filterHandle;

filterHandle filter(filterHandle data, float in);

#endif // __FILTER_H
