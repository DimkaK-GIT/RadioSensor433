#ifndef __FILTER_H
#define __FILTER_H
#include "main.h"

typedef struct
{
	uint8_t count;											// ������ ������������� ����� � ���� Master
	float fillbuff[5]; 	// ����� �� �������� Master
	float result;
}filterHandle;

filterHandle filter(filterHandle data, float in);

#endif // __FILTER_H
