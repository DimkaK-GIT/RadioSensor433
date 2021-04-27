#ifndef TYPES_H
#define TYPES_H

#include "main.h"

typedef struct Parameter
      {
				uint8_t ADR_DST;             		//адрес назначения (приемника)
        uint8_t ADR_SRC;             		//адрес свой в системе (передатчик)
				uint8_t temp1;									// выравнивание по 32 бита
				uint8_t temp2;
				uint64_t ID;										//ID прибора
				
      }structParameterHeader; 

typedef struct 
      {
				uint8_t sensor;             		//адрес назначения (приемника)
        uint8_t countSensor;            //адрес свой в системе (передатчик)
				uint8_t prevSensor;							// выравнивание по 32 бита
				uint8_t change;
				GPIO_TypeDef * SensorPort;
				uint16_t SensorPin;
      }structSensorHeader; 
#endif // TYPES_H
