#include "hc12.h"
#include "main.h"
#include "string.h"
#include "cmsis_os.h"

extern UART_HandleTypeDef huart1;
extern uint8_t HC12Mode; 

uint8_t answerAT = 0;
char hcAnswer[20];

void modeAT (void)
{
	HAL_GPIO_WritePin(set_GPIO_Port, set_Pin, GPIO_PIN_RESET);
	osDelay(40);
	HC12Mode = 1;
}

void modeModem (void)
{
	HAL_GPIO_WritePin(set_GPIO_Port, set_Pin, GPIO_PIN_SET);
	HAL_Delay(80);
	HC12Mode = 0;
}

void sendATCommand (char* command)
{
	uint8_t commandLen = strlen(command);
	
	HAL_UART_Transmit(&huart1, (uint8_t*)command, commandLen,100);
}

void sendTest( void)
{
	char buff[10];
	sprintf(buff,"AT\n\r");
	sendATCommand(buff);
}

void setSpeed (uint16_t speed)
{
	char buff[10];
	sprintf(buff,"AT+B%d\r\n",speed);
	sendATCommand(buff);
}

void setMode (uint8_t mode)
{
	char buff[10];
	sprintf(buff,"AT+FU%d\r\n",mode);
	sendATCommand(buff);
}

void setChannel (uint8_t channel)
{
	char buff[10];
	sprintf(buff,"AT+C%03d\r\n",channel);
	sendATCommand(buff);
}

void setPower (uint8_t power)
{
	char buff[10];
	sprintf(buff,"AT+P%d\r\n",power);
	sendATCommand(buff);
}

void setSleep (void)
{
	char buff[10];
	sprintf(buff,"AT+SLEEP\r\n");
	sendATCommand(buff);
}

void modeSleep(void)
{
	uint16_t pauseCount;

	modeAT();
	answerAT = 0;
	pauseCount = 0;
	memset(hcAnswer,0,sizeof(hcAnswer));

	setSleep();
	while(answerAT == 0)
	{			
		HAL_Delay(10);
		if(++pauseCount > 500)
			break;
	}

	modeModem();
}

uint8_t initHC12(uint16_t speed, uint8_t mode, uint8_t cannel, uint8_t power)
{
		uint16_t pauseCount;
		uint8_t result = 0;	
	
		modeAT();
		answerAT = 0;
	  pauseCount = 0;
		memset(hcAnswer,0,sizeof(hcAnswer));
		sendTest();
		while(answerAT == 0)
		{			
			osDelay(10);
			if(++pauseCount > 500)
				break;
		}
		hcAnswer[2] = 0;
		if (strcmp(hcAnswer,"OK") != 0)
			return 1;
		
		
		answerAT = 0;
	  pauseCount = 0;
		memset(hcAnswer,0,sizeof(hcAnswer));
		setSpeed(speed);
		while(answerAT == 0)
		{			
			osDelay(10);
			if(++pauseCount > 500)
				break;
		}
		hcAnswer[2] = 0;
		if (strcmp(hcAnswer,"OK") != 0)
			return 1;


		answerAT = 0;
	  pauseCount = 0;
		memset(hcAnswer,0,sizeof(hcAnswer));
		setMode(mode);
		while(answerAT == 0)
		{			
			osDelay(10);
			if(++pauseCount > 500)
				break;
		}
		hcAnswer[2] = 0;
		if (strcmp(hcAnswer,"OK") != 0)
			return 1;

		HAL_Delay(10);
		answerAT = 0;
	  pauseCount = 0;
		memset(hcAnswer,0,sizeof(hcAnswer));
		setChannel(cannel);
		while(answerAT == 0)
		{			
			osDelay(10);
			if(++pauseCount > 500)
				break;
		}
		hcAnswer[2] = 0;
//		if (strcmp(hcAnswer,"OK") != 0)
//			return 1;

		HAL_Delay(10);
		answerAT = 0;
	  pauseCount = 0;
		memset(hcAnswer,0,sizeof(hcAnswer));
		setPower(power);
		while(answerAT == 0)
		{			
			osDelay(10);
			if(++pauseCount > 500)
				break;
		}
		hcAnswer[2] = 0;
//		if (strcmp(hcAnswer,"OK") != 0)
//			return 1;

		modeModem();
		return result; 
}