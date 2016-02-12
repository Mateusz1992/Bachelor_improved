/*
 * BluetoothDriver.c
 *
 *  Created on: 08-02-2016
 *      Author: Mateusz
 */
#include "BluetoothDriver.h"
#include <string.h>

#define TRUE 1
#define FALSE 0

//conditions for UART HRDWARE Flow Control
#define START_ASKING 0
#define STOP_ASKING 1
//conditions for UART HRDWARE Flow Control

char device[20];

bool initStatus = FALSE; //if initialised
bool removeInitTimerRx = FALSE;
bool removeMainTimerStatus = FALSE;
bool removeRxTimerStatus = FALSE;
bool canSend = TRUE;
bool canSendStream = FALSE;


//main timer for initialisation
handle_t mainInitTimerID = 0;
uint32_t mainInitTimerStatus = SYSTM001_ERROR;
//main timer for initialisation

//timer for receiving answers from commands
handle_t HandlTimerReceiveAnsID = 0;
uint32_t StatusReceiveAns = SYSTM001_ERROR;
//timer for receiving answers from commands

//timer for receiving messages
handle_t TimerIdRxMessage = 0;
uint32_t TimerStatusRxMessage = SYSTM001_ERROR;
//timer for receiving messages

//structure representing commands and answers
commandsAndAnswers initCommandsAnswer;
//structure representing commands and answers

//indicator of connection stage
int stageOfConnection = 0;
//indicator of connection stage

//in case of failure for other files to inform that failure occured
extern bool connectionFailure = FALSE;
//in case of failure for other files to inform that failure occured

extern int turnedOnSentTimer;

int stageOfInit = 0;


void chooseDevice(char *dev)
{
	if(!strcmp(dev, "LSM9DS1"))
	{
		device[0] = 'L';
		device[1] = 'S';
		device[2] = 'M';
		device[3] = '9';
		device[4] = 'D';
		device[5] = 'S';
		device[6] = '1';
		device[7] = '\0';
	}
}

void resetConnection(void)
{
	removeReceiveTimer();
	createMainTimer();
}

void checkErrorOccurence(char *allMsg)
{
	if(strstr(allMsg, "+") != NULL)
	{
		if(strstr(allMsg, "+RDII") != NULL)//in case of disconnection
		{
			stageOfConnection = 0;
			stageOfInit = 0;
			initStatus = FALSE;
			removeInitTimerRx = FALSE;
			connectionFailure = TRUE;
			resetConnection();
		}
	}
}

void manageConnection(void)
{
	if(getStatusMainTimer())
	{
		removeMainTimer();
		setStatusMainTimer(FALSE);
		stageOfConnection = 1;
	}

	if(1 == stageOfConnection)
	{
		TimerIdRxMessage = SYSTM001_CreateTimer(200,SYSTM001_PERIODIC,RxTimerHandler,NULL);
		SYSTM001_StartTimer(TimerIdRxMessage);
		turnedOnSentTimer = 0;
		stageOfConnection = 0;
	}
}

void copyCheckAnswers(void)
{
	strcpy(initCommandsAnswer.answersBluetooth[0], "ROK\r\n");
	strcpy(initCommandsAnswer.answersBluetooth[1], "OK\r\n");
	strcpy(initCommandsAnswer.answersBluetooth[2], "OK\r\n");
	strcpy(initCommandsAnswer.answersBluetooth[3], "OK\r\n");
	strcpy(initCommandsAnswer.answersBluetooth[4], "OK\r\n");
	strcpy(initCommandsAnswer.answersBluetooth[5], "OK\r\n");
	strcpy(initCommandsAnswer.answersBluetooth[6], "OK\r\n");
	strcpy(initCommandsAnswer.answersBluetooth[7], "OK\r\n");
}

void copyCommands(void)
{
	strcpy(initCommandsAnswer.commandsBluetooth[0], "AT+JRES\r\n");
	strcpy(initCommandsAnswer.commandsBluetooth[1], "AT+JSEC=4,1,04,1111,0,0\r\n");
	strcpy(initCommandsAnswer.commandsBluetooth[2], "AT+JSLN=07,Chmura1\r\n");
	strcpy(initCommandsAnswer.commandsBluetooth[3], "AT+JRLS=32,13,0000110100001000800000805F9B34FB,Serial port 1,01,C20200\r\n");
	strcpy(initCommandsAnswer.commandsBluetooth[4], "AT+JAAC=1\r\n");
	strcpy(initCommandsAnswer.commandsBluetooth[5], "AT+JDIS=3\r\n");
	strcpy(initCommandsAnswer.commandsBluetooth[6], "AT+JGPC=FFFD,0000,0000,0000,FFFD\r\n");
	strcpy(initCommandsAnswer.commandsBluetooth[7], "AT+JSCR\r\n");
}

bool getStatusMainTimer(void)
{
	return removeMainTimerStatus;
}

void setStatusMainTimer(int status)
{
	removeMainTimerStatus = status;
}

void removeMainTimer(void)
{
	SYSTM001_StopTimer(mainInitTimerID);
	SYSTM001_DeleteTimer(mainInitTimerID);
	mainInitTimerID = 0;
}

void removeReceiveTimer(void)
{
	SYSTM001_StopTimer(TimerIdRxMessage);
	SYSTM001_DeleteTimer(TimerIdRxMessage);
	TimerIdRxMessage = 0;
}

void createMainTimer(void)
{
	mainInitTimerID = SYSTM001_CreateTimer(1000,SYSTM001_PERIODIC,timerHandlerInitBluetooth,&initCommandsAnswer);
	mainInitTimerStatus = SYSTM001_StartTimer(mainInitTimerID);
}

void createReceiveTimer(void)
{
	TimerIdRxMessage = SYSTM001_CreateTimer(200,SYSTM001_PERIODIC,RxTimerHandler,NULL);
	SYSTM001_StartTimer(TimerIdRxMessage);
}

void timerHandlerInitBluetooth(void *T)
{
	static uint8_t whichCommand = 0;

	commandsAndAnswers *temp = (commandsAndAnswers*)T;

	if(removeInitTimerRx == TRUE)
	{
		SYSTM001_DeleteTimer(HandlTimerReceiveAnsID);
		HandlTimerReceiveAnsID = 0;
		removeInitTimerRx = FALSE;
		whichCommand = stageOfInit;
	}

	if(TRUE == initStatus)
	{
		removeMainTimerStatus = TRUE;
	}

	if(FALSE == initStatus && canSend == TRUE)
	{
		//whichCommand = stageOfInit;
		volatile int length = strlen(temp->commandsBluetooth[whichCommand]);

		send(temp->commandsBluetooth[whichCommand], length);

		while((UART001_GetFlagStatus(&UART001_Handle0,UART001_TRANSFER_STATUS_BUSY_FLAG )==UART001_SET))
		{
			//int h = 0;
		}

		canSend = FALSE;
		askAboutSending();
		HandlTimerReceiveAnsID = SYSTM001_CreateTimer(300,SYSTM001_ONE_SHOT,timerHandlerInitBluetoothReceive,NULL);
		SYSTM001_StartTimer(HandlTimerReceiveAnsID);
	}
}


void timerHandlerInitBluetoothReceive(void *T)
{
	char* answer = (char*)read();
	//delayBt(10000);
	//askAboutSending();

	if(answer != NULL)
	{
		if(checkFunction((unsigned char*)answer, (unsigned char*)initCommandsAnswer.answersBluetooth[stageOfInit]))
		{
			stageOfInit++;
		}
		free(answer);
		answer = NULL;
		if(8 == stageOfInit)
		{
			stageOfInit = 0;
			initStatus = TRUE;
		}
	}
	canSend = TRUE;
	removeInitTimerRx = TRUE;
}

void RxTimerHandler(void *T)
{
	char* answer = (char*)read();
	if(answer != NULL){
		checkErrorOccurence(answer);
		chooseDevice(answer);
		free(answer);
		answer = NULL;
	}
}

void initBluetooth(void)
{
	initFlowControl();
	copyCheckAnswers();
	copyCommands();

	createMainTimer();
}

void send(char* command1, int length)
{
	static int criticValue = 0;
	if(!statusRtsFlowControl())
	{
		stopAskingAboutSending();
	}

	USIC_CH_TypeDef* UartRegs = UART001_Handle0.UartRegs;
	uint32_t WriteCount = 0;

	while(length)
	{

		if(!(USIC_CH_TCSR_TDV_Msk & UartRegs->TCSR))
		{
			if(!checkStatusCts())
			{
				char c = command1[WriteCount];

				UartRegs->TBUF[0] = c;
				length--;
				WriteCount++;
			}
			else
			{
				criticValue++;

				if(criticValue > 3)
				{
					criticValue = 0;
					break;
				}
			}
		}

		if(UART001_GetFlagStatus(&UART001_Handle0,UART001_TRANS_BUFFER_IND_FLAG) == UART001_SET)
		{
			UART001_ClearFlag(&UART001_Handle0,UART001_TRANS_BUFFER_IND_FLAG);
		}
	}
}




unsigned char* read(void)
{
	askAboutSending();
	unsigned char *readBuffer = NULL;
	int statusRxFifoFilling = 0;
	int howManyRead = 0;

	statusRxFifoFilling = USIC_GetRxFIFOFillingLevel(UART001_0_USIC_CH);

	if(((UART001_GetFlagStatus(&UART001_Handle0,UART001_FIFO_STD_RECV_BUF_FLAG)) == UART001_SET))
	{

		statusRxFifoFilling = USIC_GetRxFIFOFillingLevel(UART001_0_USIC_CH);

		if(statusRxFifoFilling != 0)
		{
			readBuffer = (unsigned char *)malloc((sizeof(unsigned char)*statusRxFifoFilling) + 1);

			howManyRead =  UART001_ReadDataBytes(&UART001_Handle0, readBuffer, statusRxFifoFilling);

			readBuffer[statusRxFifoFilling] = '\0';
		}

		UART001_ClearFlag(&UART001_Handle0,UART001_FIFO_STD_RECV_BUF_FLAG);

		while(!USIC_ubIsRxFIFOempty(UART001_0_USIC_CH))
		{
			USIC_FlushRxFIFO(UART001_0_USIC_CH);
		}
	}

	stopAskingAboutSending();

	return readBuffer;
}

//UART Hardware Flow Control Functions
void askAboutSending(void)
{
	IO004_SetOutputValue(IO004_Handle0, START_ASKING);
}

void stopAskingAboutSending(void)
{
	IO004_SetOutputValue(IO004_Handle0, STOP_ASKING);
}

int statusRtsFlowControl(void)
{
	return IO004_ReadPin(IO004_Handle0);
}

int checkStatusCts(void)
{
	return IO004_ReadPin(IO004_Handle1);
}
//UART Hardware Flow Control Functions

int checkFunction(unsigned char *answer, unsigned char *readBuffer)
{
	if(strcmp((char*)answer, (char*)readBuffer) == 0)
	{
		return 1;
	}
	return 0;
}

int connectStatus(void)
{
	return IO004_ReadPin(IO004_Handle2);
}

void initFlowControl(void)
{
	//stopAskingAboutSending();
	askAboutSending();
}

void delayBt(uint32_t d)
{
	for(uint32_t i = 0; i < d; i++)
	{
		;
	}
}
