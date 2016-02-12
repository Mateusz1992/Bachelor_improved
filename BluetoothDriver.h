/*
 * BluetoothDriver.h
 *
 *  Created on: 08-02-2016
 *      Author: Mateusz
 */

#ifndef BLUETOOTHDRIVER_H_
#define BLUETOOTHDRIVER_H_

#include <DAVE3.h>
#include <stdlib.h>

typedef struct commandsAndAnswers
{
	char commandsBluetooth[8][100];
	char answersBluetooth[8][20];
}commandsAndAnswers;

void chooseDevice(char *dev);

void resetConnection(void);

void checkErrorOccurence(char *allMsg);

void manageConnection(void);

void copyCheckAnswers(void);

void copyCommands(void);

bool getStatusMainTimer(void);

void setStatusMainTimer(int status);

void removeMainTimer(void);

void removeReceiveTimer(void);

void createMainTimer(void);

void createReceiveTimer(void);

void timerHandlerInitBluetooth(void *T);

void timerHandlerInitBluetoothReceive(void *T);

void RxTimerHandler(void *T);

void initBluetooth(void);

void send(char* command1, int length);

unsigned char* read(void);

//UART Hardware Flow Control Functions
void askAboutSending(void);

void stopAskingAboutSending(void);

int statusRtsFlowControl(void);

int checkStatusCts(void);
//UART Hardware Flow Control Functions

int checkFunction(unsigned char *answer, unsigned char *readBuffer);

int connectStatus(void);

void initFlowControl(void);

void delayBt(uint32_t d);

#endif /* BLUETOOTHDRIVER_H_ */
