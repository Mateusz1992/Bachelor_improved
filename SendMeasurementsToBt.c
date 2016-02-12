/*
 * SendMeasurementsToBt.c
 *
 *  Created on: 12-02-2016
 *      Author: Mateusz
 */
#include "CleanArray.h"
#include "LSM9DS1_Driver.h"
#include "BluetoothDriver.h"
#include "SendMeasurementsToBt.h"
#include <DAVE3.h>
#include <string.h>

char device[20]; //tablica w ktorej przechowujemy wybrany sensor
int turnedOn = 0; //czy ktorys timer jest wlaczony

char xAxis;
char yAxis;
char zAxis;

bool connectionFailure; //jesli wystapil problem z polaczeniem bluetooth to trzeba usunac timer, zmienna wskazujaca

handle_t TimerIdSentMsg; //id timera do
int turnedOnSentTimer;

void sendMeasurementsToBt(void)
{
	if(!strcmp(device, "LSM9DS1"))
	{
		readAndSendMeasurements(NULL);
		xAxis = getAccelX() + 48;
		yAxis = getAccelY() + 48;
		zAxis = getAccelZ() + 48;

		if(0 == turnedOnSentTimer)
		{
			TimerIdSentMsg = SYSTM001_CreateTimer(50,SYSTM001_PERIODIC,sendMsgLSM9DS1,NULL);
			SYSTM001_StartTimer(TimerIdSentMsg);
			turnedOnSentTimer = 1;
		}

		if(1 == turnedOnSentTimer)
		{
			if(TRUE == connectionFailure)
			{
				connectionFailure = FALSE;
				SYSTM001_StopTimer(TimerIdSentMsg);

				SYSTM001_DeleteTimer(TimerIdSentMsg);

				TimerIdSentMsg = 0;
				cleanArray();

			}
		}
	}
}


void sendMsgLSM9DS1(void *T)
{
	static int i = 0;

	if(0 == i){
		send(&xAxis, 1);
		i++;
	}
	else if(1 == i)
	{
		send(&yAxis, 1);
		i++;
	}
	else if(2 == i)
	{
		send(&zAxis, 1);
		i = 0;
	}
}
