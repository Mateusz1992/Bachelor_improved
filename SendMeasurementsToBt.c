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

char xAxis[10]/*[50]*/;
char yAxis[10]/*[50]*/;
char zAxis[10]/*[50]*/;

char xAxisf[30]/*[50]*/;
char yAxisf[30]/*[50]*/;
char zAxisf[30]/*[50]*/;

bool connectionFailure; //jesli wystapil problem z polaczeniem bluetooth to trzeba usunac timer, zmienna wskazujaca

handle_t TimerIdSentMsg; //id timera do
int turnedOnSentTimer;
char copied;
accel copiedData[100];

void sendMeasurementsToBt(void)
{
	static i = 0;
	if(!strcmp(device, "LSM9DS1"))
	{
		if(0 == i){
			startMeasurements();
			++i;
		}
		readAndSendMeasurements(NULL);


		if(getAccelX() >= 0)
		{
			xAxis[0] = 'x';
			xAxis[1] = getAccelX() + 48;
			xAxis[2] = '\0';

			sprintf(xAxisf, "x%.2f", getAccelXf());
		}
		else
		{
			xAxis[0] = 'x';
			xAxis[1] = '-';
			xAxis[2] = abs(getAccelX()) + 48;
			xAxis[3] = '\0';

			sprintf(xAxisf, "x%.2f", getAccelXf());
		}

		if(getAccelY() >= 0)
		{
			yAxis[0] = 'y';
			yAxis[1] = getAccelY() + 48;
			yAxis[2] = '\0';

			sprintf(yAxisf, "y%.2f", getAccelYf());
		}
		else
		{
			yAxis[0] = 'y';
			yAxis[1] = '-';
			yAxis[2] = abs(getAccelY()) + 48;
			yAxis[3] = '\0';

			sprintf(yAxisf, "y%.2f", getAccelYf());
		}

		if(getAccelZ() >= 0)
		{
			zAxis[0] = 'z';
			zAxis[1] = getAccelZ() + 48;
			zAxis[2] = '\0';


			sprintf(zAxisf, "z%.2f", getAccelZf());
			if(zAxis[1] == 48)
			{
				printf("Dupa");
			}
		}
		else
		{
			zAxis[0] = 'z';
			zAxis[1] = '-';
			zAxis[2] = abs(getAccelZ()) + 48;
			zAxis[3] = '\0';

			sprintf(zAxisf, "z%.2f", getAccelZf());
		}

		/*xAxis = getAccelX() + 48;
		yAxis = getAccelY() + 48;
		zAxis = getAccelZ() + 48;*/

		if(0 == turnedOnSentTimer)
		{
			TimerIdSentMsg = SYSTM001_CreateTimer(30,SYSTM001_PERIODIC,sendMsgLSM9DS1,NULL);
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
	else if(!strcmp(device, "Gyroscope"))
	{
		if(0 == i){
			startMeasurements();
			++i;
		}
		readAndSendMeasurements(NULL);


		sprintf(xAxisf, "x%.2f", getGyroXf());

		sprintf(yAxisf, "y%.2f", getGyroYf());

		sprintf(zAxisf, "z%.2f", getGyroZf());

		if(0 == turnedOnSentTimer)
		{
			TimerIdSentMsg = SYSTM001_CreateTimer(30,SYSTM001_PERIODIC,sendMsgLSM9DS1,NULL);
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
	else if(!strcmp(device, "Magnetometer"))
	{
		if(0 == i){
			startMeasurements();
			++i;
		}
		readAndSendMeasurements(NULL);


		sprintf(xAxisf, "x%.2f", getMagnetXf());

		sprintf(yAxisf, "y%.2f", getMagnetYf());

		sprintf(zAxisf, "z%.2f", getMagnetZf());

		if(0 == turnedOnSentTimer)
		{
			TimerIdSentMsg = SYSTM001_CreateTimer(30,SYSTM001_PERIODIC,sendMsgLSM9DS1,NULL);
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

	if(copied == 1){
		if(0 == i){/*
			char xMs[51];

			for(int i = 0; i < 50; i++){
				xMs[i] = copiedData[i].ax;
				xMs[i] += 48; //to 48 to offset
			}
			xMs[i] = '\0';*/

			//send(xAxis, strlen((const char*)xAxis));

			send(xAxisf, strlen((const char*)xAxisf));

			//send(xMs, strlen(xMs));

			i++;
		}
		else if(1 == i)
		{
			/*char yMs[51];

			for(int i = 0; i < 50; i++){
				yMs[i] = copiedData[i].ay;
				yMs[i] += 48; //to 48 to offset
			}
			yMs[i] = '\0';*/

			//send(yAxis, strlen((const char*)yAxis));

			send(yAxisf, strlen((const char*)yAxisf));
			/*send(yMs,strlen(yMs));*/

			i++;
		}
		else if(2 == i)
		{
			/*char zMs[51];

			for(int i = 0; i < 50; i++){
				zMs[i] = copiedData[i].az;
				zMs[i] += 48; //to 48 to offset
			}
			zMs[i] = '\0';*/

			//send(zAxis, strlen((const char*)zAxis));

			send(zAxisf, strlen((const char*)zAxisf));
			//copied = 0;
			i = 0;
		}
	}
}
