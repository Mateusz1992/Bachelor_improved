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
#include "HTU21D.h"

char device[20]; //tablica w ktorej przechowujemy wybrany sensor
int turnedOn = 0; //czy ktorys timer jest wlaczony

char xAxis[10]/*[50]*/;
char yAxis[10]/*[50]*/;
char zAxis[10]/*[50]*/;

char xAxisf[30]/*[50]*/;
char yAxisf[30]/*[50]*/;
char zAxisf[30]/*[50]*/;

//all sensors
char accX[20];
char accY[20];
char accZ[20];

char gyrX[20];
char gyrY[20];
char gyrZ[20];

char magX[20];
char magY[20];
char magZ[20];
//all sensors

char humidity[30];

char json_data[300];

bool connectionFailure; //jesli wystapil problem z polaczeniem bluetooth to trzeba usunac timer, zmienna wskazujaca

handle_t TimerIdSentMsg; //id timera do
int turnedOnSentTimer;
char copied;
accel copiedData[100];

void sendMeasurementsToBt(void)
{
	static int i = 0;
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
	else if(!strcmp(device, "Temperature"))
	{
		if(0 == i){
			//startMeasurements();
			++i;
		}
		//readAndSendMeasurements(NULL);

		float temp = readTemperature();

		sprintf(humidity, "%.2f ", temp);

		if(0 == turnedOnSentTimer)
		{
			//TimerIdSentMsg = SYSTM001_CreateTimer(30,SYSTM001_PERIODIC,sendMsgLSM9DS1,NULL);
			TimerIdSentMsg = SYSTM001_CreateTimer(30,SYSTM001_PERIODIC,sendMsgHTU21D,NULL);
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
	else if(!strcmp(device, "Humidity"))
	{
		if(0 == i){
			//startMeasurements();
			++i;
		}
		//readAndSendMeasurements(NULL);

		float temp = readHumidity();

		sprintf(humidity, "%.2f ", temp);

		if(0 == turnedOnSentTimer)
		{
			//TimerIdSentMsg = SYSTM001_CreateTimer(30,SYSTM001_PERIODIC,sendMsgLSM9DS1,NULL);
			TimerIdSentMsg = SYSTM001_CreateTimer(30,SYSTM001_PERIODIC,sendMsgHTU21D,NULL);
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
	else if(!strcmp(device, "All_sens"))
	{
		if(0 == i){
			startMeasurements();
			++i;
		}
		readAndSendMeasurements(NULL);

		//acceleration all sensors
		sprintf(accX, "x%.2f ", getAccelXf());

		sprintf(accY, "y%.2f ", getAccelYf());

		sprintf(accZ, "z%.2f ", getAccelZf());
		//acceleration all sensors

		//gyroscope all sensors
		sprintf(gyrX, "x%.2f ", getGyroXf());

		sprintf(gyrY, "y%.2f ", getGyroYf());

		sprintf(gyrZ, "z%.2f ", getGyroZf());
		//gyroscope all sensors

		//magnetometer all sensors
		sprintf(magX, "x%.2f ", getMagnetXf());

		sprintf(magY, "y%.2f ", getMagnetYf());

		sprintf(magZ, "z%.2f ", getMagnetZf());
		//magnetometer all sensors

		sprintf(json_data, "{"
								"\"m\":"
								"["
										"{"
											"\"id\":a,"
											"\"x\":%.2f,"
											"\"y\":%.2f,"
											"\"z\":%.2f"
										"},"
										"{"
											"\"id\":g,"
											"\"x\":%.2f,"
											"\"y\":%.2f,"
											"\"z\":%.2f"
										"},"
										"{"
											"\"id\":m,"
											"\"x\":%.2f,"
											"\"y\":%.2f,"
											"\"z\":%.2f"
										"},"
										"{"
											"\"id\":t,"
											"\"x\":%.2f"
										"},"
										"{"
											"\"id\":h,"
											"\"x\":%.2f"
										"}"
								"]"
							"}", getAccelXf(), getAccelYf(), getAccelZf(), getGyroXf(), getGyroYf(), getGyroZf(), getMagnetXf(), getMagnetYf(), getMagnetZf(),
							readTemperature(), readHumidity());

		if(0 == turnedOnSentTimer)
		{
			TimerIdSentMsg = SYSTM001_CreateTimer(50,SYSTM001_PERIODIC,sendAllSensors,NULL);
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
		if(0 == i){
			//send("a", strlen("a"));
			//send("x", strlen("x"));
			send(xAxisf, strlen((const char*)xAxisf));

			i++;
		}
		else if(1 == i)
		{
			//send("a", strlen("a"));
			//send("y", strlen("x"));

			send(yAxisf, strlen((const char*)yAxisf));

			i++;
		}
		else if(2 == i)
		{
			/*send("a", strlen("a"));
			send("z", strlen("x"));*/

			send(zAxisf, strlen((const char*)zAxisf));

			i = 0;
		}
	}
}


void sendMsgHTU21D(void *T)
{
	static int i = 0;

	/*if(copied == 1)
	{*/
		send(humidity, strlen((const char*)humidity));
	//}
}


void sendAllSensors(void *T)
{
	static int i = 0;

	if(copied == 1){
		send(json_data, strlen((const char*)json_data));
		/*if(0 == i)//accelerometer BEGIN
		{
			send("a", strlen("a"));
			i++;
		}
		else if(1 == i){
			send(accX, strlen((const char*)accX));

			i++;
		}
		else if(2 == i)
		{
			send("a", strlen("a"));
			i++;
		}
		else if(3 == i)
		{
			send(accY, strlen((const char*)accY));

			i++;
		}
		else if(4 == i)
		{
			send("a", strlen("a"));
			i++;
		}
		else if(5 == i)
		{
			send(accZ, strlen((const char*)accZ));

			//i = 0;
			i++;
		}//accelerometer END
		else if(6 == i)//gyroscope BEGIN
		{
			send("g", strlen("g"));
			i++;
		}
		else if(7 == i)
		{
			send(gyrX, strlen((const char*)gyrX));
			i++;
		}
		else if(8 == i)
		{
			send("g", strlen("g"));
			i++;
		}
		else if(9 == i)
		{
			send(gyrY, strlen((const char*)gyrY));
			i++;
		}
		else if(10 == i)
		{
			send("g", strlen("g"));
			i++;
		}
		else if(11 == i)
		{
			send(gyrZ, strlen((const char*)gyrZ));
			i++;
		}//gyroscope END
		else if(12 == i) //magnetometer BEGIN
		{
			send("m", strlen("m"));
			i++;
		}
		else if(13 == i)
		{
			send(magX, strlen((const char*)magX));
			i++;
		}
		else if(14 == i)
		{
			send("m", strlen("m"));
			i++;
		}
		else if(15 == i)
		{
			send(magY, strlen((const char*)magY));
			i++;
		}
		else if(16 == i)
		{
			send("m", strlen("m"));
			i++;
		}
		else if(17 == i)
		{
			send(magZ, strlen((const char*)magZ));
			i++;
		} //magnetometer END
		else if(18 == i)
		{
			send(json_data, strlen((const char*)json_data));
			i = 0;
		}*/
	}
}
