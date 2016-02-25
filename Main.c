/*
 * Main.c
 *
 *  Created on: 12-02-2016
 *      Author: Mateusz
 */

#include <DAVE3.h>			//Declarations from DAVE3 Code Generation (includes SFR declaration)

#include "BluetoothDriver.h"
#include "timerFunctions.h"
#include "LSM9DS1_Driver.h"
#include <string.h>
#include "SendMeasurementsToBt.h"

void timerHandlerSendMessage(void *T);

extern char device[20];


void check(void)
{
	if(I2C001_SET == I2C001_GetFlagStatus(&I2C001_Handle0,I2C001_FLAG_RSIF))
	{
		;
	}
	else if(I2C001_SET == I2C001_GetFlagStatus(&I2C001_Handle0,I2C001_FLAG_DLIF))
	{
		;
	}
	else if(I2C001_SET == I2C001_GetFlagStatus(&I2C001_Handle0,I2C001_FLAG_TSIF))
	{
		;
	}
	else if(I2C001_SET == I2C001_GetFlagStatus(&I2C001_Handle0,I2C001_FLAG_TBIF))
	{
		;
	}
	else if(I2C001_SET == I2C001_GetFlagStatus(&I2C001_Handle0,I2C001_FLAG_RIF))
	{
		;
	}
	else if(I2C001_SET == I2C001_GetFlagStatus(&I2C001_Handle0,I2C001_FLAG_WRONG_TDF))
	{
		;
	}
	else if(I2C001_SET == I2C001_GetFlagStatus(&I2C001_Handle0,I2C001_FLAG_NACK_RECEIVED))
	{
		;
	}
	else if(I2C001_SET == I2C001_GetFlagStatus(&I2C001_Handle0,I2C001_FLAG_SRBI))
	{
		int a;
	}
	else if(I2C001_SET == I2C001_GetFlagStatus(&I2C001_Handle0,I2C001_FLAG_RBERI))
	{
		int b;
	}
	else if(I2C001_SET == I2C001_GetFlagStatus(&I2C001_Handle0,I2C001_FLAG_STBI))
	{
		int c;
	}
	else if(I2C001_SET == I2C001_GetFlagStatus(&I2C001_Handle0,I2C001_FLAG_TBERI))
	{
		int d;
	}
}

int main(void)
{
//	status_t status;		// Declaration of return variable for DAVE3 APIs (toggle comment if required)
	int siema = 0;

	DAVE_Init();			// Initialization of DAVE Apps

	initLSM9DS1();
	calibrate(TRUE);
	resetConnectionIndicator();

	initBluetooth();
	startMeasurements();

	while(1)
	{
		//if(siema == 0 && )
		sendMeasurementsToBt();
		manageConnection();

		check();


	}
	return 0;
}

