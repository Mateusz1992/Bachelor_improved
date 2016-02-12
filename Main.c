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


int main(void)
{
//	status_t status;		// Declaration of return variable for DAVE3 APIs (toggle comment if required)


	DAVE_Init();			// Initialization of DAVE Apps

	initBluetooth();
	initLSM9DS1();
	calibrate(TRUE);
	startMeasurements();

	while(1)
	{

		sendMeasurementsToBt();
		manageConnection();
	}
	return 0;
}

