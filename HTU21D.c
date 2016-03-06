/*
 * HTU21D.c
 *
 *  Created on: 28-02-2016
 *      Author: Mateusz
 */
#include "HTU21D.h"
#include "FIFO_functions.h"

#define DELAY 12000
#define SHIFTED_DIVISOR 0x988000


handle_t TimerRdHumid;
handle_t TimerRdTmp;

int errorCounter = 0;
int errorCounterTmp = 0;



int wrong_checksum = 0;

int readHumid = 0;
int readTmp = 0;

int timerHumidOn = 0;
int timerTmpOn = 0;

float rh = 0;
float realTemperature = 0;

int canH = 1;
int canT = 1;

float readHumidity(void)
{
	if(canH)
	{
		canH = 0;
		I2C001_DataType data1;
		data1.Data1.TDF_Type = I2C_TDF_MStart;
		data1.Data1.Data = ((HTDU21D_ADDRESS << 1) | I2C_WRITE);
		I2C001_WriteData(&I2C001_Handle1,&data1);

		delay11(DELAY);

		I2C001_DataType data2;
		data2.Data1.TDF_Type = I2C_TDF_MTxData;
		data2.Data1.Data = TRIGGER_HUMD_MEASURE_NOHOLD;
		I2C001_WriteData(&I2C001_Handle1,&data2);

		delay11(DELAY);

		if(timerHumidOn == 0)
		{
			TimerRdHumid = SYSTM001_CreateTimer(55, SYSTM001_PERIODIC, readHumidTimerHandler, NULL);
			SYSTM001_StartTimer(TimerRdHumid);
			timerHumidOn = 1;
		}

		while(TRUE)
		{
			if(errorCounter > 3)
			{
				errorCounter = 0;
				SYSTM001_StopTimer(TimerRdHumid);
				SYSTM001_DeleteTimer(TimerRdHumid);
				TimerRdHumid = 0;
				timerHumidOn = 0;
				canH = 1;
				return 998;
			}
			else if(1 == wrong_checksum)
			{
				wrong_checksum = 0;
				SYSTM001_StopTimer(TimerRdHumid);
				SYSTM001_DeleteTimer(TimerRdHumid);
				TimerRdHumid = 0;
				timerHumidOn = 0;
				canH = 1;
				return 999;
			}
			else if(1 == readHumid)
			{
				readHumid = 0;
				SYSTM001_StopTimer(TimerRdHumid);
				SYSTM001_DeleteTimer(TimerRdHumid);
				TimerRdHumid = 0;
				timerHumidOn = 0;
				canH = 1;
				break;
			}
		}
	}
	return rh;
}

void readHumidTimerHandler(void *T)
{
	I2C001_DataType data1;
	data1.Data1.TDF_Type = I2C_TDF_MRStart;
	data1.Data1.Data = ((HTDU21D_ADDRESS << 1) | I2C_READ);
	I2C001_WriteData(&I2C001_Handle1,&data1);

	delay11(DELAY);

	if(I2C001_GetFlagStatus(&I2C001_Handle1,I2C001_FLAG_NACK_RECEIVED) == I2C001_SET)
	{
		I2C001_ClearFlag(&I2C001_Handle1,I2C001_FLAG_NACK_RECEIVED);
		errorCounter++;
	}
	else
	{
		uint8_t msb = 0x00;
		uint8_t lsb = 0x00;
		uint8_t checksum = 0x00;

		USIC_CH_TypeDef* I2CRegs = I2C001_Handle1.I2CRegs;

		I2C001_DataType data2;
		data2.Data1.TDF_Type = I2C_TDF_MRxAck0;
		data2.Data1.Data = ubyteFF;
		I2C001_WriteData(&I2C001_Handle1,&data2);

		delay11(DELAY);

		if(!USIC_ubIsRxFIFOempty(I2CRegs))
		{
			msb = (uint8_t)I2CRegs->OUTR;
			//Result = (bool)TRUE;
		}

		I2C001_DataType data3;
		data3.Data1.TDF_Type = I2C_TDF_MRxAck0;
		data3.Data1.Data = ubyteFF;
		I2C001_WriteData(&I2C001_Handle1,&data3);

		delay11(DELAY);

		if(!USIC_ubIsRxFIFOempty(I2CRegs))
		{
			lsb = (uint8_t)I2CRegs->OUTR;
			//Result = (bool)TRUE;
		}

		I2C001_DataType data4;
		data4.Data1.TDF_Type = I2C_TDF_MRxAck1;
		data4.Data1.Data = ubyteFF;
		I2C001_WriteData(&I2C001_Handle1,&data4);

		delay11(DELAY);

		I2C001_DataType data5;
		data5.Data1.TDF_Type = I2C_TDF_MStop;
		data5.Data1.Data = ubyteFF;
		I2C001_WriteData(&I2C001_Handle1,&data5);

		delay11(DELAY);



		int d = USIC_GetRxFIFOFillingLevel(I2CRegs);
		// Read receive buffer, put the data in DataReceive1


		if(!USIC_ubIsRxFIFOempty(I2CRegs))
		{
			checksum = (uint8_t)I2CRegs->OUTR;
			//Result = (bool)TRUE;
		}


		unsigned int rawHumid = ((unsigned int) msb << 8) | (unsigned int) lsb;

		if(check_crc(rawHumid, checksum) != 0)
		{
			wrong_checksum = 1;
		}

		rawHumid &= 0xFFFC; //Zero out the status bits but keep them in place

		//Given the raw temperature data, calculate the actual temperature
		float tempRH = rawHumid / (float)65536; //2^16 = 65536
		rh = -6 + (125 * tempRH); //From page 14

		readHumid = 1;
	}
}

float readTemperature(void)
{
	if(canT){
		canT = 0;

		I2C001_DataType data1;
		data1.Data1.TDF_Type = I2C_TDF_MStart;
		data1.Data1.Data = ((HTDU21D_ADDRESS << 1) | I2C_WRITE);
		I2C001_WriteData(&I2C001_Handle1,&data1);

		delay11(DELAY);

		I2C001_DataType data2;
		data2.Data1.TDF_Type = I2C_TDF_MTxData;
		data2.Data1.Data = TRIGGER_TEMP_MEASURE_NOHOLD;
		I2C001_WriteData(&I2C001_Handle1,&data2);

		delay11(DELAY);

		if(timerTmpOn == 0)
		{
			TimerRdTmp = SYSTM001_CreateTimer(55, SYSTM001_PERIODIC, readTmpTimerHandler, NULL);
			SYSTM001_StartTimer(TimerRdTmp);
			timerTmpOn = 1;
		}

		while(TRUE)
		{
			if(errorCounterTmp > 3)
			{
				errorCounterTmp = 0;
				SYSTM001_StopTimer(TimerRdTmp);
				SYSTM001_DeleteTimer(TimerRdTmp);
				TimerRdTmp = 0;
				timerTmpOn = 0;
				canT = 1;
				return 998;
			}
			else if(1 == wrong_checksum)
			{
				wrong_checksum = 0;
				SYSTM001_StopTimer(TimerRdTmp);
				SYSTM001_DeleteTimer(TimerRdTmp);
				TimerRdTmp = 0;
				timerTmpOn = 0;
				canT = 1;
				return 999;
			}
			else if(1 == readTmp)
			{
				readTmp = 0;
				SYSTM001_StopTimer(TimerRdTmp);
				SYSTM001_DeleteTimer(TimerRdTmp);
				TimerRdTmp = 0;
				timerTmpOn = 0;
				canT = 1;
				break;
			}
		}
	}
	return realTemperature;
}

void readTmpTimerHandler(void *T)
{
	I2C001_DataType data1;
	data1.Data1.TDF_Type = I2C_TDF_MRStart;
	data1.Data1.Data = ((HTDU21D_ADDRESS << 1) | I2C_READ);
	I2C001_WriteData(&I2C001_Handle1,&data1);

	delay11(DELAY);

	if(I2C001_GetFlagStatus(&I2C001_Handle1,I2C001_FLAG_NACK_RECEIVED) == I2C001_SET)
	{
		I2C001_ClearFlag(&I2C001_Handle1,I2C001_FLAG_NACK_RECEIVED);
		errorCounterTmp++;
	}
	else
	{
		uint8_t msb = 0x00;
		uint8_t lsb = 0x00;
		uint8_t checksum = 0x00;

		USIC_CH_TypeDef* I2CRegs = I2C001_Handle1.I2CRegs;

		I2C001_DataType data2;
		data2.Data1.TDF_Type = I2C_TDF_MRxAck0;
		data2.Data1.Data = ubyteFF;
		I2C001_WriteData(&I2C001_Handle1,&data2);

		delay11(DELAY);

		if(!USIC_ubIsRxFIFOempty(I2CRegs))
		{
			msb = (uint8_t)I2CRegs->OUTR;
			//Result = (bool)TRUE;
		}

		I2C001_DataType data3;
		data3.Data1.TDF_Type = I2C_TDF_MRxAck0;
		data3.Data1.Data = ubyteFF;
		I2C001_WriteData(&I2C001_Handle1,&data3);

		delay11(DELAY);

		if(!USIC_ubIsRxFIFOempty(I2CRegs))
		{
			lsb = (uint8_t)I2CRegs->OUTR;
			//Result = (bool)TRUE;
		}

		I2C001_DataType data4;
		data4.Data1.TDF_Type = I2C_TDF_MRxAck1;
		data4.Data1.Data = ubyteFF;
		I2C001_WriteData(&I2C001_Handle1,&data4);

		delay11(DELAY);

		I2C001_DataType data5;
		data5.Data1.TDF_Type = I2C_TDF_MStop;
		data5.Data1.Data = ubyteFF;
		I2C001_WriteData(&I2C001_Handle1,&data5);

		delay11(DELAY);



		int d = USIC_GetRxFIFOFillingLevel(I2CRegs);
		// Read receive buffer, put the data in DataReceive1


		if(!USIC_ubIsRxFIFOempty(I2CRegs))
		{
			checksum = (uint8_t)I2CRegs->OUTR;
			//Result = (bool)TRUE;
		}


		unsigned int rawTmp = ((unsigned int) msb << 8) | (unsigned int) lsb;

		if(check_crc(rawTmp, checksum) != 0)
		{
			wrong_checksum = 1;
		}

		rawTmp &= 0xFFFC; //Zero out the status bits but keep them in place

		//Given the raw temperature data, calculate the actual temperature
		float temp = rawTmp / (float)65536; //2^16 = 65536
		realTemperature = (float)(-46.85 + (175.72 * temp)); //From page 14

		readTmp = 1;
	}
}

/*void flushFIFO(void)
{
	USIC_CH_TypeDef* I2CRegs = I2C001_Handle0.I2CRegs;
	USIC_FlushTxFIFO(I2CRegs);
}*/


void delay11(int delay)
{
	for(int i = 0; i < delay; i++)
	{
		;
	}
}


uint8_t check_crc(uint16_t message_from_sensor, uint8_t check_value_from_sensor)
{
	  uint32_t remainder = (uint32_t)message_from_sensor << 8; //Pad with 8 bits because we have to add in the check value
	  remainder |= check_value_from_sensor; //Add on the check value

	  uint32_t divsor = (uint32_t)SHIFTED_DIVISOR;

	  for (int i = 0 ; i < 16 ; i++) //Operate on only 16 positions of max 24. The remaining 8 are our remainder and should be zero when we're done.
	  {
	    //Serial.print("remainder: ");
	    //Serial.println(remainder, BIN);
	    //Serial.print("divsor:    ");
	    //Serial.println(divsor, BIN);
	    //Serial.println();

	    if( remainder & (uint32_t)1<<(23 - i) ) //Check if there is a one in the left position
	      remainder ^= divsor;

	    divsor >>= 1; //Rotate the divsor max 16 times so that we have 8 bits left of a remainder
	  }

	  return (uint8_t)remainder;
}

