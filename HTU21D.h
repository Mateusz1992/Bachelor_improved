/*
 * HTU21D.h
 *
 *  Created on: 28-02-2016
 *      Author: Mateusz
 */

#ifndef HTU21D_H_
#define HTU21D_H_
#include <DAVE3.h>

#define TRUE 1

#define HTDU21D_ADDRESS 0x40  //Unshifted 7-bit I2C address for the sensor

#define TRIGGER_TEMP_MEASURE_HOLD  0xE3
#define TRIGGER_HUMD_MEASURE_HOLD  0xE5
#define TRIGGER_TEMP_MEASURE_NOHOLD  0xF3
#define TRIGGER_HUMD_MEASURE_NOHOLD  0xF5
#define WRITE_USER_REG  0xE6
#define READ_USER_REG  0xE7
#define SOFT_RESET  0xFE



float readHumidity(void);

void readHumidTimerHandler(void *T);



float readTemperature(void);

void readHumidTimerHandler(void *T);

void readTmpTimerHandler(void *T);

//void flushFIFO(void);

void delay11(int delay);

uint8_t check_crc(uint16_t message_from_sensor, uint8_t check_value_from_sensor);

#endif /* HTU21D_H_ */
