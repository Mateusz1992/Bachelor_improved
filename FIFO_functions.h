/*
 * FIFO_functions.h
 *
 *  Created on: 12-02-2016
 *      Author: Mateusz
 */

#ifndef FIFO_FUNCTIONS_H_
#define FIFO_FUNCTIONS_H_

#include <DAVE3.h>


void flushFIFO(const I2C001Handle_type *  I2CHandle);

void clearErrorFlags(void);

#endif /* FIFO_FUNCTIONS_H_ */
