/*
 * CleanArray.c
 *
 *  Created on: 12-02-2016
 *      Author: Mateusz
 */
char device[20];

void cleanArray(void)
{
	for(int i = 0; i < 20; i++)
	{
		device[i] = '\0';
	}
}

