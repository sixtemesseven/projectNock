/*
 * adxl372.cpp
 *
 *  Created on: Nov 17, 2018
 *      Author: justRandom
 */

#include "adxl372.h"

void ADXL372::regWrite(uint8_t reg, uint8_t data)
{
	uint8_t d[2] = { };
	d[0] = reg << 1; //shift left 1, add 0 (LSB is R/W Bit)
	d[1] = data;

	HAL_GPIO_WritePin(PIN_BANK, CSS_PIN, GPIO_PIN_RESET); //TODO replace GPIO with sturct
	HAL_SPI_Transmit(SPI_HANDLER, d, 2, 1000);
	HAL_GPIO_WritePin(PIN_BANK, CSS_PIN, GPIO_PIN_SET);
}

uint8_t ADXL372::regRead(uint8_t reg)
{
	uint8_t d[2] = {};
	d[0] = (reg << 1) + 1; //shift left 1, add 1 (LSB is R/_W Bit)

	HAL_GPIO_WritePin(PIN_BANK, CSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(SPI_HANDLER, &d[0], 1, 1000);
	HAL_Delay(1);
	HAL_SPI_Receive(SPI_HANDLER, &d[1], 1, 1000);
	HAL_GPIO_WritePin(PIN_BANK, CSS_PIN, GPIO_PIN_SET);
	return d[1];
}

void ADXL372::initialize()
{
	HAL_GPIO_WritePin(PIN_BANK, CSS_PIN, GPIO_PIN_SET);
	HAL_Delay(10);
}

bool ADXL372::test()
{
	//Testing device id of device
	return(regRead(0x00)==0x1D);
}

void ADXL372::multiRead(uint8_t startReg, uint8_t* data, uint8_t nos)
{
	//Send one byte specifing the register to read
	//Read nos bytes of data back
	HAL_GPIO_WritePin(PIN_BANK, CSS_PIN, GPIO_PIN_RESET);
	uint8_t startRegOne = startReg + 0b10000000; //read access so plus 128
	HAL_SPI_Transmit(SPI_HANDLER, &startRegOne, 1, 1000);
	HAL_SPI_Receive(SPI_HANDLER, data, nos, 1000);
	HAL_GPIO_WritePin(PIN_BANK, CSS_PIN, GPIO_PIN_SET);
}

void ADXL372::getReadableData(uint32_t* data)
{
	//Read sensor bytes into buffer
	uint8_t buffer[15] = {0x00};
	multiRead(0x0C, buffer, 15);

	//Calculate Accel and Gyro values
	//k counts the position of buffer[]
	//i counts the position of data[]
	uint8_t k = 0;
	for(uint8_t i = 0; i < 6; i++)
	{
		data[i] = (uint32_t) buffer[k++]; 				//LSB
		data[i] += ((uint32_t) buffer[k++]) << 8; 		//MSB
	}

	//Calculate Sensor Time
	//k counts the position of buffer[]
	//j counts the bitshift position
	for(uint8_t j = 0; j < 3; j++)
	{
		data[6] += (((uint32_t) buffer[k++]) << (8*j)); 				//sensor time lsb to msb
	}
}

void ADXL372::getQuickData(uint8_t* data)
{
	//Read Accel and Gyro Bytes and write to data
	multiRead(0x0C, data, 15);
}

