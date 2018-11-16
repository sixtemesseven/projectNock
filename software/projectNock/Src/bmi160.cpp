/*
 * bmi160.c
 *
 *  Created on: Nov 15, 2018
 *      Author: justRandom
 */

#include "bmi160.h"


void BMI160::regWrite(uint8_t reg, uint8_t data)
{
	uint8_t d[2] = { };
	d[0] = reg;
	d[1] = data;

	HAL_GPIO_WritePin(BMI160_PIN_BANK, BMI160_CSS_PIN, GPIO_PIN_RESET); //TODO replace GPIO with sturct
	HAL_SPI_Transmit(BMI160_SPI_HANDLER, d, 2, 1000);
	HAL_GPIO_WritePin(BMI160_PIN_BANK, BMI160_CSS_PIN, GPIO_PIN_SET);
}

uint8_t BMI160::regRead(uint8_t reg)
{
	uint8_t d[2] = {};
	d[0] = reg + 0b10000000;

	HAL_GPIO_WritePin(BMI160_PIN_BANK, BMI160_CSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(BMI160_SPI_HANDLER, &d[0], 1, 1000);
	HAL_Delay(1);
	HAL_SPI_Receive(BMI160_SPI_HANDLER, &d[1], 1, 1000);
	HAL_GPIO_WritePin(BMI160_PIN_BANK, BMI160_CSS_PIN, GPIO_PIN_SET);
	return d[1];
}

void BMI160::initializeBMI160()
{
	HAL_GPIO_WritePin(BMI160_PIN_BANK, BMI160_CSS_PIN, GPIO_PIN_SET); //Set css pin high
	HAL_Delay(100);
	regWrite(BMI160_RA_CMD, BMI160_CMD_SOFT_RESET); //Soft reset to get into known state
	HAL_Delay(10);
	regRead(0x0F); //dummy read bmi to force spi modes
	HAL_Delay(10);
	regWrite(BMI160_RA_CMD, BMI160_CMD_ACC_MODE_NORMAL); //start accelerometer
	HAL_Delay(2000); //TODO can be shorter but must be checked!!!
	regWrite(BMI160_RA_CMD, BMI160_CMD_GYR_MODE_NORMAL); //start gyros
	HAL_Delay(2000); //TODO can be shorter but must be checked!!!
}

bool BMI160::testBMI160()
{
	return(regRead(0x00)==0x1D);
}

void BMI160::multiReadBMI160(uint8_t startReg, uint8_t* data, uint8_t nos)
{
	HAL_GPIO_WritePin(BMI160_PIN_BANK, BMI160_CSS_PIN, GPIO_PIN_RESET);
	uint8_t startRegOne = startReg + 0b10000000; //read access so plus 128
	HAL_SPI_Transmit(BMI160_SPI_HANDLER, &startRegOne, 1, 1000);
	HAL_SPI_Receive(BMI160_SPI_HANDLER, data, nos, 1000);
	HAL_GPIO_WritePin(BMI160_PIN_BANK, BMI160_CSS_PIN, GPIO_PIN_SET);
}

void BMI160::getReadableDataBMI160(uint32_t* data)
{
	//k counts the position of buffer[]
	//i counts the position of data[]

	uint8_t buffer[15] = {0x00};
	multiReadBMI160(0x0C, buffer, 15);

	uint8_t k = 0;
	for(int i = 0; i < 6; i++)
	{
		data[i] = buffer[k]; //gyro / accel LSB
		k++;
		data[i] = ((uint16_t) buffer[k]) << 8; //MSB
		k++;
	}
	for(int i = 0; i < 3; i++)
	{
		data[6] += buffer[k] << 8*k; //sensor time lsb to msb
		k++;
	}
}

void BMI160::getQuickDataBMI160(uint8_t* data)
{
	multiReadBMI160(0x0C, data, 15);
}
