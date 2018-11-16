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

bool testBMI160()
{


}
void multiReadBMI160(uint8_t startReg, uint8_t* data, uint8_t nos);
void getDataBMI160(uint16_t* data);
