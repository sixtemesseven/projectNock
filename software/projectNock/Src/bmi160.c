/*
 * bmi160.c
 *
 *  Created on: Nov 15, 2018
 *      Author: justRandom
 */

#include "bmi160.h"


BMI160_SPI_HARDWARE_DEF initBMI160(SPI_HandleTypeDef bmiSPI, uint16_t bmiGPIO, GPIO_TypeDef bmiBANK)
{
	BMI160_SPI_HARDWARE_DEF buf;
	buf.BMI160_SPI_HANDLER = bmiSPI;
	buf.BMI160_CSS_PIN = bmiGPIO;
	buf.BMI160_PIN_BANK = bmiBANK;
	return buf;
}


void regWrite(BMI160_SPI_HARDWARE_DEF bmiInterface, uint8_t reg, uint8_t data)
{
	uint8_t d[2] = { };
	d[0] = reg + 0b10000000;
	d[1] = data;

	HAL_GPIO_WritePin(GPIOA, bmiInterface.BMI160_CSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&bmiInterface.BMI160_SPI_HANDLER, d, 2, 1000);
	HAL_GPIO_WritePin(GPIOA, bmiInterface.BMI160_CSS_PIN, GPIO_PIN_SET);
}

uint8_t regRead(BMI160_SPI_HARDWARE_DEF bmiInterface, uint8_t reg)
{
	uint8_t d[2] = {};
	d[0] = reg + 0b10000000;

	HAL_GPIO_WritePin(GPIOA, bmiInterface.BMI160_CSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&bmiInterface.BMI160_SPI_HANDLER, &d[0], 1, 1000);
	HAL_Delay(1);
	HAL_SPI_Receive(&bmiInterface.BMI160_SPI_HANDLER, &d[1], 1, 1000);
	HAL_GPIO_WritePin(GPIOA, bmiInterface.BMI160_CSS_PIN, GPIO_PIN_SET);
	return d[1];
}

void initializeBMI160(BMI160_SPI_HARDWARE_DEF bmiInterface)
{
	regWrite(bmiInterface, BMI160_RA_CMD, BMI160_CMD_SOFT_RESET); //Soft reset to get into known state
	HAL_Delay(10);
	regRead(bmiInterface, 0x0F); //dummy read bmi to force spi modes
	HAL_Delay(10);
	regWrite(bmiInterface, BMI160_RA_CMD, BMI160_CMD_ACC_MODE_NORMAL); //start accelerometer
	HAL_Delay(1000); //TODO can be shorter but must be checked!!!
	regWrite(bmiInterface, BMI160_RA_CMD, BMI160_CMD_GYR_MODE_NORMAL); //start gyros
	HAL_Delay(1000); //TODO can be shorter but must be checked!!!
}





