/*
 * adxl372.h
 *
 *  Created on: Nov 17, 2018
 *      Author: justRandom
 */

#ifndef ADXL372_H_
#define ADXL372_H_

#include "stdint.h"
#include "stdbool.h"
#include "stm32f4xx_hal.h"

//Registers for ADXL372
const uint8_t DEVID_AD = 0x00;
const uint8_t DEVID_MST = 0x01;
const uint8_t DEVID_PRODUCT = 0x02;
const uint8_t REVID = 0x03;
const uint8_t STATUS = 0x04;
const uint8_t STATUS2 = 0x05;
const uint8_t FIFO_ENTRIES2 = 0x06;
const uint8_t FIFO_ENTRIES = 0x07;
const uint8_t XDATA_H = 0x08;
const uint8_t XDATA_L = 0x09;
const uint8_t YDATA_H = 0x0a;
const uint8_t YDATA_L = 0x0b;
const uint8_t ZDATA_H = 0x0c;
const uint8_t ZDATA_L = 0x0d;
const uint8_t MAXPEAK_X_H = 0x0e;
const uint8_t MAXPEAK_X_L = 0x15;
const uint8_t MAXPEAK_Y_H = 0x16;
const uint8_t MAXPEAK_Y_L = 0x17;
const uint8_t MAXPEAK_Z_H = 0x18;
const uint8_t MAXPEAK_Z_L = 0x19;
const uint8_t OFFSET_X = 0x20;
const uint8_t OFFSET_Y = 0x21;
const uint8_t OFFSET_Z = 0x22;
const uint8_t THRESH_ACT_X_H = 0x23;
const uint8_t THRESH_ACT_X_L = 0x24;
const uint8_t THRESH_ACT_Y_H = 0x25;
const uint8_t THRESH_ACT_Y_L = 0x26;
const uint8_t THRESH_ACT_Z_H = 0x27;
const uint8_t THRESH_ACT_Z_L = 0x28;
const uint8_t TIME_ACT = 0x29;
const uint8_t THRESH_INACT_X_H = 0x2A;
const uint8_t THRESH_INACT_X_L = 0x2B;
const uint8_t THRESH_INACT_Y_H = 0x2c;
const uint8_t THRESH_INACT_Y_L = 0x2d;
const uint8_t THRESH_INACT_Z_H = 0x2e;
const uint8_t THRESH_INACT_Z_L = 0x2f;
const uint8_t TIME_INACT_H = 0x30;
const uint8_t TIME_INACT_L = 0x31;
const uint8_t THRESH_ACT2_X_H = 0x32;
const uint8_t THRESH_ACT2_X_L = 0x33;
const uint8_t THRESH_ACT2_Y_L = 0x35;
const uint8_t THRESH_ACT2_Z_H = 0x36;
const uint8_t THRESH_ACT2_Z_L = 0x37;
const uint8_t HPF = 0x38;
const uint8_t FIFO_SAMPLES = 0x39;
const uint8_t FIFO_CTL = 0x3A;
const uint8_t INT1_MAP = 0x3b;
const uint8_t INT2_MAP = 0x3c;
const uint8_t TIMING = 0x3d;
const uint8_t MEASURE = 0x3e;
const uint8_t POWER_CTL = 0x3f;
const uint8_t SELF_TEST = 0x40;
const uint8_t DRESET = 0x41;
const uint8_t FIFO_DATA = 0x42;


class ADXL372
{
	public:
			ADXL372(SPI_HandleTypeDef* bmiSPI, uint16_t bmiGPIO, GPIO_TypeDef* bmiBANK)
			{
				SPI_HANDLER = bmiSPI;
				CSS_PIN = bmiGPIO;
				PIN_BANK = bmiBANK;
			}
			void regWrite(uint8_t reg, uint8_t data);
			uint8_t regRead(uint8_t reg);
			void initialize();
			bool test();
			void getReadableData(uint32_t* data);
			void getQuickData(uint8_t* data);
			void multiRead(uint8_t startReg, uint8_t* data, uint8_t nos);
	private:
			SPI_HandleTypeDef* SPI_HANDLER;			//Spi handler and pin setting stuff
			uint16_t CSS_PIN;
			GPIO_TypeDef* PIN_BANK;
};





#endif /* ADXL372_H_ */
