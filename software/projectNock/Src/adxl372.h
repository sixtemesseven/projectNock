/*
 * adxl372.h
 *
 *  Created on: Nov 15, 2018
 *      Author: justRandom
 */

#ifndef ADXL372_H_
#define ADXL372_H_

	  //Registers for ADXL372
	  const uint16_t DEVID_AD = 0x00;
	  const uint16_t DEVID_MST = 0x01;
	  const uint16_t DEVID_PRODUCT = 0x02;
	  const uint16_t REVID = 0x03;
	  const uint16_t STATUS = 0x04;
	  const uint16_t STATUS2 = 0x05;
	  const uint16_t FIFO_ENTRIES2 = 0x06;
	  const uint16_t FIFO_ENTRIES = 0x07;
	  const uint16_t XDATA_H = 0x08;
	  const uint16_t XDATA_L = 0x09;
	  const uint16_t YDATA_H = 0x0a;
	  const uint16_t YDATA_L = 0x0b;
	  const uint16_t ZDATA_H = 0x0c;
	  const uint16_t ZDATA_L = 0x0d;
	  const uint16_t MAXPEAK_X_H = 0x0e;
	  const uint16_t MAXPEAK_X_L = 0x15;
	  const uint16_t MAXPEAK_Y_H = 0x16;
	  const uint16_t MAXPEAK_Y_L = 0x17;
	  const uint16_t MAXPEAK_Z_H = 0x18;
	  const uint16_t MAXPEAK_Z_L = 0x19;
	  const uint16_t OFFSET_X = 0x20;
	  const uint16_t OFFSET_Y = 0x21;
	  const uint16_t OFFSET_Z = 0x22;
	  const uint16_t THRESH_ACT_X_H = 0x23;
	  const uint16_t THRESH_ACT_X_L = 0x24;
	  const uint16_t THRESH_ACT_Y_H = 0x25;
	  const uint16_t THRESH_ACT_Y_L = 0x26;
	  const uint16_t THRESH_ACT_Z_H = 0x27;
	  const uint16_t THRESH_ACT_Z_L = 0x28;
	  const uint16_t TIME_ACT = 0x29;
	  const uint16_t THRESH_INACT_X_H = 0x2A;
	  const uint16_t THRESH_INACT_X_L = 0x2B;
	  const uint16_t THRESH_INACT_Y_H = 0x2c;
	  const uint16_t THRESH_INACT_Y_L = 0x2d;
	  const uint16_t THRESH_INACT_Z_H = 0x2e;
	  const uint16_t THRESH_INACT_Z_L = 0x2f;
	  const uint16_t TIME_INACT_H = 0x30;
	  const uint16_t TIME_INACT_L = 0x31;
	  const uint16_t THRESH_ACT2_X_H = 0x32;
	  const uint16_t THRESH_ACT2_X_L = 0x33;
	  const uint16_t THRESH_ACT2_Y_L = 0x35;
	  const uint16_t THRESH_ACT2_Z_H = 0x36;
	  const uint16_t THRESH_ACT2_Z_L = 0x37;
	  const uint16_t HPF = 0x38;
	  const uint16_t FIFO_SAMPLES = 0x39;
	  const uint16_t FIFO_CTL = 0x3A;
	  const uint16_t INT1_MAP = 0x3b;
	  const uint16_t INT2_MAP = 0x3c;
	  const uint16_t TIMING = 0x3d;
	  const uint16_t MEASURE = 0x3e;
	  const uint16_t POWER_CTL = 0x3f;
	  const uint16_t SELF_TEST = 0x40;
	  const uint16_t FIFO_DATA = 0x42;


#endif /* ADXL372_H_ */
