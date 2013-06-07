/**
 * \file imu_functions.h
 * Rev 0 - 06/11/2012
 * RLEG project - 2012
 *
 * @brief library of the I2C communication between Gumstix Overo Fire and 3 different devices: ADXL345, ITG3200 and HMC5883
 * @author Caio Gustavo Mesquita Ângelo
 * @date 2012-2013
 */
#ifndef IMU_FUNCTIONS_H_INCLUDED
#define IMU_FUNCTIONS_H_INCLUDED

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>

#include "imu_regs.h"

#define ASCII_0 0x30

#define SUCCESS 1
#define FAILURE	-1

#define ADD_ADXL345 0x53
#define ADD_ITG3200 0x68
#define ADD_HMC5883 0x1E

/////////////////////////
//     FUNCTIONS FOR   // 
//     ACCELEROMETER   //
//     ADXL345         //
/////////////////////////

/**
 * @defgroup acc Functions for accelerometer ADXL345
 */


/**
 * @brief WRITE TO REGISTER 
 * @ingroup acc
 * @param i2c_dev Communication Port
 * @param[in] reg Register Command
 * @param[in] data Data to write
 */
int acc_write_reg(int i2c_dev, uint8_t reg, uint8_t data);

/**
 * @brif READ  COUNT 8-BIT REGISTER IN SEQUENCE
 * @ingroup acc
 * @param i2c_dev Communication Port
 * @param[in] reg Register Command
 * @param[in] count Number of register in sequence (1-29)
 *
 * @return Data of register or NULL in case of any problem.
 */
uint8_t* acc_read_reg(int i2c_dev, uint8_t reg, uint8_t count);

/**
 * @brief INITIALIZE ACCELEROMETER
 * @ingroup acc
 * @param i2c_dev Communication Port
 * @param full_res Set resolution to 10 bits if value equal 0
 *        else set to Full resolution
 * @param rate Set data output rate (Hz), possible values:
 *        (3200,1600,800,400,200,100,50,25,
 *         12: 12.5Hz,
 *         6: 6.25Hz)
 * @param range Set range of read, possible values:
 *        (2: +- 2g
 *        4: +- 4g
 *        8: +- 8g
 *        anyother: 16g)
 * @return flag with SUCCESS or FAILURE
 */
int acc_init(int i2c_dev, uint8_t full_res, uint16_t rate, uint8_t range);

/**
 * @brief READ ALL DATA AT ONCE (X, Y and Z)
 * @ingroup acc
 * @param i2c_dev Communication Port
 * @param[out] data Data's vector
 * 
 * @return flag with SUCCESS or FAILURE
 */
int acc_read_all_data(int i2c_dev, short int *data);

/**
 * @brief READ DATA (X, Y or Z)
 * @ingroup acc
 * @param i2c_dev Communication Port
 * @param[in] axis Accelerate's axis to read ('X' or 'Y' or 'Z')
 *
 * @return 
 */
short int acc_read_data(int i2c_dev, int axis);

char* conv_byte_hex_bin(uint8_t* hvalue);

/**
 * @brief Read all accelerometer data and print in stdio
 * @ingroup acc
 * @param i2c_dev Communication Port
 */
int acc_read_all_reg(int i2c_dev);

//////////////////////////
//	FUNCTIONS FOR	//
//	GYROMETER	//
//	ITG3200 	//
//////////////////////////

/* WRITE TO REGISTER */
int gyr_write_reg(int i2c_dev, uint8_t reg, uint8_t data);

/* READ  COUNT 8-BIT REGISTER IN SEQUENCE*/
uint8_t* gyr_read_reg(int i2c_dev, uint8_t reg, uint8_t count);
/*
count:	number of registers in sequence (1 - 9)
*/

/* INITIALIZE GYROMETER */
int gyr_init(int i2c_dev, float rate, short int lpf_bw, char clk_source, char *act);
/* Parameters
- lpf_bw (low pass filter bandwidth in Hz) = 256, 188, 98, 42, 20, 10 or 5
- rate (output data rate in Hz)
<= 1000 (Fint), if lpf_bw is not 256
<= 8000 (Fint), if lpf_bw is 256
Real rate will be the closest superior rate possible, respecting the formula Rate = Fint/n, 1<=n<=255
- clk_source =
'I'	Internal oscillator
'X'	PLL with X Gyro reference
'Y'	PLL with Y Gyro reference
'Z'	Pll with Z Gyro reference
Obs: 	It is highly recommended that the device is configured to use one of the gyros as the
	clock reference, due to the improved stability
- act (activated gyro data) =
"XYZ"	All axes are activated
"YZ"	Only Y and Z axes are activated
"XZ"	Only X and Z axes are activated
"XY"	Only X and Y axes are activated
"X"	Only X axis is activated
"Y"	Only Y axis is activated
"Z"	Only Z axis is activated
""	Device in very low power sleep mode
*/

/* READ ALL DATA AT ONCE (X, Y, Z and T) */
int gyr_read_all_data(int i2c_dev, short int *data);

/* READ DATA (X, Y, Z or T) */
short int gyr_read_data(int i2c_dev, int type);
/* 
type: 'X' or 'Y' or 'Z' or 'T'(temperature)
*/

//////////////////////////
//	FUNCTIONS FOR	//
//	MAGNETOMETER	//
//	HMC5883 	//
//////////////////////////

/**
 * WRITE TO REGISTER
 */
int mag_write_reg(int i2c_dev, uint8_t reg, uint8_t data);

/**
 * READ  COUNT 8-BIT REGISTER IN SEQUENCE
 *
 * @param count number of registers in sequence (1-13)
 */
uint8_t* mag_read_reg(int i2c_dev, uint8_t reg, uint8_t count);

/**
 * INITIALIZE MAGNETOMETER 
 */
int mag_init(int i2c_dev, uint8_t rate, uint8_t range, uint8_t samples_avg, uint8_t meas_mode, uint8_t op_mode);
/* Parameters
- rate (data output rate in Hz) = 3, 15, 30, 75
0:	0.75Hz
1:	1.5Hz
7:	7.5Hz
- samples_avg (number of sampled averaged per measurement output) = 1, 2, 4, 8
- meas_mode (measurement mode):
0:	Normal measurement configuration. The device follows normal measurement flow.
	The positive and negative pins of the resistive load are left floating and high impedance
1:	Positive bias configuration for X, Y, and Z axes. In this configuration, a positive current
	is forced across the resistive load for all three axes
2:	Negative bias configuration for X, Y, and Z axes. In this configuration, a negative current
	is forced across the resistive load for all three axes
- range:
	Range (Ga)	Resolution (mG/LSb)
0:	+- 0.88		0.73
1:	+- 1.3		0.92
2:	+- 1.9		1.22
3:	+- 2.5		1.52
4:	+- 4.0		2.27
5:	+- 4.7		2.56
6:	+- 5.6		3.03
7:	+- 8.1		4.35
*/

/* READ ALL DATA AT ONCE (X, Y and Z) */
int mag_read_all_data(int i2c_dev, short int *data);

/* READ DATA (X, Y or Z) */
short int mag_read_data(int i2c_dev, int axis);
/* 
axis: 'X' or 'Y' or 'Z'
*/

#endif
