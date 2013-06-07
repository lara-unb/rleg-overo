/**
 * @file imu_regs.h
 * @brief Registers that are present in each device from the IMU (ADXL345, ITG3200 and HMC5883)
 */
#ifndef IMU_REGS_H
#define IMU_REGS_H

/* Registers in ADXL345 (acc) - Register map */
/* 	REGISTER_NAME		ADDRESS		TYPE	DESCRIPTION */
#define ACC_DEVID		0x00	//	R	Device ID
#define ACC_THRESH_TAP		0x1D	//	R/W	Tap Threshold
#define ACC_OFSX		0x1E	//	R/W	X-axis offset
#define ACC_OFSY		0x1F	//	R/W	Y-axis offset
#define ACC_OFSZ		0x20	//	R/W	Z-axis offset
#define ACC_DUR			0x21	//	R/W	Tap duration
#define ACC_LATENT		0x22	//	R/W	Tap latency
#define ACC_WINDOW		0x23	//	R/W	Tap window
#define ACC_THRESH_ACT		0x24	//	R/W	Activity threshold
#define ACC_THRESH_INACT	0x25	//	R/W	Inactivity threshold
#define ACC_TIME_INACT		0x26	//	R/W	Inactivity time
#define ACC_ACT_INACT_CTL	0x27	//	R/W	Axis enable control for activity and inactivity detection
#define ACC_THRESH_FF		0x28	//	R/W	Free-fall threshold
#define ACC_TIME_FF		0x29	//	R/W	Free-fall time
#define ACC_TAP_AXES		0x2A	//	R/W	Axis control for tap/double tap
#define ACC_ACT_TAP_STATUS	0x2B	//	R	Source of tap/double tap
#define ACC_BW_RATE		0x2C	//	R/W	Data rate and power mode control
#define ACC_POWER_CTL		0x2D	//	R/W	Power-saving features control
#define ACC_INT_ENABLE		0x2E	//	R/W	Interrupt enable control
#define ACC_INT_MAP		0x2F	//	R/W	Interrupt mapping control
#define ACC_INT_SOURCE		0x30	//	R	Source of interrupts
#define ACC_DATA_FORMAT		0x31	//	R/W	Data format control
#define ACC_DATAX0		0x32	//	R	X-Axis Data 0
#define ACC_DATAX1		0x33	//	R	X-Axis Data 1
#define ACC_DATAY0		0x34	//	R	Y-Axis Data 0
#define ACC_DATAY1		0x35	//	R	Y-Axis Data 1
#define ACC_DATAZ0		0x36	//	R	Z-Axis Data 0
#define ACC_DATAZ1		0x37	//	R	Z-Axis Data 1
#define ACC_FIFO_CTL		0x38	//	R/W	FIFO control
#define ACC_FIFO_STATUS		0x39	//	R	FIFO status

#define ACC_DATA_FORMAT_FULLRES_FULLRANGE	0x0B	// value in DATA_FORMAT to full resolution and full range (16g)
#define ACC_POWER_CTL_MEAS_MODE			0x08	// value in POWER_CTL to activate measurement mode 

/* Registers in ITG3200 (gyr) */
/* 	REGISTER_NAME		ADDRESS		TYPE	DESCRIPTION */
#define GYR_WHO_AM_I		0x00	//	R/W	Who Am I: I2C address
#define GYR_SMPLRT_DIV		0x15	//	R/W	Sample rate divider
#define GYR_DLPF_FS		0x16	//	R/W	Parameters related to the sensor acquisition
#define GYR_INT_CFG		0x17	//	R/W	Interrupt configuration
#define GYR_INT_STATUS		0x1A	//	R	Status of the device interrupts
#define GYR_TEMP_OUT_H		0x1B	//	R	Data output Temperature MSB
#define GYR_TEMP_OUT_L		0x1C	//	R	Data output Temperature LSB
#define GYR_GYRO_XOUT_H		0x1D	//	R	Data output X MSB
#define GYR_GYRO_XOUT_L		0x1E	//	R	Data output X LSB
#define GYR_GYRO_YOUT_H		0x1F	//	R	Data output Y MSB
#define GYR_GYRO_YOUT_L		0x20	//	R	Data output Y LSB
#define GYR_GYRO_ZOUT_H		0x21	//	R	Data output Z MSB
#define GYR_GYRO_ZOUT_L		0x22	//	R	Data output Z LSB
#define GYR_PWR_MGM		0x3E	//	R/W	Power management


/* Registers in HMC5883 (mag) */
/* 	REGISTER_NAME		ADDRESS		TYPE	DESCRIPTION */
#define MAG_CONFIG_A		0x00	//	R/W	Configuration register A
#define MAG_CONFIG_B		0x01	//	R/W	Configuration register B
#define MAG_MODE		0x02	//	R/W	Mode
#define MAG_DATAX1		0x03	//	R	Data output X MSB
#define MAG_DATAX0		0x04	//	R	Data output X LSB
#define MAG_DATAZ1		0x05	//	R	Data output Z MSB
#define MAG_DATAZ0		0x06	//	R	Data output Z LSB
#define MAG_DATAY1		0x07	//	R	Data output Y MSB
#define MAG_DATAY0		0x08	//	R	Data output Y LSB
#define MAG_STATUS		0x09	//	R	Status
#define MAG_ID_A		0x0A	//	R	Identification register A
#define MAG_ID_B		0x0B	//	R	Identification register B
#define MAG_ID_C		0x0C	//	R	Identification register C


#endif /* ifndef IMU_REGS_H */
