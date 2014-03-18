#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "communication/communication.h"

#define SUCCESS		1
#define FAILURE		-1

// used data rleg_99.mat and rleg.mag with ellipsoid_fit.m: rotating slowly at many directions
#define ACC_BIAS_X	19.2
#define ACC_BIAS_Y	6.024
#define ACC_BIAS_Z	-18.97
#define ACC_FS_X	262.82
#define ACC_FS_Y	262.15
#define ACC_FS_Z	252.20

#define MAG_BIAS_X	-143
#define MAG_BIAS_Y	-101
#define MAG_BIAS_Z	3
#define MAG_FS_X	177
#define MAG_FS_Y	139
#define MAG_FS_Z	85

// used data rleg_2.mat (stopped)
#define GYR_BIAS_X	-60.8067
#define GYR_BIAS_Y	40.0800
#define GYR_BIAS_Z	0.7267
#define GYR_FS_X	0.001214142	// from datasheet (rad/s)/LSB
#define GYR_FS_Y	0.001214142
#define GYR_FS_Z	0.001214142

// No tested
#define ENC_FS      0.08789062  // (2pi)/(2^12)
#define ENC_MAX     4096        // 2^12 

/**
 * Calibrate all sensors
 * @ingroup calibrate
 */
void calibrate_all(IMU_DATA_STRUCT *imu_data);

/**
 * Calibrate imu sensors
 * @ingroup calibrate
 */
void calibrate_imu(IMU_DATA_STRUCT *imu_data);

/**
 * Calibrate encoder to return values in the
 * rage of 0 to 90
 * @ingroup 
 */
void calibrate_enc(ENC_DATA_STRUCT *enc_data);

#endif