#include<stdio.h>
#include<stdlib.h>
#include "communication/communication.h"
#include "calibration.h"
//#include <string.h>


void calibrate_all(IMU_DATA_STRUCT *imu_data)
{
  calibrate_imu(imu_data);
  return;
}

void calibrate_imu(IMU_DATA_STRUCT *imu_data)
{
  // With this parameters, the vectors of acceleration and magnetic field will have norm = 1 theorically (i.e. in g or G)
  imu_data->calib.acc.x = ((double)imu_data->acc.x-ACC_BIAS_X)/ACC_FS_X;
  imu_data->calib.acc.y = ((double)imu_data->acc.y-ACC_BIAS_Y)/ACC_FS_Y;
  imu_data->calib.acc.z = ((double)imu_data->acc.z-ACC_BIAS_Z)/ACC_FS_Z;
  
  imu_data->calib.mag.x = ((double)imu_data->mag.x-MAG_BIAS_X)/MAG_FS_X;
  imu_data->calib.mag.y = ((double)imu_data->mag.y-MAG_BIAS_Y)/MAG_FS_Y;
  imu_data->calib.mag.z = ((double)imu_data->mag.z-MAG_BIAS_Z)/MAG_FS_Z;
  
  // With this parameters, we consider right datasheet scale and we get in rad/s
  imu_data->calib.gyr.x = ((double)imu_data->gyr.x-GYR_BIAS_X)*GYR_FS_X;
  imu_data->calib.gyr.y = ((double)imu_data->gyr.y-GYR_BIAS_Y)*GYR_FS_Y;
  imu_data->calib.gyr.z = ((double)imu_data->gyr.z-GYR_BIAS_Z)*GYR_FS_Z;
  
  imu_data->calib_temp = ((double)imu_data->temp+13200)/280 + 35;
  
  return;
}
