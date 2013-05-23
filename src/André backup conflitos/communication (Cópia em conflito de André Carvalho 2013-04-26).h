#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "imu_functions.h"
#include "spi_functions.h"
#include "gpio_functions.h"
#include "imu_regs.h"


typedef struct imu_param{
    struct param_acc{
      uint8_t full_res;
      uint16_t rate; 
      uint8_t range;
    }acc;
    struct param_gyr {
      float rate;
      short int lpf_bw;
      char clk_source;
      char *act;
    }gyr;
    struct param_mag {
      uint8_t rate;
      uint8_t range;
      uint8_t samples_avg;
      uint8_t meas_mode;
      uint8_t op_mode;
    }mag;
    int i2c_dev;
}IMU_PARAM_STRUCT;

typedef struct spi_param{
    uint8_t mode;
    uint32_t speed;
    uint8_t cs;
    int spi_dev;
}SPI_PARAM_STRUCT;

typedef struct dataxyz{
    short int x;
    short int y;
    short int z;
}DATA_XYZ;

typedef struct imu_data{
  DATA_XYZ acc;
  DATA_XYZ gyr;
  DATA_XYZ mag;
  short int temp;
  uint8_t new_data;
}IMU_DATA_STRUCT;
/*
typedef struct ang_data{
  short int knee;
  //...
}ANG_DATA_STRUCT;
*/
typedef struct eff_data{
  DATA_XYZ F;
  DATA_XYZ M;
  uint8_t new_data;
}EFF_DATA_STRUCT;

  // INITIALIZATION OF SENSORS AND DEVICES
  int devices_init(IMU_PARAM_STRUCT *imu_param, SPI_PARAM_STRUCT *spi_param);
  
  // READ ALL DATA FROM SENSORS AND ADC
  void read_all_data(int i2c_dev, int spi_dev, IMU_DATA_STRUCT *imu_data,EFF_DATA_STRUCT *eff_data /*, ANG_DATA_STRUCT &ang_data*/);
  
  
#endif