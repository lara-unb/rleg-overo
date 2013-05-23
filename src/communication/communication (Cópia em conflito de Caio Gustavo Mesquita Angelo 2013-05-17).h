#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "imu_functions.h"
#include "spi_functions.h"
#include "gpio_functions.h"
#include "imu_regs.h"

#define FAILURE		-1
#define SUCCESS		1

#define GPIO_CS_S0		168	// GPIOs that select CS in Demux
#define GPIO_CS_S1		64
#define GPIO_CS_S2		176
#define GPIO_CS_S3		65

#define GPIO_DAC_SHDN		66	// GPIO that deals with DAC power
#define GPIO_DAC_LDAC		67

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

typedef struct dataxyzdouble{
    double x;
    double y;
    double z;
}DATA_XYZ_DOUBLE;

typedef struct imu_data{
  DATA_XYZ acc;
  DATA_XYZ gyr;
  DATA_XYZ mag;
  struct calibrated{
    DATA_XYZ_DOUBLE acc;
    DATA_XYZ_DOUBLE gyr;
    DATA_XYZ_DOUBLE mag;
  }calib;
  short int temp;
  double calib_temp;
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
/*
typedef struct dac_data{
  short int Out_0;
  short int Out_1;
  short int Read_0;
  short int Read_1;
  int success;
}DAC_DATA_STRUCT;
*/
typedef struct mra_data{
  short int v_ctl;
  short int v_ctl_read;
  int new_data;
  int new_ctl;
}MRA_DATA_STRUCT;
  // INITIALIZATION OF SENSORS AND DEVICES
  int devices_init(IMU_PARAM_STRUCT *imu_param, SPI_PARAM_STRUCT *spi_param, MRA_DATA_STRUCT *mra_data);
  
  // READ ALL DATA FROM SENSORS AND ADC
  int read_all_data(int i2c_dev, int spi_dev, IMU_DATA_STRUCT *imu_data,EFF_DATA_STRUCT *eff_data, MRA_DATA_STRUCT *mra_data /*, ANG_DATA_STRUCT &ang_data*/);
  void actuate(int spi_dev, MRA_DATA_STRUCT *mra_data);
  void mra_shut_down(void);
#endif