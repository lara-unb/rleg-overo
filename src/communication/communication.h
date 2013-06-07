/**
 * @file communication.h
 * @author Caio Gustavo Mesquita Ângelo
 * @date 2012-2013
 */


#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "imu_functions.h"
#include "spi_functions.h"
#include "gpio_functions.h"
#include "imu_regs.h"

#define GPIO_CS_S0		168	// GPIOs that select CS in Demux
#define GPIO_CS_S1		64
#define GPIO_CS_S2		176
#define GPIO_CS_S3		65

#define GPIO_DAC_SHDN		66	// GPIO that deals with DAC power
#define GPIO_DAC_LDAC		67


/**
 * Configs of IMU
 */
typedef struct imu_param{
    /**
     * Accelerometer Parameters
     */
    struct param_acc{
      uint8_t full_res;
      uint16_t rate; 
      uint8_t range;
    }acc;
    /**
     * Gyrometer Parameters
     */
    struct param_gyr {
      float rate;
      short int lpf_bw;
      char clk_source;
      char *act;
    }gyr;
    /**
     * Magnetometer Parameters
     */
    struct param_mag {
      uint8_t rate;
      uint8_t range;
      uint8_t samples_avg;
      uint8_t meas_mode;
      uint8_t op_mode;
    }mag;
    int i2c_dev;
}IMU_PARAM_STRUCT;

/**
 * Configs for SPI
 */
typedef struct spi_param{
    uint8_t mode;
    uint32_t speed;
    uint8_t cs;
    int spi_dev;
}SPI_PARAM_STRUCT;

/**
* A structure to represent a 3d Vector
*/
typedef struct dataxyz{
    short int x;
    short int y;
    short int z;
}DATA_XYZ;

/**
* Vector definition on double
*/
typedef struct dataxyzdouble{
    double x;
    double y;
    double z;
}DATA_XYZ_DOUBLE;

/**
 * Data of IMU structure
 */
typedef struct imu_data{
  DATA_XYZ acc; /**< Accel Vector*/
  DATA_XYZ gyr; /**< Gyrometer Vector*/
  DATA_XYZ mag; /**< Magnetormeter Vector*/
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
/**
 * Struct to control MRA
 */
typedef struct mra_data{
  short int v_ctl; /**<Voltage level for control output */
  short int v_ctl_read; /**< Voltage level read from the actuator*/
  int new_data; /**<??*/
  int new_ctl; /**<??*/
}MRA_DATA_STRUCT;

  /**
   * INITIALIZATION OF SENSORS AND DEVICES
   * @param *imu_param Structure with IMU parameters
   * @param *spi_param Structure with SPI parameters
   * @param *mra_data Structure to control MRA
   * @return flag with SUCCESS or FAILURE
   */
  int devices_init(IMU_PARAM_STRUCT *imu_param, SPI_PARAM_STRUCT *spi_param, MRA_DATA_STRUCT *mra_data);
  
  /**
   * READ ALL DATA FROM SENSORS AND ADC
   * @param i2c_dev 
   * @param spi_dev
   * @param *imu_data
   * @param *eff_data
   * @param *mra_data
   * @return flag with SUCCESS or FAILURE
   */
  int read_all_data(int i2c_dev, int spi_dev, IMU_DATA_STRUCT *imu_data,EFF_DATA_STRUCT *eff_data, MRA_DATA_STRUCT *mra_data /*, ANG_DATA_STRUCT &ang_data*/);
  
 /**
  * @brief Applies the control signal to the actuator
  * @param spi_dev Containing SPI variable ID
  * @param mra_data Struct containing the control signal
  * @return nothing
  */
  void actuate(int spi_dev, MRA_DATA_STRUCT *mra_data);

  /**
   * @brief Turn off MRA
   * @todo Check this function
   */
  void mra_shut_down(void);
#endif
