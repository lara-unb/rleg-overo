#include<stdio.h>
#include "communication.h"


int status;

int devices_init(IMU_PARAM_STRUCT *imu_param, SPI_PARAM_STRUCT *spi_param)
{
  
  if( (imu_param->i2c_dev = open("/dev/i2c-3", O_RDWR))<0 )
  {
    perror("Failed to open i2c_dev");
    return FAILURE;
  }
  if ((ioctl(imu_param->i2c_dev, I2C_SLAVE, ADD_HMC5883)) < 0) {
		perror("ioctl(I2C_SLAVE) mag");
		return FAILURE;
  }
  if( (mag_init(imu_param->i2c_dev, imu_param->mag.rate, imu_param->mag.range, imu_param->mag.samples_avg, imu_param->mag.meas_mode, imu_param->mag.op_mode))< 0)
  {
      perror("Error in magnetometer initialization");
      return FAILURE;
  }
  if( ioctl(imu_param->i2c_dev, I2C_SLAVE, ADD_ITG3200) < 0) {
		perror("ioctl(I2C_SLAVE) gyr");
		return FAILURE;
  } 
  if( gyr_init(imu_param->i2c_dev, imu_param->gyr.rate, imu_param->gyr.lpf_bw, imu_param->gyr.clk_source, imu_param->gyr.act)<0 )
  {
      perror("Error in gyrometer initialization");
      return FAILURE;
  }
  if ( (ioctl(imu_param->i2c_dev, I2C_SLAVE, ADD_ADXL345)) < 0) {
	perror("ioctl(I2C_SLAVE) acc");
	return FAILURE;
  }  
  if( acc_init(imu_param->i2c_dev, imu_param->acc.full_res, imu_param->acc.rate, imu_param->acc.range)<0)
  {
	perror("Error in accelerometer initialization");
	return FAILURE;
  }

  if ((spi_param->spi_dev=spi_init(spi_param->mode,spi_param->speed,spi_param->cs))<0) 
  {
    perror("Error in SPI device initialization");
    return FAILURE;
  }
  return SUCCESS;
}

void read_all_data(int i2c_dev, int spi_dev, IMU_DATA_STRUCT *imu_data,EFF_DATA_STRUCT *eff_data /*, ANG_DATA_STRUCT &ang_data*/)
{
  short int data[4];
  int f=0;
  // Read IMU data
  imu_data->new_data=SUCCESS;
  
  if ( (ioctl(i2c_dev, I2C_SLAVE, ADD_ADXL345))<0) 
	imu_data->new_data=0;
  if( (acc_read_all_data(i2c_dev,data))==FAILURE )
  {
	imu_data->new_data=0;
	printf("\nimu_data->new_data = %d\n",imu_data->new_data);
	f=1;
  }
  imu_data->acc.x=data[0];
  imu_data->acc.y=data[1];
  imu_data->acc.z=data[2];  
  
  if ( (ioctl(i2c_dev, I2C_SLAVE, ADD_ITG3200)) < 0) 
	imu_data->new_data=0;
  if( (gyr_read_all_data(i2c_dev,data))==FAILURE )
	imu_data->new_data=0;
  imu_data->gyr.x=data[0];
  imu_data->gyr.y=data[1];
  imu_data->gyr.z=data[2];
  imu_data->temp=data[3];
  
  if ( (ioctl(i2c_dev, I2C_SLAVE, ADD_HMC5883)) < 0) 
	imu_data->new_data=0;
  if( (mag_read_all_data(i2c_dev,data))==FAILURE )
	imu_data->new_data=0;
  imu_data->mag.x=data[0];
  imu_data->mag.y=data[1];
  imu_data->mag.z=data[2];
  
  //Read Efforts data
  eff_data->new_data=SUCCESS;
  
  if( (adc_read(spi_dev,0,1,data))==FAILURE )
	eff_data->new_data=FAILURE;
  eff_data->F.x=data[0];
  if( f==1 )
    printf("imu_data->new_data antes do return= %d\n",imu_data->new_data);
  return;
}