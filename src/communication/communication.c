/**
 * @file communication.c
 * @brief Communication Routines
 * @headerfile communication.h ""
 */

#include <stdio.h>
#include "communication.h"


int status;

int devices_init(IMU_PARAM_STRUCT *imu_param, SPI_PARAM_STRUCT *spi_param, MRA_DATA_STRUCT *mra_data)
{
#if USE_IMU
  if( (imu_param->i2c_dev = open("/dev/i2c-3", O_RDWR))<0 )
  {
    perror("Failed to open i2c_dev");
    return FAILURE;
  }

  if ((ioctl(imu_param->i2c_dev, I2C_SLAVE, ADD_HMC5883)) < 0) {
		perror("ioctl(I2C_SLAVE) mag");
		return FAILURE;
  }

  /*if( (mag_init(imu_param->i2c_dev, imu_param->mag.rate, imu_param->mag.range, imu_param->mag.samples_avg, imu_param->mag.meas_mode, imu_param->mag.op_mode))< 0)
  {
      perror("Error in magnetometer initialization");
      return FAILURE;
  }
*/  

  if( ioctl(imu_param->i2c_dev, I2C_SLAVE, ADD_ITG3200) < 0) {
		perror("ioctl(I2C_SLAVE) gyr");
		return FAILURE;
  } 

  if( gyr_init(imu_param->i2c_dev, imu_param->gyr.rate, imu_param->gyr.lpf_bw, imu_param->gyr.clk_source, imu_param->gyr.act)<0 )
  {
      perror("Error in gyrometer initialization");
      //return FAILURE;
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
#endif

  //#ifdef USE_SPI
  if ((spi_param->spi_dev=spi_init(spi_param->mode,spi_param->speed,spi_param->cs))<0) 
  {
    perror("Error in SPI device initialization");
    return FAILURE;
  }

  //Enable DAC and writing 0
  mra_data->v_ctl=0;
//  mra_data->Out_1=0;gpio_write(GPIO_CS_S3,1)==FAILURE

  actuate(spi_param->spi_dev, mra_data);
  if( mra_data->new_ctl == FAILURE )
  {
	perror("Error inhome directory MRA initialization");
	return FAILURE;
  }
 // if( gpio_write(GPIO_DAC_SHDN,1)==FAILURE )
 // {
//	perror("Error in SHDN initialization");
//	return FAILURE;
//  }
  
//#endif

  return SUCCESS;
}

int read_all_data(int i2c_dev, int spi_dev, IMU_DATA_STRUCT *imu_data,EFF_DATA_STRUCT *eff_data, MRA_DATA_STRUCT *mra_data , ENC_DATA_STRUCT *enc_data)
{
  short int data[4];
  int f=0;
  // Read IMU data
  imu_data->new_data=SUCCESS;
#if USE_IMU
  if ( (ioctl(i2c_dev, I2C_SLAVE, ADD_ADXL345))<0) 
	imu_data->new_data=0;
  else if( (acc_read_all_data(i2c_dev,data))==FAILURE )
  {
	imu_data->new_data=0;
	//printf("\nimu_data->new_data = %d\n",imu_data->new_data);
	//f=1;
  }
  else{
    imu_data->acc.x=data[0];
    imu_data->acc.y=data[1];
    imu_data->acc.z=data[2];  
  }
  
  if ( (ioctl(i2c_dev, I2C_SLAVE, ADD_ITG3200)) < 0) 
	imu_data->new_data=0;
  else if( (gyr_read_all_data(i2c_dev,data))==FAILURE )
	imu_data->new_data=0;
  else{
    imu_data->gyr.x=data[0];
    imu_data->gyr.y=data[1];
    imu_data->gyr.z=data[2];
    imu_data->temp=data[3];
  }
  
  if ( (ioctl(i2c_dev, I2C_SLAVE, ADD_HMC5883)) < 0) 
	imu_data->new_data=0;
  else if( (mag_read_all_data(i2c_dev,data))==FAILURE )
	imu_data->new_data=0;
  else{
    imu_data->mag.x=data[0];
    imu_data->mag.y=data[1];
    imu_data->mag.z=data[2];
  }
#endif
  //Read Efforts data
  eff_data->new_data=SUCCESS;
  
  if( (gpio_write(GPIO_CS_S3,1)==FAILURE) || (gpio_write(GPIO_CS_S2,1)==FAILURE) || (gpio_write(GPIO_CS_S1,1)==FAILURE) || (gpio_write(GPIO_CS_S0,0)==FAILURE))
	eff_data->new_data=FAILURE;
  else{ 
    if( (adc_read(spi_dev,0,1,data))==FAILURE )
	eff_data->new_data=FAILURE;
    else eff_data->F.x=data[0];
  //Read MR actuator voltage control
    if( (adc_read(spi_dev,7,1,data))==SUCCESS )
    {
	mra_data->v_ctl_read=data[0];
	mra_data->new_data=SUCCESS;
    }
  }
  //if( f==1 )
    //printf("imu_data->new_data antes do return= %d\n",imu_data->new_data);

  //Read Encoder data
  enc_data->new_data = SUCCESS;
#if USE_ENCODER
  if( (gpio_write(GPIO_CS_S3,1)==FAILURE) || (gpio_write(GPIO_CS_S2,1)==FAILURE) || (gpio_write(GPIO_CS_S1,1)==FAILURE) || (gpio_write(GPIO_CS_S0,1)==FAILURE))
	enc_data->new_data=FAILURE;
  if(enc_read_pos(enc_data->spi_dev,&(enc_data->angle))==FAILURE)
    enc_data->new_data = FAILURE;
#endif

  if( imu_data->new_data==FAILURE || eff_data->new_data==FAILURE || mra_data->new_data==FAILURE || enc_data->new_data == FAILURE)
    return FAILURE;
    
  return SUCCESS;
}

/*Function: actuate
 * Summary:      Applies the control signal to the actuator
 * Parameters:   spi_dev containing SPI variable ID
 *               mra_data struct containing the control signal
 * Return:       nothing
 */
void actuate(int spi_dev, MRA_DATA_STRUCT *mra_data)
{
  //printf("i = %d\n",i);
  //i++;
  if( gpio_write(GPIO_DAC_SHDN,1)==FAILURE )
	mra_data->new_ctl=FAILURE; else
  if( (gpio_write(GPIO_CS_S3,1)==FAILURE) || (gpio_write(GPIO_CS_S2,0)==FAILURE) || (gpio_write(GPIO_CS_S1,1)==FAILURE) || (gpio_write(GPIO_CS_S0,0)==FAILURE) )
	mra_data->new_ctl=FAILURE;
  else if( (dac_write(spi_dev,0,1,mra_data->v_ctl))==FAILURE )
	mra_data->new_ctl=FAILURE;
//  else if( (dac_write(spi_dev,1,1,mra_data->???))==FAILURE )
//	mra_data->success=FAILURE;
  else mra_data->new_ctl=SUCCESS;
  return;
}

void dac_shut_down(void)
{
  while( (gpio_read(GPIO_DAC_SHDN))==0 )
    gpio_write(GPIO_DAC_SHDN,0);
  return;
}
