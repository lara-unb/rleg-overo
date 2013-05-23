/*
Author: Caio Gustavo Mesquita Ângelo
Rev 0 - 11/11/2012
RLEG project - 2012

Implements the I2C communication between Gumstix Overo Fire and HMC5883
*/
#include"imu_functions.h"

int mag_write_reg(int i2c_dev, uint8_t reg, uint8_t data)
{
  uint8_t reg_data[2];

  reg_data[0] = reg;
  reg_data[1] = data;

  if( MAG_MODE<reg )
  {
      perror("Write unsucessful: Not a valid writable register");
      return -1;
  }
	
  if (write(i2c_dev, &reg_data, 2) != 2) { 		 
	  perror("Write unsuccessful");
	  return -1;
  }

  return 1;
}

uint8_t* mag_read_reg(int i2c_dev, uint8_t reg, uint8_t count)
{
  uint8_t data[13];
  int i;

  //data=(uint8_t*)malloc((count+1)*sizeof(data));
  
  data[0] = reg;

  if (write(i2c_dev, &data, 1) != 1) { 		 
	perror("write before read");
	return NULL;
  }
  data[0] = 0;
  if (read(i2c_dev, &data, count) != count) {
	perror("read");
	return NULL;	
  }

  return data;
}

int mag_init(int i2c_dev, uint8_t rate, uint8_t range, uint8_t samples_avg, uint8_t meas_mode, uint8_t op_mode)
{ 
  uint8_t data=0;

  switch(rate){
    case 15:
      data=0x10;
      break;
    case 0:
      data=0x00;
      break;
    case 1:
      data=0x04;
      break;
    case 3:
      data=0x08;
      break;
    case 7:
      data=0x0C;
      break;
    case 30:
      data=0x14;
      break;
    case 75:
      data=0x18;
      break;
    default:
      perror("Wrong rate value");
      break;
  }
  
  switch(samples_avg){
    case 8:
      data=data|0x60;
      break;
    case 1:
      break;
    case 2:
      data=data|0x20;
      break;
    case 4:
      data=data|0x40;
      break;
    default:
      perror("Wrong samples_avg value");
      break;
  }
  
  data+=meas_mode;
  
  if(mag_write_reg(i2c_dev, MAG_CONFIG_A, data)<0){
    perror("Write in Configuration register A unsuccesful");
    return -1;
  }
  
  if(mag_write_reg(i2c_dev, MAG_CONFIG_B, range<<5)<0){
    perror("Write in configuration register B unsuccesful");
    return -1;
  }
  
  if(mag_write_reg(i2c_dev, MAG_MODE, op_mode)<0){
    perror("Write in mode register unsuccesful");
    return -1;
  }
  
  return 1;
}

uint8_t mag_read_all_data(int i2c_dev, short int *data)
{
  uint8_t i, j;
  uint8_t *data8;
  union result
  {
    unsigned short int usgnd[3];
    short int sgnd[3];
  } result;
  //result=(union result*)malloc(3*sizeof(union result));
  if( (data8=mag_read_reg(i2c_dev,MAG_DATAX1,6))==NULL)
  {
    perror("Read accelerometer register failed");
    return -1;
  }
  //result.usgnd=(unsigned int*)malloc(3*sizeof(unsigned int));
  j=0;
  for(i=0; i<3; i++)
  {
    result.usgnd[j]=0;
    result.usgnd[j]=result.usgnd[j]|((unsigned short int)data8[i*2+1])|(((unsigned int)data8[i*2])<<8);
    data[j]=result.sgnd[j];
    j=2-i; // trick used to change the sorting of the data from XZY to XYZ
  }
  return result.sgnd;  
}

short int mag_read_data(int i2c_dev, int axis)
{
  uint8_t *data;
  union result
  {
    unsigned int usgnd;
    int sgnd;
  }result;

  switch(axis){
    case 'X':
      data=mag_read_reg(i2c_dev,MAG_DATAX1,2);
      break;
    case 'Y':
      data=mag_read_reg(i2c_dev,MAG_DATAY1,2);
      break;
    case 'Z':
      data=mag_read_reg(i2c_dev,MAG_DATAZ1,2);
      break;
    default:
      perror("Wrong argument for axis in mag_read_data");
      return -1;
  }
  result.usgnd=0;
  result.usgnd=result.usgnd|(((unsigned short int)data[1]))|(((unsigned short int)data[0])<<8);
  return result.sgnd;
}