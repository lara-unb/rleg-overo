/**
 * @author Caio Gustavo Mesquita Ângelo
 * Rev 0 - 12/11/2012
 * RLEG project - 2012
 * 
 * Implements the I2C communication between Gumstix Overo Fire and ITG3200
 */
#include"imu_functions.h"

int gyr_write_reg(int i2c_dev, uint8_t reg, uint8_t data)
{
  uint8_t reg_data[2];

  reg_data[0] = reg;
  reg_data[1] = data;

  if( !( (reg==GYR_WHO_AM_I)||((GYR_SMPLRT_DIV<=reg)&&(reg<=GYR_INT_CFG))||(reg==GYR_PWR_MGM) ) )
  {
      //perror("Write unsucessful: Not a valid writable register");
      return FAILURE;
  }
	
  if (write(i2c_dev, &reg_data, 2) != 2) { 		 
	  //perror("Write unsuccessful");
	  return FAILURE;
  }

  return SUCCESS;
}

uint8_t* gyr_read_reg(int i2c_dev, uint8_t reg, uint8_t count)
{
  uint8_t data[9];
  int i;

  //data=(uint8_t*)malloc((count+1)*sizeof(data));
  
  data[0] = reg;

  if (write(i2c_dev, &data, 1) != 1) { 		 
	//perror("write before read");
	return NULL;
  }
  data[0] = 0;
  if (read(i2c_dev, &data, count) != count) {
	//perror("read");
	return NULL;	
  }

  return data;
}

int gyr_init(int i2c_dev, float rate, short int lpf_bw, char clk_source, char *act)
{ 
  uint8_t data=0;
  float Fint=1000;

  switch(lpf_bw){
    case 256:
      Fint=8000;
      data=0x18;
      break;
    case 188:
      data=0x19;
      break;
    case 98:
      data=0x1A;
      break;
    case 42:
      data=0x1B;
      break;
    case 20:
      data=0x1C;
      break;
    case 10:
      data=0x1D;
      break;
    case 5:
      data=0x1E;
      break;
    default:
      perror("Wrong low pass filter bandwidth value");
      break;
  }
  
  if(gyr_write_reg(i2c_dev, GYR_DLPF_FS, data)<0){
    perror("Write in Configuration register A unsuccesful");
    return -1;
  }

  if( (rate>Fint)||(rate<0) )
  {
    perror("Wrong choice for rate");
    return -1;
  }
  data=(uint8_t)(Fint/rate - 1);
  
  if(gyr_write_reg(i2c_dev, GYR_SMPLRT_DIV, data)<0){
    perror("Write in configuration register B unsuccesful");
    return -1;
  }
      
  switch(clk_source){
    case 'I':
      data=0x00;
      break;
    case 'X':
      data=0x01;
      break;
    case 'Y':
      data=0x02;
      break;
    case 'Z':
      data=0x03;
      break;
    default:
      perror("Wrong clk_source value");
      break;
  }
      
  if( strcmp(act,"XYZ")==0 );
  else if( strcmp(act,"YZ")==0 )
      data=data|0x20;
  else if( strcmp(act,"")==0 )
      data=data|0x40;
  else if( strcmp(act,"Y")==0 )
      data=data|0x28;
  else if( strcmp(act,"Z")==0 )
      data=data|0x30;
  else if( strcmp(act,"X")==0 )
      data=data|0x18;
  else if( strcmp(act,"XY")==0 )
      data=data|0x08;
  else if( strcmp(act,"XZ")==0 )
      data=data|0x10;
  else{
    perror("Wrong act value in gyrometer initialization");
    return -1;
  }
   
  if(gyr_write_reg(i2c_dev, GYR_PWR_MGM, data)<0){
    perror("Write in mode register unsuccesful");
    return -1;
  }
  
  return 1;
}

int gyr_read_all_data(int i2c_dev, short int *data)
{
  uint8_t i;
  uint8_t *data8;
  union result
  {
    unsigned short int usgnd[4];
    int short sgnd[4];
  } result;
  if( (data8=gyr_read_reg(i2c_dev,GYR_TEMP_OUT_H,8))==NULL)
  {
    //perror("Read accelerometer register failed");
    return FAILURE;
  }
  // Result receives gyro data:
  for(i=1; i<4; i++)
  {
    result.usgnd[i-1]=0;
    result.usgnd[i-1]=result.usgnd[i-1]|((unsigned short int)data8[i*2+1])|(((unsigned int)data8[i*2])<<8);
    data[i-1]=result.sgnd[i-1];
  }
  // Result receives temperature data:
  result.usgnd[3]=0;
  result.usgnd[3]=result.usgnd[3]|((unsigned short int)data8[1])|(((unsigned int)data8[0])<<8);  
  data[3]=result.sgnd[3];
  return SUCCESS;  
}

short int gyr_read_data(int i2c_dev, int type)
{
  uint8_t *data;
  union result
  {
    unsigned int usgnd;
    int sgnd;
  }result;

  switch(type){
    case 'X':
      data=gyr_read_reg(i2c_dev,GYR_GYRO_XOUT_H,2);
      break;
    case 'Y':
      data=gyr_read_reg(i2c_dev,GYR_GYRO_YOUT_H,2);
      break;
    case 'Z':
      data=gyr_read_reg(i2c_dev,GYR_GYRO_ZOUT_H,2);
      break;
    case 'T':
      data=gyr_read_reg(i2c_dev,GYR_TEMP_OUT_H,2);
    default:
      //perror("Wrong argument for type in gyr_read_data");
      return -1;
  }
  result.usgnd=0;
  result.usgnd=result.usgnd|(((unsigned short int)data[1]))|(((unsigned short int)data[0])<<8);
  return result.sgnd;
}
