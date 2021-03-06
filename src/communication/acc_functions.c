/**
 * Rev 0 - 11/11/2012
 * RLEG project - 2012
 *
 * Implements the I2C communication between Gumstix Overo Fire and ADXL345
 * @author Caio Gustavo Mesquita Ângelo
 * @headerfile imu_functions.h ""
 */
#include "imu_functions.h"

int acc_init(int i2c_dev, uint8_t full_res, uint16_t rate, uint8_t range)
{ 
  uint8_t data=0;

  switch(range){
    case 2:
      data=0x08;;
      break;
    case 4:
      data=0x09;
      break;
    case 8:
      data=0x0A;
      break;
    default:
      data=0x0B;
      break;
  }
  
  if(full_res==0)
    data=data&(0xF7);
  
  if((acc_write_reg(i2c_dev, ACC_DATA_FORMAT, data))<0){
    perror("Data Format unsuccesful");
    return FAILURE;
  }
  
  data=ACC_POWER_CTL_MEAS_MODE;
  if((acc_write_reg(i2c_dev, ACC_POWER_CTL, data))<0){
    perror("Power Control unsuccesful");
    return FAILURE;
  }
  
  switch(rate){
    case 100:
      data=0x0A;
      break;
    case 3200:
      data=0x0F;;
      break;
    case 1600:
      data=0x0E;
      break;
    case 800:
      data=0x0D;
      break;
    case 400:
      data=0x0C;
      break;
    case 200:
      data=0x0B;
      break;
    case 50:
      data=0x09;
      break; 
    case 25:
      data=0x08;
      break;
    case 12:
      data=0x07;
      break;
    case 6:
      data=0x06;
      break;
    default:
      perror("Wrong rate value");
      return FAILURE;
  }
  if((acc_write_reg(i2c_dev, ACC_BW_RATE, data))<0){
    perror("Write in BW_RATE unsuccesful");
    return FAILURE;
  }
  
  return SUCCESS;
}

int acc_write_reg(int i2c_dev, uint8_t reg, uint8_t data)
{
  uint8_t reg_data[2];

  reg_data[0] = reg;
  reg_data[1] = data;

  if( reg==ACC_DEVID || reg==ACC_ACT_TAP_STATUS || reg==ACC_INT_SOURCE || reg==ACC_FIFO_STATUS ||
      reg==ACC_DATAX0 || reg==ACC_DATAX1 || reg==ACC_DATAY0 || reg==ACC_DATAY1 || reg==ACC_DATAZ0 || reg==ACC_DATAZ1)
  {
      perror("Write unsucessful: Read-Only Register");
      return FAILURE;
  }
	
  if ((write(i2c_dev, &reg_data, 2)) != 2) { 		 
	  perror("Write unsuccessful");
	  return FAILURE;
  }

  return SUCCESS;
}

uint8_t* acc_read_reg(int i2c_dev, uint8_t reg, uint8_t count)
{
  uint8_t data[29];
  int i;

  //data=(uint8_t*)malloc((count+1)*sizeof(data));
  
  data[0] = reg;

  if ((write(i2c_dev, &data, 1)) != 1) { 		 
	//perror("write before read");
	return NULL;
  }
  data[0] = 0;
  if ((read(i2c_dev, &data, count)) != count) {
	//perror("read");
	return NULL;	
  }

  return data;
}

short int acc_read_data(int i2c_dev, int axis)
{
  uint8_t *data;
  union result
  {
    unsigned short int usgnd;
    short int sgnd;
  }result;

  switch(axis){
    case 'X':
      data=acc_read_reg(i2c_dev,ACC_DATAX0,2);
      break;
    case 'Y':
      data=acc_read_reg(i2c_dev,ACC_DATAY0,2);
      break;
    case 'Z':
      data=acc_read_reg(i2c_dev,ACC_DATAZ0,2);
      break;
    default:
      //perror("Wrong argument for axis in acc_read_data");
      return -1;
  }
  result.usgnd=0;
  result.usgnd=result.usgnd|(((unsigned short int)data[0]))|(((unsigned short int)data[1])<<8);
  return result.sgnd;
}

int acc_read_all_data(int i2c_dev, short int *data)
{
  int i;
  uint8_t *data8;
  union result
  {
    unsigned short int usgnd[3];
    short int sgnd[3];
  } result;
  //result=(union result*)malloc(3*sizeof(union result));
  if( (data8=acc_read_reg(i2c_dev,ACC_DATAX0,6))==NULL )
  {
    //perror("Read accelerometer register failed");
    return FAILURE;
  }
  //result.usgnd=(unsigned int*)malloc(3*sizeof(unsigned int));
  for(i=0; i<3; i++)
  {
    result.usgnd[i]=0;
    result.usgnd[i]=result.usgnd[i]|((unsigned short int)data8[i*2])|(((unsigned short int)data8[i*2+1])<<8);
    data[i]=result.sgnd[i];
  }
  return SUCCESS;  
}

char* conv_byte_hex_bin(uint8_t* hvalue)
{
  char bits[9];
  int i;
  for( i=0; i<8; i++)
  {
    bits[i]=((*hvalue)<<i)>>7+ASCII_0;
  }
  bits[8]='\0';
  return bits;
}

int acc_read_all_reg(int i2c_dev)
{
  uint8_t *hvalue0, *hvalue1, *hvalue;
  int i;
  
  hvalue0=acc_read_reg(i2c_dev,ACC_DEVID,1);
  printf("\nhey");
  printf("\nhey3");
  printf("\nhey2");
  printf("\nhey3");
  hvalue1=acc_read_reg(i2c_dev,ACC_THRESH_TAP, 29);
  printf("\nhey3");
  //hvalue--;
  hvalue=(uint8_t*)malloc(30*sizeof(uint8_t));
  hvalue[0]=hvalue0[0];
  for( i=0; i<29; i++)
    hvalue[i+1]=hvalue1[i];
  printf("\nhey4");
  printf("\nACC_DEVID = 0x%02X = %s", hvalue0[0],conv_byte_hex_bin(hvalue0));
  printf("\nACC_THRESH_TAP = 0x%02X = %s", hvalue1[0],conv_byte_hex_bin(hvalue+1));
  printf("\nACC_OFSX = 0x%02X = %s", hvalue[2],conv_byte_hex_bin(hvalue+2));
  printf("\nACC_OFSY = 0x%02X = %s", hvalue[3],conv_byte_hex_bin(hvalue+3));
  printf("\nACC_OFSZ = 0x%02X = %s", hvalue[4],conv_byte_hex_bin(hvalue+4));
  printf("\nACC_DUR = 0x%02X = %s", hvalue[5],conv_byte_hex_bin(hvalue+5));
  printf("\nACC_LATENT = 0x%02X = %s", hvalue[6],conv_byte_hex_bin(hvalue+6));
  printf("\nACC_WINDOW = 0x%02X = %s", hvalue[7],conv_byte_hex_bin(hvalue+7));
  printf("\nACC_THRESH_ACT = 0x%02X = %s", hvalue[8],conv_byte_hex_bin(hvalue+8));
  printf("\nACC_THRESH_INACT = 0x%02X = %s", hvalue[9],conv_byte_hex_bin(hvalue+9));
  printf("\nACC_TIME_INACT = 0x%02X = %s", hvalue[10],conv_byte_hex_bin(hvalue+10));
  printf("\nACC_ACT_INACT_CTL = 0x%02X = %s", hvalue[11],conv_byte_hex_bin(hvalue+11));
  printf("\nACC_THRESH_FF = 0x%02X = %s", hvalue[12],conv_byte_hex_bin(hvalue+12));
  printf("\nACC_TIME_FF = 0x%02X = %s", hvalue[13],conv_byte_hex_bin(hvalue+13));
  printf("\nACC_TAP_AXES = 0x%02X = %s", hvalue[14],conv_byte_hex_bin(hvalue+14));
  printf("\nACC_ACT_TAP_STATUS = 0x%02X = %s", hvalue[15],conv_byte_hex_bin(hvalue+15));
  printf("\nACC_BW_RATE = 0x%02X = %s", hvalue[16],conv_byte_hex_bin(hvalue+16));
  printf("\nACC_POWER_CTL = 0x%02X = %s", hvalue[17],conv_byte_hex_bin(hvalue+17));
  printf("\nACC_INT_ENABLE = 0x%02X = %s", hvalue[18],conv_byte_hex_bin(hvalue+18));
  printf("\nACC_INT_MAP = 0x%02X = %s", hvalue[19],conv_byte_hex_bin(hvalue+19));
  printf("\nACC_INT_SOURCE = 0x%02X = %s", hvalue[20],conv_byte_hex_bin(hvalue+20));
  printf("\nACC_DATA_FORMAT = 0x%02X = %s", hvalue1[20],conv_byte_hex_bin(hvalue1+21));
  printf("\nACC_DATAX0 = 0x%02X = %s", hvalue1[21],conv_byte_hex_bin(hvalue1+22));
  printf("\nACC_DATAX1 = 0x%02X = %s", hvalue1[22],conv_byte_hex_bin(hvalue1+23));
  printf("\nACC_DATAY0 = 0x%02X = %s", hvalue1[23],conv_byte_hex_bin(hvalue1+24));
  printf("\nACC_DATAY1 = 0x%02X = %s", hvalue1[24],conv_byte_hex_bin(hvalue1+25));
  printf("\nACC_DATAZ0 = 0x%02X = %s", hvalue1[25],conv_byte_hex_bin(hvalue1+26));
  printf("\nACC_DATAZ1 = 0x%02X = %s", hvalue1[26],conv_byte_hex_bin(hvalue1+27));
  printf("\nACC_FIFO_CTL = 0x%02X = %s", hvalue[28],conv_byte_hex_bin(hvalue+28));
  printf("\nACC_FIFO_STATUS = 0x%02X = %s", hvalue[29],conv_byte_hex_bin(hvalue+29));
  return 1;
}
















