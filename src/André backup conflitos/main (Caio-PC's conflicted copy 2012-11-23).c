#include<stdio.h>
#include"Communication/communication.h"
#include "main.h"

void log_init(FILE *log)
{
  fprintf(log,"Iterations\tADC\t|\t\tACC\t\t|\t\tMAG\t\t|\t\tGYR\t\tT\n"); printf("\nIterations\tADC\t|\t\tACC\t\t|\t\tMAG\t\t|\t\tGYR\t\tT\n");
  fprintf(log,"    #\t|\t0\t|\t X\tY\tZ\t|\t X\tY\tZ\t|\t X\tY\tZ\t\n"); printf("    #\t|\t0\t|\t X\tY\tZ\t|\t X\tY\tZ\t|\t X\tY\tZ\t\n");
  fprintf(log,"____________________________________________________________________________________________________________________________\n");
  printf("____________________________________________________________________________________________________________________________\n");
  return;
}

int main(void)
{
  int i2c_dev, spi_dev;
  FILE *log;
  short int *data;
  short int datap[4];
  int i;
  double value[3];

  if( (i2c_dev = open("/dev/i2c-3", O_RDWR))<0 )
  {
    perror("Failed to open i2c_dev");
    return -1;
  }

// INITIALIZATION OF SENSORS AND DEVICES
  if ((ioctl(i2c_dev, I2C_SLAVE, ADD_HMC5883)) < 0) {
		perror("ioctl(I2C_SLAVE) mag");
		return -1;
  }
  if( (mag_init(i2c_dev, 75, 0, 8, 0, 0))< 0)
  {
      perror("Error in magnetometer initialization");
      return -1;
  }
  if( ioctl(i2c_dev, I2C_SLAVE, ADD_ITG3200) < 0) {
		perror("ioctl(I2C_SLAVE) gyr");
		return -1;
  } 
  if( gyr_init(i2c_dev, 100, 188, 'Z', "XYZ")<0 )
  {
      perror("Error in gyrometer initialization");
      return -1;
  }
  if ( (ioctl(i2c_dev, I2C_SLAVE, ADD_ADXL345)) < 0) {
	perror("ioctl(I2C_SLAVE) acc");
	return -1;
  }  
  if( acc_init(i2c_dev, 1, 100, 16)<0)
  {
	perror("Error in accelerometer initialization");
	return -1;
  }

  if ((spi_dev=spi_init(0,46875,0))<0) 
  {
    perror("Error in SPI device initialization");
    return -1;
  }
  
  if( (log=fopen("log.txt","w"))==NULL )
  {
    perror("Unsuccessful log file openning");
    return -1;
  }
  log_init(log);
  
// Initializing gdatalogger
  if(!gDataLogger_Init(&gDataLogger,"matlabdatafiles/gmatlabdatafile.mat",NULL)){
		printf("\nErro em gDataLogger_Init\n\n");
		return EXIT_FAILURE;
	}

	gDataLogger_DeclareVariable(&gDataLogger,"acc","m/s²",1,3,1000);
	gDataLogger_DeclareVariable(&gDataLogger,"gyr","\][{º/s",1,3,1000);
	gDataLogger_DeclareVariable(&gDataLogger,"mag","mG",1,3,1000);
	gDataLogger_DeclareVariable(&gDataLogger,"temp","K",1,1,1000);
	gDataLogger_DeclareVariable(&gDataLogger,"Vin","V",1,1,1000);
  
// ROUTINE 
  for( i=1; i<=20; i++)
  {  
      fprintf(log,"    %d\t|",i); printf("    %d\t|",i);
  // ADC
/*FALTA IMPLEMENTAR AS ESCRITAS NOS GPIOS RESPONSÁVEIS PELA SELEÇÃO DO CHIP SELECT QUANDO REALMENTE FOR USAR A PLACA*/
      //spi_dev=spi_init(0,46875,0);
      datap[0]=adc_read(spi_dev,0,1);
      fprintf(log,"\t%d\t|",datap[0]);printf("\t%d\t|",datap[0]);
      value[0]=(double)datap[0]/1000;
      gDataLogger_InsertVariable(&gDataLogger,"Vin",&(value[0]));
  // ACCelerometer
      if ( (ioctl(i2c_dev, I2C_SLAVE, ADD_ADXL345)) < 0) {
	perror("ioctl(I2C_SLAVE) acc");
	return -1;
      }  
      if( (acc_read_all_data(i2c_dev,datap))>0 ){
	fprintf(log,"\t%d\t%d\t%d\t|",datap[0],datap[1],datap[2]); printf("\t%d\t%d\t%d\t|",datap[0],datap[1],datap[2]);
        
      }
      else{
	fprintf(log,"\t?\t?\t?\t|"); printf("\t?\t?\t?\t|");}
      value[0]=(double)datap[0]*0.0392;
      value[1]=(double)datap[1]*0.0392;
      value[2]=(double)datap[2]*0.0392;
      gDataLogger_InsertVariable(&gDataLogger,"acc",value);
  // MAGometer    
      if( (ioctl(i2c_dev, I2C_SLAVE, ADD_HMC5883)) < 0) {
	perror("ioctl(I2C_SLAVE) mag");
	return -1;
      } 
      if( (mag_read_all_data(i2c_dev,datap))>0 ){
	fprintf(log,"\t%d\t%d\t%d\t|",datap[0],datap[1],datap[2]); printf("\t%d\t%d\t%d\t|",datap[0],datap[1],datap[2]);}
      else{
	fprintf(log,"\t?\t?\t?\t|"); printf("\t?\t?\t?\t|");}
      value[0]=(double)datap[0]*0.73;
      value[1]=(double)datap[1]*0.73;
      value[2]=(double)datap[2]*0.73;
      gDataLogger_InsertVariable(&gDataLogger,"mag",value);
  // GYRometer    
      if( (ioctl(i2c_dev, I2C_SLAVE, ADD_ITG3200)) < 0) {
	perror("ioctl(I2C_SLAVE) gyr");
	return -1;
      } 
      if( (gyr_read_all_data(i2c_dev,datap))>0 ){
	fprintf(log,"\t%d\t%d\t%d\t%d",datap[0],datap[1],datap[2],datap[3]); printf("\t%d\t%d\t%d\t%d",datap[0],datap[1],datap[2],datap[3]);}
      else{
	fprintf(log,"\t?\t?\t?\t?"); printf("\t?\t?\t?\t?");}
      value[0]=(double)datap[0]*0.0392;
      value[1]=(double)datap[1]*0.0392;
      value[2]=(double)datap[2]*0.0392;
      gDataLogger_InsertVariable(&gDataLogger,"acc",value);
  //---------------------
  // Updating matlab file
  
  //---------------------
      fprintf(log,"\n");printf("\n");
  }
  
// CLOSING FILES
  fclose(log);
  close(i2c_dev);
  close(spi_dev);
return 1;
}
