/**
 * Main rotine for control:
 * @author Rafael Lima
 * @version 0.1
 */

#include<stdio.h>
#include"Communication/communication.h"
#include "main.h"
#include "taskScheduler.h"

/*Strutures for tasks:*/
  TASK_S task1,task2;

/*Strutures for configuration*/
  IMU_PARAM_STRUCT imu_param;
  SPI_PARAM_STRUCT spi_param;

/*Strutures for communicate*/
  IMU_DATA_STRUT imu_data;
  ENC_DATA_STRUCT enc_data;

/* UI statistics */
int total = 0;
int failure = 0;

int main(void){
  int return_value = SUCCESS;

/*Setting config values:*/
  // Parameters for IMU
  imu_param.acc.full_res=1;
  imu_param.acc.rate=100;
  imu_param.acc.range=16;
  imu_param.gyr.rate=100;
  imu_param.gyr.lpf_bw=188;
  imu_param.gyr.clk_source='Z';
  imu_param.gyr.act="XYZ";
  imu_param.mag.rate=75;
  imu_param.mag.range=0;
  imu_param.mag.samples_avg=8;
  imu_param.mag.meas_mode=0;
  imu_param.mag.op_mode=0;

  // Parameters for SPI
  spi_param.mode=0;
  spi_param.speed=375000;
  spi_param.cs=0;

  // Task for control UI:
  timer_new_task(task1,ui_task);

/* Initialization:*/
  if(devices_init(&imu_param,&spi_param,&mra_data)!=SUCCESS){
    perror("Unsuccesful devices initialization");
    return FAILURE;
  }

  if(ui_init()!=SUCCESS){
    perror("Unsuccesful user interface initialization");
    return FAILURE;
  }

  while(return_value == SUCCESS){
    task1->runFunction();
  }

  return return_value;
}

void ui_task(){
  ui_update(&imu_data, &eff_data, &mra_data,&enc_data, total, failure);
}

/**
 * Internal function to end program
 */
void exit_program(void){
  quittask = 1;
  return;
}
