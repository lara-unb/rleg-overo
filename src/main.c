/**
 * Main rotine for control:
 * @author Rafael Lima
 * @version 0.1
 */

#include <stdio.h>
#include <math.h>
#include "communication/communication.h"
#include "main.h"
#include "taskScheduler.h"
#include "ui.h"
//#include "control/control.h"

/*Strutures for tasks:*/
  TASK_S task1;

/* Task period*/
#define TASK_UI_PERIOD 100000 //100ms

/*Strutures for configuration*/
  IMU_PARAM_STRUCT imu_param;
  SPI_PARAM_STRUCT spi_param;

/*Strutures for communicate*/
  EFF_DATA_STRUCT eff_data;
  IMU_DATA_STRUCT imu_data;
  ENC_DATA_STRUCT enc_data;
  MRA_DATA_STRUCT mra_data;

/* UI statistics */
int total = 0;
int failure = 0;
int acquire = 0;

int quittask = 0;
int t0 = 0;

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

  // Periodic task :
  timer_new_task(&task1,main_task);
  
/*Initialization:*/
  if(devices_init(&imu_param,&spi_param,&mra_data)!=SUCCESS){
    perror("Unsuccesful devices initialization");
    return FAILURE;
  }

  if(ui_init()!=SUCCESS){
    perror("Unsuccesful user interface initialization");
    return FAILURE;
  }

  timer_start_task(&task1,TASK1_PERIOD);
  //timer_start_task(task2);

/* Main loop: */
  while(quittask == 0){
    //ui_task(0);
  }

/*Shutting Down:*/
  timer_stop_task(&task1);
  usleep(2000);
  //timer_stop_task(&task2);

  if(ui_close()==FAILURE){
    return_value = FAILURE;
  }

  close(imu_param.i2c_dev);
  close(spi_param.spi_dev);

  return return_value;
}

static void main_task(int signo){
  ui_task();
  control_task();
}

static void ui_task(){
  total++;

  if(read_all_data(imu_param.i2c_dev, spi_param.spi_dev, &imu_data,&eff_data, &mra_data, &enc_data) != SUCCESS) failure++;

  ui_update(&imu_data, &eff_data, &mra_data,&enc_data, total, failure);

}

static void control_task(){
  total++;

/*Input*/
  if(read_all_data(imu_param.i2c_dev, spi_param.spi_dev, &imu_data,&eff_data, &mra_data, &enc_data) != SUCCESS) failure++;
  
/* Control */
  mra_data.v_ctl= 1275 - (uint8_t)(800*cosf(task1.t_global*1000));
  //control_main(task1.t_global,&imu_data,&eff_data,&mra_data);
/* Actuate */
  actuate(spi_param.spi_dev,&mra_data);
}

/// @TODO Review of this function:
int get_time(double *time_control_task_s, double *Ts_control_task_s, double *mean_time_control_task_s, double *t0_control_task_s)
{
  *time_control_task_s = task1.t_global;
  *Ts_control_task_s = (task1.period_us)/1e6;
  *mean_time_control_task_s = task1.T_mean_global;
  *t0_control_task_s = t0;

  return SUCCESS;
}

/**
 * Internal function to end program
 */
void exit_program(void){
  quittask = 1;
  return;
}
