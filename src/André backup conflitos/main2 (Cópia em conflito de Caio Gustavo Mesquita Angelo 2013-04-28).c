#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <ncurses.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h>
#include <math.h>
//#include "gmatrix.h"
//#include "calibration.h"
//#include "protocol.h"
//#include "estimation.h"
//#include "control.h"
#include"communication/communication.h"
#include "datalogger.h"
#include "ui.h"
#include "main2.h"
#include "calibration/calibration.h"

/* This runs tasks in three points:
 * 1) In the while(quittask == 0) loop: background (semi-periodic, usleep governed) tasks (datalogger update)
 * 2) In the int periodic_task_1(void) function, periodic (low-precision), set by the TASK1_PERIOD_US define:
 * (main data acquisition/control task)
 * 3) In the int periodic_task_2(void) function, periodic (low-precision), set by the TASK2_PERIOD_US define:
 * (UI task)
 */

// Configuration
#define TASK1_PERIOD_US     20000 //20ms
#define TASK2_PERIOD_US     100000 //100ms

// Internal functions
void f_timer_task_1(union sigval sigval);
void f_timer_task_2(union sigval sigval);

// Module variables
unsigned char quittask=0;
int acquire=0;
int total = 0;
int failure = 0;

// Task 1: Data acquisition/control task
timer_t timer_task_1;
volatile double t_task_1_global = 0.0;
volatile double T_task_1_exec_global = 0.0;
volatile double T_task_1_mean_global = 0.0;
volatile double T_task_1_min_global = 0.0;
volatile double T_task_1_max_global = 0.0;
volatile int task_1_period_us = TASK1_PERIOD_US;
volatile int flag_task_1_firstexecution = 1;
volatile double t0 = 0.0;

// Task 2: UI task

timer_t timer_task_2;
volatile double t_task_2_global = 0.0;
volatile double T_task_2_exec_global = 0.0;
volatile double T_task_2_mean_global = 0.0;
volatile double T_task_2_min_global = 0.0;
volatile double T_task_2_max_global = 0.0;
volatile int task_2_period_us = TASK2_PERIOD_US;
volatile int flag_task_2_firstexecution = 1;
unsigned int telemetry_mode = UI_NCURSES_MODE;
      
int buff_i=0; //i of buffer
short int buff[3][3][3]; //{acc,gyr,mag}{x,y,z}{i=1,2,3}

// Shared structures
//
  IMU_PARAM_STRUCT imu_param;
  SPI_PARAM_STRUCT spi_param;
  IMU_DATA_STRUCT imu_data;
  EFF_DATA_STRUCT eff_data;
  MRA_DATA_STRUCT mra_data;
// Calibrated data
//IMUMEASURE imu_measure;
//MAGNETOMETERMEASURE magnetometer_measure;

// State estimate
//ESTIMATION_DATA_STRUCT estimation_data;

// Controllers
//CONTROL_DATA_STRUCT control_data;

int main(void)
{
    int status = 0;
    int n = 0;
    int return_value = SUCCESS;
 
// Parameters for IMU and SPI

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
  
  spi_param.mode=0;
  spi_param.speed=46875;
  spi_param.cs=0;
  

  
   // printf("\ngpio186 inicialmente = %d\n",gpio_read(GPIO_S0));
  //printf("gpio10 inicialmente = %d\n",gpio_read(GPIO_SHDN));
  
// Initialization

  if( devices_init(&imu_param, &spi_param, &mra_data)==FAILURE )
  {
    perror("Unsuccesful devices initialization");
    return -1;
  }
  if( datalogger_init()!=SUCCESS )
  {
    perror("Unsuccesful datalogger initialization");
    return -1;
  }
  if( ui_init() != SUCCESS)
  {
    perror("Unsuccesful user interface initialization");
    return -1;
  }

  //gpio_write(GPIO_S0,1);
  //mra_data.Out_0=2275;
  //actuate(spi_param.spi_dev,&mra_data);
 
  //return_value=FAILURE;
  
    // Main Loop
    if(return_value != FAILURE)
    {
	//datalogger_start();
        timer_start_task_1();
        usleep(0.5*task_1_period_us);
        timer_start_task_2();
        usleep(5*task_1_period_us);
	
        while(quittask == 0)
        {
            #if ANU_COMPILE_FOR_OVERO
            #else
            //if(datalogger_status() == DATALOGGER_RUNNING) datalogger_update_IPC();
            #endif

            
            if(++n>100)
            {
	      //printf("n = %d\n",n);
                if(datalogger_status() == DATALOGGER_RUNNING) datalogger_write_file();
                n = 0;
            }
            usleep(5*task_1_period_us);
        }
	datalogger_stop();
        timer_stop_task_1();
        usleep(TASK1_PERIOD_US);
        timer_stop_task_2();
        usleep(TASK2_PERIOD_US);
    }

    status = datalogger_close();
    if(status != DATALOGGER_SUCCESS)
    {
        return_value = FAILURE;
    }

    status = ui_close();
    if(status != SUCCESS)
    {
        return_value = FAILURE;
    }
    
/*
    status = protocol_close();
    if(status != PROTOCOL_SUCCESS)
    {
        return_value = FAILURE;
    }

    status = control_close();
    if(status != CONTROL_SUCCESS)
    {
        return_value = FAILURE;
    }

    status = estimation_close(&estimation_data);
    if(status != ESTIMATION_SUCCESS)
    {
        return_value = FAILURE;
    }
*/
     close(imu_param.i2c_dev);
     close(spi_param.spi_dev);


    return return_value;
}

int periodic_task_1(void)
{
    //static int previous_calibration_status = CALIBRATION_NOT_RUNNING;
    //int current_calibration_status; // Used to detect lowering edge
    static int previous_datalogger_status = DATALOGGER_NOT_RUNNING;
    int current_datalogger_status; // Used to detect rising edge
    char user =0;
   /* int buff_i=0; //i of buffer
    int i, j, k, equal=0;
    short int buff[3][3][3]; //{acc,gyr,mag}{x,y,z}{i=1,2,3}
*/

    // Main communication/control thread

    //Acquire
    total++;
    if( read_all_data(imu_param.i2c_dev, spi_param.spi_dev, &imu_data, &eff_data, &mra_data) != SUCCESS)
      failure++;
        /*buff_i=buff_i%5;
        buff[0][0][buff_i]=imu_data.acc.x;
	buff[0][1][buff_i]=imu_data.acc.y;
	buff[0][2][buff_i]=imu_data.acc.z;
	buff[1][0][buff_i]=imu_data.gyr.x;
	buff[1][0][buff_i]=imu_data.gyr.x;
	buff[1][1][buff_i]=imu_data.gyr.y;
	buff[1][2][buff_i]=imu_data.gyr.z;
	buff[2][0][buff_i]=imu_data.mag.x;
	buff[2][1][buff_i]=imu_data.mag.y;
	buff[2][2][buff_i]=imu_data.mag.z;
	buff_i++;
	
	for( i=0; i<3; i++)
	{
	  
	  for( j=0; j<3; j++)
	    for( k=1; k<3; k++)
	    {
	      if( buff[i][j][k]==buff[i][j][0] )
		equal=1;
	      else
		equal=0;
	    }
	  if( equal==1)
	    i=3;
	}
	if( equal==1 )
	{
	  devices_init(&imu_param, &spi_param, &mra_data);
	  equal=0;
	}*/
/*    if(read_all_data(imu_param.i2c_dev, spi_param.spi_dev, &imu_data, &eff_data) != SUCCESS)
    {
        failure++;
    }
*/
    // Calibrate and Estimate
    calibrate_all(&imu_data);
    //calibration_calibrate_all(&pwm_read_data, &scp1000_data, &battery_data, &gps_data, &imu_data, &pitot_data, &sonar_data, &calibration_local_coordinate_system_data, &calibration_altimeter_data, &calibration_local_fields_data, &gps_measure, &imu_measure, &magnetometer_measure);
    //estimation_update_state_estimate(&estimation_data, &gps_measure, &imu_measure, &magnetometer_measure, &sonar_measure, &calibration_local_fields_data, task_1_period_us/(double)1e6);

    // Control
    mra_data.v_ctl=2275;//+(uint8_t)(400*cosf(t_task_1_global*1000));
    //control_main_task(t_task_1_global, &pwm_write_data, &pwm_read_data, &control_data, &estimation_data);

    // Actuate
    actuate(spi_param.spi_dev, &mra_data);
    //protocol_send_actuation_data(&pwm_write_data);

    // Log
    /*user=getch();
    switch(user){
      case 'e': 
	exit_program();
	break;
      case 'a':
	acquire=1;
	break;
      case 's':
	acquire=0;
	break;
      default:
	break;
    }*/
    current_datalogger_status = datalogger_status();
    if( (current_datalogger_status == DATALOGGER_RUNNING))
    {
        if(previous_datalogger_status == DATALOGGER_NOT_RUNNING) // Rising edge
        {
            datalogger_set_Ts(task_1_period_us/1e6);
            reset_timer();
        }
        datalogger_update(t_task_1_global, T_task_1_exec_global, T_task_2_exec_global, t0, &imu_data, &eff_data, &mra_data /*&imu_measure, &magnetometer_measure, &estimation_data, &control_data*/);
  //printf("\n    %d\t|",i);
 /* printw("%d\t|",eff_data.F.x);
printw("\t%d\t%d\t%d\t|",imu_data.acc.x,imu_data.acc.y,imu_data.acc.z);
printw("\t%d\t%d\t%d\t|",imu_data.mag.x,imu_data.mag.y,imu_data.mag.z);
printw("\t%d\t%d\t%d\t%d",imu_data.gyr.x,imu_data.gyr.y,imu_data.gyr.z, imu_data.temp);
printw("\t%d\t%d",eff_data.new_data,imu_data.new_data);
printw("\t%d\t%d",mra_data.Out_0, mra_data.Read_0);
printw("\n");
refresh();*/
    }
    previous_datalogger_status = current_datalogger_status;

/*    current_calibration_status = calibration_get_status();
    if(current_calibration_status == CALIBRATION_RUNNING)
    {
        calibration_imu_add_sample_bias_estimate(&imu_data);
        calibration_altimeter_add_sample_initial_pressure(&scp1000_data, &calibration_altimeter_data);
    }
    else // Calibration not running
    {
        if(previous_calibration_status == CALIBRATION_RUNNING) // Lowering edge
        {
            calibration_imu_finish_bias_estimate(&imu_data);
            calibration_altimeter_finish_initial_pressure_estimate(&calibration_altimeter_data);
        }
    }
    previous_calibration_status = current_calibration_status;
*/
    return SUCCESS;
}

int periodic_task_2(void)
{

    int i, j, k, equal=0;

    // UI thread
    if(telemetry_mode == UI_NCURSES_MODE)
    {
        ui_update(&imu_data, &eff_data, &mra_data, total, failure);
    }
                
            buff_i=buff_i%5;
        buff[0][0][buff_i]=imu_data.acc.x;
	buff[0][1][buff_i]=imu_data.acc.y;
	buff[0][2][buff_i]=imu_data.acc.z;
	buff[1][0][buff_i]=imu_data.gyr.x;
	buff[1][0][buff_i]=imu_data.gyr.x;
	buff[1][1][buff_i]=imu_data.gyr.y;
	buff[1][2][buff_i]=imu_data.gyr.z;
	buff[2][0][buff_i]=imu_data.mag.x;
	buff[2][1][buff_i]=imu_data.mag.y;
	buff[2][2][buff_i]=imu_data.mag.z;
	buff_i++;
	
	for( i=0; i<3; i++)
	{
	  
	  for( j=0; j<3; j++)
	    for( k=1; k<3; k++)
	    {
	      if( buff[i][j][k]==buff[i][j][0] )
		equal=1;
	      else
		equal=0;
	    }
	  if( equal==1)
	    i=3;
	}
	if( equal==1 )
	{
	  close(imu_param.i2c_dev);
	  close(spi_param.spi_dev);
	  devices_init(&imu_param, &spi_param, &mra_data);
	  equal=0;
	}
    /*if(telemetry_mode == UI_MAVLINK_MODE)
    {
        mavlink_module_update(t_task_1_global, t0, &battery_data, &gps_data, &imu_data, &pitot_data, &pwm_read_data, &pwm_write_data, &scp1000_data, &sonar_data, &gps_measure, &imu_measure, &magnetometer_measure, &estimation_data, &control_data);
    }*/

    return SUCCESS;
}

void f_timer_task_1(union sigval sigval)
{
    sigval.sival_int = 0; // Remove warning
    timer_function_task_1();
}

void f_timer_task_2(union sigval sigval)
{
    sigval.sival_int = 0; // Remove warning
    timer_function_task_2();
}

void timer_start_task_1(void)
{
    struct itimerspec itimer;
    struct sigevent sigev;
    int erno = 0;

    flag_task_1_firstexecution = 1;

    itimer.it_interval.tv_sec=0;
    itimer.it_interval.tv_nsec=task_1_period_us * 1000;
    itimer.it_value=itimer.it_interval;

    memset(&sigev, 0, sizeof (struct sigevent));

    sigev.sigev_value.sival_int = 0;
    sigev.sigev_notify = SIGEV_THREAD;
    sigev.sigev_notify_attributes = NULL;
    sigev.sigev_notify_function = f_timer_task_1;

    if (timer_create(CLOCK_REALTIME, &sigev, &timer_task_1) < 0)
    {
        fprintf(stderr, "[%d]: %s\n", __LINE__, strerror(erno));
        exit(erno);
    }

    if(timer_settime(timer_task_1, 0, &itimer, NULL) < 0)
    {
        fprintf(stderr, "[%d]: %s\n", __LINE__, strerror(erno));
        exit(erno);
    }

}

void timer_start_task_2(void)
{
    struct itimerspec itimer;
    struct sigevent sigev;
    int erno = 0;

    flag_task_2_firstexecution = 1;

    itimer.it_interval.tv_sec=0;
    itimer.it_interval.tv_nsec=task_2_period_us * 1000;
    itimer.it_value=itimer.it_interval;

    memset (&sigev, 0, sizeof (struct sigevent));
    sigev.sigev_value.sival_int = 0;
    sigev.sigev_notify = SIGEV_THREAD;
    sigev.sigev_notify_attributes = NULL;
    sigev.sigev_notify_function = f_timer_task_2;

    if(timer_create(CLOCK_REALTIME, &sigev, &timer_task_2) < 0)
    {
        fprintf(stderr, "[%d]: %s\n", __LINE__, strerror(erno));
        exit(erno);
    }

    if(timer_settime(timer_task_2, 0, &itimer, NULL) < 0)
    {
        fprintf(stderr, "[%d]: %s\n", __LINE__, strerror(erno));
        exit(erno);
    }
}

void timer_stop_task_1(void)
{
    int erno = 0;
    if(timer_delete(timer_task_1) < 0)
    {
        fprintf(stderr, "[%d]: %s\n", __LINE__, strerror(erno));
        exit(erno);
    }
}

void timer_stop_task_2(void)
{
    int erno = 0;
    if(timer_delete(timer_task_2) < 0)
    {
        fprintf(stderr, "[%d]: %s\n", __LINE__, strerror(erno));
        exit(erno);
    }
}

void timer_function_task_1(void)
{
    int status = 0;
    static struct timeval timereset;
    static struct timeval time;
    static struct timeval time_exec_start;
    static struct timeval time_exec_end;
    static double T_task,t_task,t_task_previous;
    static double T_task_mean,T_task_min,T_task_max,T_task_exec;

    gettimeofday(&time_exec_start, NULL);

    // Time calculation
    if(flag_task_1_firstexecution)
    {
        gettimeofday(&timereset, NULL);
        t_task = 0.0;
        T_task_mean = 0.0;
        T_task_min = 0.0;
        T_task_max = 0.0;
        T_task_exec = 0.0;
    }

    gettimeofday(&time, NULL);
    t_task = ((time.tv_sec - timereset.tv_sec) + (time.tv_usec - timereset.tv_usec)*1e-6);
    T_task = t_task - t_task_previous;
    t_task_previous = t_task;

    if(flag_task_1_firstexecution)
    {
        T_task_mean = T_task;
        T_task_min  = 1e200;
        T_task_max  = 0.0;
        t0 = t_task;
    }
    else
    {
        T_task_mean = 0.05*T_task + 0.95*T_task_mean;
        if(T_task < T_task_min) T_task_min  = T_task;
        if(T_task > T_task_max) T_task_max  = T_task;
    }

    // Run the thread
    status = periodic_task_1();

    gettimeofday(&time_exec_end, NULL);
    T_task_exec = ((time_exec_end.tv_sec - time_exec_start.tv_sec) + (time_exec_end.tv_usec - time_exec_start.tv_usec)*1e-6);

    t_task_1_global = t_task;
    T_task_1_mean_global = T_task_mean;
    T_task_1_min_global = T_task_min;
    T_task_1_max_global = T_task_max;
    T_task_1_exec_global = T_task_exec;

    flag_task_1_firstexecution = 0;
}

void timer_function_task_2(void)
{
    int status = 0;
    static struct timeval timereset;
    static struct timeval time;
    static struct timeval time_exec_start;
    static struct timeval time_exec_end;
    static double T_task,t_task,t_task_previous;
    static double T_task_mean,T_task_min,T_task_max,T_task_exec;

    gettimeofday(&time_exec_start, NULL);

    // Time calculation
    if(flag_task_2_firstexecution)
    {
        gettimeofday(&timereset, NULL);
        t_task = 0.0;
        T_task_mean = 0.0;
        T_task_min = 0.0;
        T_task_max = 0.0;
        T_task_exec = 0.0;
    }

    gettimeofday(&time, NULL);
    t_task = ((time.tv_sec - timereset.tv_sec) + (time.tv_usec - timereset.tv_usec)*1e-6);
    T_task = t_task - t_task_previous;
    t_task_previous = t_task;

    if(flag_task_2_firstexecution)
    {
        T_task_mean = T_task;
        T_task_min  = 1e200;
        T_task_max  = 0.0;
    }
    else
    {
        T_task_mean = 0.05*T_task + 0.95*T_task_mean;
        if(T_task < T_task_min) T_task_min  = T_task;
        if(T_task > T_task_max) T_task_max  = T_task;
    }

    // Run the thread
    status = periodic_task_2();

    gettimeofday(&time_exec_end, NULL);
    T_task_exec = ((time_exec_end.tv_sec - time_exec_start.tv_sec) + (time_exec_end.tv_usec - time_exec_start.tv_usec)*1e-6);

    t_task_2_global = t_task;
    T_task_2_mean_global = T_task_mean;
    T_task_2_min_global = T_task_min;
    T_task_2_max_global = T_task_max;
    T_task_2_exec_global = T_task_exec;

    flag_task_2_firstexecution = 0;
}

int reset_timer(void)
{
    t0 = t_task_1_global;
    return SUCCESS;
}

int get_time(double *time_control_task_s, double *Ts_control_task_s, double *mean_time_control_task_s, double *t0_control_task_s)
{
    *time_control_task_s = t_task_1_global;
    *Ts_control_task_s = task_1_period_us/1e6;
    *mean_time_control_task_s = T_task_1_mean_global;
    *t0_control_task_s = t0;
    return SUCCESS;
}

void exit_program(void)
{
	quittask = 1;
	return;
}
