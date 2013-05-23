#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "gmatrix.h"
#include "imu.h"
#include "magnetometer.h"
#include "gps.h"
#include "sonar.h"
#include "sensors.h"
#include "calibration.h"
#include "protocol.h"
#include "estimation.h"
#include "control.h"
#include "datalogger.h"
#include "ui.h"
#include "mavlink_module.h"
#include "threads_linux.h"

/* This runs tasks in three points:
 * 1) In the while(quittask == 0) loop: background (semi-periodic, usleep governed) tasks (datalogger update)
 * 2) In the int threads_linux_periodic_task_1(void) function, periodic (low-precision), set by the TASK1_PERIOD_US define:
 * (main data acquisition/control task)
 * 3) In the int threads_linux_periodic_task_2(void) function, periodic (low-precision), set by the TASK2_PERIOD_US define:
 * (UI task)
 */

// Configuration
#define TASK1_PERIOD_US     20000 //20ms
#define TASK2_PERIOD_US     100000 //100ms

// Internal functions
void threads_linux_timer_task_1(union sigval sigval);
void threads_linux_timer_task_2(union sigval sigval);

// Module variables
extern unsigned char quittask;
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

// Shared data structures
// Raw data
BATTERY_DATA_STRUCT battery_data;
GPS_DATA_STRUCT gps_data;
IMU_DATA_STRUCT imu_data;
PITOT_DATA_STRUCT pitot_data;
PWM_READ_DATA_STRUCT pwm_read_data;
PWM_WRITE_DATA_STRUCT pwm_write_data;
SCP1000_DATA_STRUCT scp1000_data;
SONAR_DATA_STRUCT sonar_data;

// Calibrated data
GPSMEASURE gps_measure;
IMUMEASURE imu_measure;
MAGNETOMETERMEASURE magnetometer_measure;
SONARMEASURE sonar_measure;

// Coordinate systems, local fields and pressure
CALIBRATION_ALTIMETER_STRUCT calibration_altimeter_data;
CALIBRATION_LOCAL_COORDINATE_SYSTEM_STRUCT calibration_local_coordinate_system_data;
CALIBRATION_LOCAL_FIELDS_STRUCT calibration_local_fields_data;

// State estimate
ESTIMATION_DATA_STRUCT estimation_data;

// Controllers
CONTROL_DATA_STRUCT control_data;

int threads_linux_init(void)
{
    int status = 0;
    int n = 0;
    int return_value = THREADS_LINUX_SUCCESS;

    // Init data
    status = sensors_init(&battery_data, &gps_data, &imu_data, &pitot_data, &pwm_read_data, &pwm_write_data, &scp1000_data, &sonar_data);
    if(status != SENSORS_SUCCESS)
    {
        return_value = THREADS_LINUX_FAILURE;
    }

    if(return_value != THREADS_LINUX_FAILURE)
    {
        status = calibration_init(&calibration_local_coordinate_system_data, &calibration_local_fields_data, &calibration_altimeter_data);
        if(status != CALIBRATION_SUCCESS)
        {
            return_value = THREADS_LINUX_FAILURE;
        }
    }

    if(return_value != THREADS_LINUX_FAILURE)
    {
        status = gps_init(&gps_measure);
        if(status != 1)
        {
            return_value = THREADS_LINUX_FAILURE;
        }
    }

    if(return_value != THREADS_LINUX_FAILURE)
    {
        status = imu_init(&imu_measure);
        if(status != 1)
        {
            return_value = THREADS_LINUX_FAILURE;
        }
    }

    if(return_value != THREADS_LINUX_FAILURE)
    {
        status = magnetometer_init(&magnetometer_measure);
        if(status != 1)
        {
            return_value = THREADS_LINUX_FAILURE;
        }
    }

    if(return_value != THREADS_LINUX_FAILURE)
    {
    	status = sonar_init(&sonar_measure);
    	if(status != 1)
    	{
    		return_value = THREADS_LINUX_FAILURE;
    	}
    }

    if(return_value != THREADS_LINUX_FAILURE)
    {
        status = estimation_init(ESTIMATION_DO_NOT_ESTIMATE_ACCEL_BIAS, &estimation_data);
        if(status != ESTIMATION_SUCCESS)
        {
            return_value = THREADS_LINUX_FAILURE;
        }
    }

    if(return_value != THREADS_LINUX_FAILURE)
    {
        status = control_init(&control_data);
        if(status != CONTROL_SUCCESS)
        {
            return_value = THREADS_LINUX_FAILURE;
        }
    }

    if(return_value != THREADS_LINUX_FAILURE)
    {
        status = protocol_init();
        if(status != PROTOCOL_SUCCESS)
        {
            return_value = THREADS_LINUX_FAILURE;
        }
    }

    if(return_value != THREADS_LINUX_FAILURE)
    {
        status = ui_init();
        if(status != UI_SUCCESS)
        {
            return_value = THREADS_LINUX_FAILURE;
        }
    }

    if(return_value != THREADS_LINUX_FAILURE)
    {
        status = datalogger_init();
        if(status != DATALOGGER_SUCCESS)
        {
            return_value = THREADS_LINUX_FAILURE;
        }
    }

    // Main Loop
    if(return_value != THREADS_LINUX_FAILURE)
    {
        threads_linux_timer_start_task_1();
        usleep(0.5*task_1_period_us);
        threads_linux_timer_start_task_2();
        usleep(5*task_1_period_us);

        while(quittask == 0)
        {
            #if ANU_COMPILE_FOR_OVERO
            #else
            if(datalogger_status() == DATALOGGER_RUNNING) datalogger_update_IPC();
            #endif
            if(++n>100)
            {
                if(datalogger_status() == DATALOGGER_RUNNING) datalogger_write_file();
                n = 0;
            }
            usleep(5*task_1_period_us);
        }

        threads_linux_timer_stop_task_1();
        usleep(TASK1_PERIOD_US);
        threads_linux_timer_stop_task_2();
        usleep(TASK2_PERIOD_US);
    }

    status = datalogger_close();
    if(status != DATALOGGER_SUCCESS)
    {
        return_value = THREADS_LINUX_FAILURE;
    }

    status = ui_close();
    if(status != UI_SUCCESS)
    {
        return_value = THREADS_LINUX_FAILURE;
    }

    status = protocol_close();
    if(status != PROTOCOL_SUCCESS)
    {
        return_value = THREADS_LINUX_FAILURE;
    }

    status = control_close();
    if(status != CONTROL_SUCCESS)
    {
        return_value = THREADS_LINUX_FAILURE;
    }

    status = estimation_close(&estimation_data);
    if(status != ESTIMATION_SUCCESS)
    {
        return_value = THREADS_LINUX_FAILURE;
    }

    status = magnetometer_close(&magnetometer_measure);
    if(status != 1)
    {
        return_value = THREADS_LINUX_FAILURE;
    }

    status = imu_close(&imu_measure);
    if(status != 1)
    {
        return_value = THREADS_LINUX_FAILURE;
    }
    status = gps_close(&gps_measure);
    if(status != 1)
    {
        return_value = THREADS_LINUX_FAILURE;
    }

    status = calibration_close(&calibration_local_coordinate_system_data, &calibration_local_fields_data, &calibration_altimeter_data);
    if(status != CALIBRATION_SUCCESS)
    {
        return_value = THREADS_LINUX_FAILURE;
    }

    status = sensors_close();
    if(status != SENSORS_SUCCESS)
    {
        return_value = THREADS_LINUX_FAILURE;
    }

    return return_value;
}

int threads_linux_periodic_task_1(void)
{
    static int previous_calibration_status = CALIBRATION_NOT_RUNNING;
    int current_calibration_status; // Used to detect lowering edge
    static int previous_datalogger_status = DATALOGGER_NOT_RUNNING;
    int current_datalogger_status; // Used to detect rising edge

    // Main communication/control thread

    //Acquire
    total++;
    if(protocol_request_all_data(&battery_data, &gps_data, &imu_data, &pitot_data, &pwm_read_data, &pwm_write_data, &scp1000_data, &sonar_data) != PROTOCOL_SUCCESS)
    {
        failure++;
    }

    // Calibrate and Estimate
    calibration_calibrate_all(&pwm_read_data, &scp1000_data, &battery_data, &gps_data, &imu_data, &pitot_data, &sonar_data, &calibration_local_coordinate_system_data, &calibration_altimeter_data, &calibration_local_fields_data, &gps_measure, &imu_measure, &magnetometer_measure);
    estimation_update_state_estimate(&estimation_data, &gps_measure, &imu_measure, &magnetometer_measure, &sonar_measure, &calibration_local_fields_data, task_1_period_us/(double)1e6);

    // Control
    control_main_task(t_task_1_global, &pwm_write_data, &pwm_read_data, &control_data, &estimation_data);

    // Actuate
    protocol_send_actuation_data(&pwm_write_data);

    // Log
    current_datalogger_status = datalogger_status();
    if(current_datalogger_status == DATALOGGER_RUNNING)
    {
        if(previous_datalogger_status == DATALOGGER_NOT_RUNNING) // Rising edge
        {
            datalogger_set_Ts(task_1_period_us/1e6);
            datalogger_set_initial_altimeter_parameters(&calibration_altimeter_data);
            datalogger_set_local_coordinate_system(&calibration_local_coordinate_system_data);
            datalogger_set_local_fields(&calibration_local_fields_data);
            threads_reset_timer();
        }
        datalogger_update(t_task_1_global, T_task_1_exec_global, T_task_2_exec_global, t0, &battery_data, &gps_data, &imu_data, &pitot_data, &pwm_read_data, &pwm_write_data, &scp1000_data, &sonar_data, &gps_measure, &imu_measure, &magnetometer_measure, &estimation_data, &control_data);
    }
    previous_datalogger_status = current_datalogger_status;

    current_calibration_status = calibration_get_status();
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

    return THREADS_LINUX_SUCCESS;
}

int threads_linux_periodic_task_2(void)
{
    // UI thread
    if(telemetry_mode == UI_NCURSES_MODE)
    {
        ui_update(&battery_data, &gps_data, &imu_data, &pitot_data, &pwm_read_data, &pwm_write_data, &scp1000_data, &sonar_data, &gps_measure, &imu_measure, &magnetometer_measure, &calibration_local_coordinate_system_data, &calibration_local_fields_data, &calibration_altimeter_data, &estimation_data, &control_data, total, failure);
    }
    if(telemetry_mode == UI_MAVLINK_MODE)
    {
        mavlink_module_update(t_task_1_global, t0, &battery_data, &gps_data, &imu_data, &pitot_data, &pwm_read_data, &pwm_write_data, &scp1000_data, &sonar_data, &gps_measure, &imu_measure, &magnetometer_measure, &estimation_data, &control_data);
    }

    return THREADS_LINUX_SUCCESS;
}

void threads_linux_timer_task_1(union sigval sigval)
{
    sigval.sival_int = 0; // Remove warning
    threads_linux_timer_function_task_1();
}

void threads_linux_timer_task_2(union sigval sigval)
{
    sigval.sival_int = 0; // Remove warning
    threads_linux_timer_function_task_2();
}

void threads_linux_timer_start_task_1(void)
{
    struct itimerspec itimer;
    struct sigevent sigev;
    int errno = 0;

    flag_task_1_firstexecution = 1;

    itimer.it_interval.tv_sec=0;
    itimer.it_interval.tv_nsec=task_1_period_us * 1000;
    itimer.it_value=itimer.it_interval;

    memset(&sigev, 0, sizeof (struct sigevent));
    sigev.sigev_value.sival_int = 0;
    sigev.sigev_notify = SIGEV_THREAD;
    sigev.sigev_notify_attributes = NULL;
    sigev.sigev_notify_function = threads_linux_timer_task_1;

    if (timer_create(CLOCK_REALTIME, &sigev, &timer_task_1) < 0)
    {
        fprintf(stderr, "[%d]: %s\n", __LINE__, strerror(errno));
        exit(errno);
    }

    if(timer_settime(timer_task_1, 0, &itimer, NULL) < 0)
    {
        fprintf(stderr, "[%d]: %s\n", __LINE__, strerror(errno));
        exit(errno);
    }
}

void threads_linux_timer_start_task_2(void)
{
    struct itimerspec itimer;
    struct sigevent sigev;
    int errno = 0;

    flag_task_2_firstexecution = 1;

    itimer.it_interval.tv_sec=0;
    itimer.it_interval.tv_nsec=task_2_period_us * 1000;
    itimer.it_value=itimer.it_interval;

    memset (&sigev, 0, sizeof (struct sigevent));
    sigev.sigev_value.sival_int = 0;
    sigev.sigev_notify = SIGEV_THREAD;
    sigev.sigev_notify_attributes = NULL;
    sigev.sigev_notify_function = threads_linux_timer_task_2;

    if(timer_create(CLOCK_REALTIME, &sigev, &timer_task_2) < 0)
    {
        fprintf(stderr, "[%d]: %s\n", __LINE__, strerror(errno));
        exit(errno);
    }

    if(timer_settime(timer_task_2, 0, &itimer, NULL) < 0)
    {
        fprintf(stderr, "[%d]: %s\n", __LINE__, strerror(errno));
        exit(errno);
    }
}

void threads_linux_timer_stop_task_1(void)
{
    int errno = 0;
    if(timer_delete(timer_task_1) < 0)
    {
        fprintf(stderr, "[%d]: %s\n", __LINE__, strerror(errno));
        exit(errno);
    }
}

void threads_linux_timer_stop_task_2(void)
{
    int errno = 0;
    if(timer_delete(timer_task_2) < 0)
    {
        fprintf(stderr, "[%d]: %s\n", __LINE__, strerror(errno));
        exit(errno);
    }
}

void threads_linux_timer_function_task_1(void)
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
    status = threads_linux_periodic_task_1();

    gettimeofday(&time_exec_end, NULL);
    T_task_exec = ((time_exec_end.tv_sec - time_exec_start.tv_sec) + (time_exec_end.tv_usec - time_exec_start.tv_usec)*1e-6);

    t_task_1_global = t_task;
    T_task_1_mean_global = T_task_mean;
    T_task_1_min_global = T_task_min;
    T_task_1_max_global = T_task_max;
    T_task_1_exec_global = T_task_exec;

    flag_task_1_firstexecution = 0;
}

void threads_linux_timer_function_task_2(void)
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
    status = threads_linux_periodic_task_2();

    gettimeofday(&time_exec_end, NULL);
    T_task_exec = ((time_exec_end.tv_sec - time_exec_start.tv_sec) + (time_exec_end.tv_usec - time_exec_start.tv_usec)*1e-6);

    t_task_2_global = t_task;
    T_task_2_mean_global = T_task_mean;
    T_task_2_min_global = T_task_min;
    T_task_2_max_global = T_task_max;
    T_task_2_exec_global = T_task_exec;

    flag_task_2_firstexecution = 0;
}

int threads_reset_timer(void)
{
    t0 = t_task_1_global;
    return THREADS_LINUX_SUCCESS;
}

int threads_get_time(double *time_control_task_s, double *Ts_control_task_s, double *mean_time_control_task_s, double *t0_control_task_s)
{
    *time_control_task_s = t_task_1_global;
    *Ts_control_task_s = task_1_period_us/1e6;
    *mean_time_control_task_s = T_task_1_mean_global;
    *t0_control_task_s = t0;
    return THREADS_LINUX_SUCCESS;
}
