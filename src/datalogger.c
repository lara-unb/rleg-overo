#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#if ANU_COMPILE_FOR_XENOMAI
#include <native/mutex.h>
#else
#include <pthread.h>
#endif

#include <pthread.h>
#include "gdatalogger/gqueue.h"
#include "gdatalogger/gmatlabdatafile.h"
#include "gdatalogger/gdatalogger.h"
#include "communication/communication.h"
/*
#include "calibration.h"
#include "estimation.h"
#include "control.h"
*/
#include "datalogger.h"

// Module variables
GDATALOGGER gDataLogger;
unsigned int datalogger_initialized = DATALOGGER_NOT_INITIALIZED;
unsigned int datalogger_running = DATALOGGER_NOT_RUNNING;
#if ANU_COMPILE_FOR_XENOMAI
RT_MUTEX datalogger_mutex;
#else
pthread_mutex_t datalogger_mutex = PTHREAD_MUTEX_INITIALIZER;
#endif

// Internal functions
int datalogger_lock_mutex(void);
int datalogger_unlock_mutex(void);

int datalogger_init(void)
{
    #if DATALOGGER_MODULE_ENABLED
    double enabled_variables[16][1];
    char datalogger_filename[256];
	#if ANU_COMPILE_FOR_XENOMAI
    rt_mutex_create(&datalogger_mutex,"Data Logger Mutex");
	#endif
    datalogger_lock_mutex();

    if(datalogger_initialized == DATALOGGER_INITIALIZED)
    {
        datalogger_unlock_mutex();
        return DATALOGGER_FAILURE;
    }

    datalogger_running = DATALOGGER_NOT_RUNNING;

	// Init datalogger
    strcpy(datalogger_filename, DATALOGGER_FOLDER);
    strcat(datalogger_filename, "/");
    strcat(datalogger_filename, DATALOGGER_FILE_NAME);
    strcat(datalogger_filename, ".mat");
	#if DATALOGGER_ENABLE_NEW_FILE_ENNUMERATION
    char datalogger_tmp[256];
    int i = 0;
    if(datalogger_file_exists(datalogger_filename) == DATALOGGER_SUCCESS) // File exists
    {
    	strcpy(datalogger_tmp, DATALOGGER_FOLDER);
        strcat(datalogger_tmp, "/");
        strcat(datalogger_tmp, DATALOGGER_FILE_NAME);
        strcat(datalogger_tmp, "_%d.mat");
        do
        {
            sprintf(datalogger_filename, datalogger_tmp, ++i);
        }
        while((datalogger_file_exists(datalogger_filename) == DATALOGGER_SUCCESS) && (i<100));
        if(i == 100)
        {
            datalogger_unlock_mutex();
            return DATALOGGER_FAILURE;
        }
    }
	#endif //DATALOGGER_ENABLE_NEW_FILE_ENNUMERATION

    if(!gDataLogger_Init(&gDataLogger, datalogger_filename, NULL))
    {
        datalogger_unlock_mutex();
        return DATALOGGER_FAILURE;
    }

	// Declare variables
	// Arguments for gDataLogger_DeclareVariable:
	// datalogger pointer, variable name, units, number of rows, number of columns, queue size

	// Variable that tells MATLAB which variables are enabled
    enabled_variables[0][0] = (double) DATALOGGER_LOG_TIME;
    enabled_variables[1][0] = (double) DATALOGGER_LOG_EXECTIMES;
    enabled_variables[2][0] = (double) DATALOGGER_LOG_RAW_IMU;
    enabled_variables[3][0] = (double) DATALOGGER_LOG_CALIBRATED_IMU;
    enabled_variables[4][0] = (double) DATALOGGER_LOG_EFFORTS;
    enabled_variables[5][0] = (double) DATALOGGER_LOG_MRA;
    //enabled_variables[12][0] = (double) DATALOGGER_LOG_LOCAL_COORDINATE_SYSTEM;
    //enabled_variables[13][0] = (double) DATALOGGER_LOG_LOCAL_FIELDS;
    //enabled_variables[14][0] = (double) DATALOGGER_LOG_ESTIMATOR;
    //enabled_variables[15][0] = (double) DATALOGGER_LOG_CONTROLLERS;

    gDataLogger_DeclareVariable(&gDataLogger, "enabled_vars", "", 6, 1, 1);
    gDataLogger_InsertVariable(&gDataLogger, "enabled_vars", &enabled_variables[0][0]);

	#if DATALOGGER_LOG_TIME
	gDataLogger_DeclareVariable(&gDataLogger, "t", "s", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
    gDataLogger_DeclareVariable(&gDataLogger, "Ts", "s", 1, 1, 1);
    #endif // DATALOGGER_LOG_TIME

    #if DATALOGGER_LOG_EXECTIMES
    gDataLogger_DeclareVariable(&gDataLogger, "t_control_exec", "s", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
    gDataLogger_DeclareVariable(&gDataLogger, "t_ui_exec", "s", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
    #endif // DATALOGGER_LOG_EXECTIMES

	#if DATALOGGER_LOG_RAW_IMU
	gDataLogger_DeclareVariable(&gDataLogger, "imu_accelerometer_x_raw", "raw", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
	gDataLogger_DeclareVariable(&gDataLogger, "imu_accelerometer_y_raw", "raw", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
	gDataLogger_DeclareVariable(&gDataLogger, "imu_accelerometer_z_raw", "raw", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
	gDataLogger_DeclareVariable(&gDataLogger, "imu_magnetometer_x_raw", "raw", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
	gDataLogger_DeclareVariable(&gDataLogger, "imu_magnetometer_y_raw", "raw", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
	gDataLogger_DeclareVariable(&gDataLogger, "imu_magnetometer_z_raw", "raw", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
	gDataLogger_DeclareVariable(&gDataLogger, "imu_gyrometer_x_raw", "raw", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
	gDataLogger_DeclareVariable(&gDataLogger, "imu_gyrometer_y_raw", "raw", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
	gDataLogger_DeclareVariable(&gDataLogger, "imu_gyrometer_z_raw", "raw", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
    gDataLogger_DeclareVariable(&gDataLogger, "imu_valid_data", "", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
	#endif // DATALOGGER_LOG_IMU
	
    #if DATALOGGER_LOG_EFFORTS
    gDataLogger_DeclareVariable(&gDataLogger, "Vin0", "mV", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
    gDataLogger_DeclareVariable(&gDataLogger, "Vin0_valid_data", "", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
    #endif
    
    #if DATALOGGER_LOG_MRA
    gDataLogger_DeclareVariable(&gDataLogger, "v_ctl", "mV", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
    gDataLogger_DeclareVariable(&gDataLogger, "v_ctl_read", "mV", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
    #endif

    #if DATALOGGER_LOG_CALIBRATED_IMU
    gDataLogger_DeclareVariable(&gDataLogger, "imu_accelerometer_x_g", "g", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
    gDataLogger_DeclareVariable(&gDataLogger, "imu_accelerometer_y_g", "g", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
    gDataLogger_DeclareVariable(&gDataLogger, "imu_accelerometer_z_g", "g", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
    gDataLogger_DeclareVariable(&gDataLogger, "imu_magnetometer_x_B", "B", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
    gDataLogger_DeclareVariable(&gDataLogger, "imu_magnetometer_y_B", "B", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
    gDataLogger_DeclareVariable(&gDataLogger, "imu_magnetometer_z_B", "B", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
    gDataLogger_DeclareVariable(&gDataLogger, "imu_gyrometer_x_rads", "rads", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
    gDataLogger_DeclareVariable(&gDataLogger, "imu_gyrometer_y_rads", "rads", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
    gDataLogger_DeclareVariable(&gDataLogger, "imu_gyrometer_z_rads", "rads", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
    gDataLogger_DeclareVariable(&gDataLogger, "imu_calibrated_valid_data", "", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
    //gDataLogger_DeclareVariable(&gDataLogger, "imu_calibrated_valid_accelerometer_data", "", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
    //gDataLogger_DeclareVariable(&gDataLogger, "imu_calibrated_valid_gyrometer_data", "", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
    //gDataLogger_DeclareVariable(&gDataLogger, "imu_calibrated_valid_magnetometer_data", "", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
    #endif // DATALOGGER_LOG_CALIBRATED_IMU

    datalogger_initialized = DATALOGGER_INITIALIZED;
    datalogger_unlock_mutex();
    #endif //DATALOGGER_MODULE_ENABLED

    return DATALOGGER_SUCCESS;
}

int datalogger_close(void)
{
    #if DATALOGGER_MODULE_ENABLED
	datalogger_lock_mutex();
	if(datalogger_initialized != DATALOGGER_INITIALIZED)
    {
	    datalogger_unlock_mutex();
        return DATALOGGER_ERROR_NOT_INITIALIZED;
    }

    gDataLogger_MatfileUpdate(&gDataLogger); // Update data
    gDataLogger_Close(&gDataLogger); // Close datalogger

    datalogger_initialized = DATALOGGER_NOT_INITIALIZED;
    datalogger_unlock_mutex();

	#if ANU_COMPILE_FOR_XENOMAI
    rt_mutex_delete(&datalogger_mutex);
	#endif	//ANU_COMPILE_FOR_XENOMAI

    #endif //DATALOGGER_MODULE_ENABLED

    return DATALOGGER_SUCCESS;
}

int datalogger_write_file(void)
{
    #if DATALOGGER_MODULE_ENABLED
    datalogger_lock_mutex();
    if(datalogger_initialized != DATALOGGER_INITIALIZED)
    {
        datalogger_unlock_mutex();
        return DATALOGGER_ERROR_NOT_INITIALIZED;
    }
    gDataLogger_MatfileUpdate(&gDataLogger); // Empty buffers in the file
    datalogger_unlock_mutex();
    #endif

    return DATALOGGER_SUCCESS;
}

int datalogger_update_IPC(void)
{
    #if DATALOGGER_MODULE_ENABLED
    datalogger_lock_mutex();
    if(datalogger_initialized != DATALOGGER_INITIALIZED)
    {
        datalogger_unlock_mutex();
        return DATALOGGER_ERROR_NOT_INITIALIZED;
    }

    gDataLogger_IPCUpdate(&gDataLogger); // Manages IPC
    datalogger_unlock_mutex();
    #endif
    return DATALOGGER_SUCCESS;
}

int datalogger_update(double t_s, double t_control_exec_s, double t_ui_exec_s, double t0_s, IMU_DATA_STRUCT *pimu_data, EFF_DATA_STRUCT *peff_data, MRA_DATA_STRUCT *pmra_data/*IMUMEASURE *pimu_measure, MAGNETOMETERMEASURE *pmagnetometer_measure, ESTIMATION_DATA_STRUCT *pestimation_data, CONTROL_DATA_STRUCT *pcontrol_data */)
{
    double tmp = 0.0;

    datalogger_lock_mutex();

    if(datalogger_initialized != DATALOGGER_INITIALIZED)
    {
        datalogger_unlock_mutex();
        return DATALOGGER_ERROR_NOT_INITIALIZED;
    }

    #if DATALOGGER_LOG_TIME
    double tmp_t = t_s - t0_s;
    gDataLogger_InsertVariable(&gDataLogger, "t", &tmp_t);
    #else
    double tmp_t = 0.0;
    tmp_t = t_s; // Remove warning
    #endif // DATALOGGER_LOG_TIME

    #if DATALOGGER_LOG_EXECTIMES
    gDataLogger_InsertVariable(&gDataLogger, "t_control_exec", &t_control_exec_s);
    gDataLogger_InsertVariable(&gDataLogger, "t_ui_exec", &t_ui_exec_s);
    #else
    double tmp_e = 0.0; // Remove warning
    tmp_e = t_control_exec_s; // Remove warning
    tmp_e = t_ui_exec_s; // Remove warning
    #endif // DATALOGGER_LOG_EXECTIMES


    #if DATALOGGER_LOG_RAW_IMU
    tmp = (double) pimu_data->acc.x;
    gDataLogger_InsertVariable(&gDataLogger, "imu_accelerometer_x_raw", &tmp);
    tmp = (double) pimu_data->acc.y;
    gDataLogger_InsertVariable(&gDataLogger, "imu_accelerometer_y_raw", &tmp);
    tmp = (double) pimu_data->acc.z;
    gDataLogger_InsertVariable(&gDataLogger, "imu_accelerometer_z_raw", &tmp);
    tmp = (double) pimu_data->mag.x;
    gDataLogger_InsertVariable(&gDataLogger, "imu_magnetometer_x_raw", &tmp);
    tmp = (double) pimu_data->mag.y;
    gDataLogger_InsertVariable(&gDataLogger, "imu_magnetometer_y_raw", &tmp);
    tmp = (double) pimu_data->mag.z;
    gDataLogger_InsertVariable(&gDataLogger, "imu_magnetometer_z_raw", &tmp);
    tmp = (double) pimu_data->gyr.x;
    gDataLogger_InsertVariable(&gDataLogger, "imu_gyrometer_x_raw", &tmp);
    tmp = (double) pimu_data->gyr.y;
    gDataLogger_InsertVariable(&gDataLogger, "imu_gyrometer_y_raw", &tmp);
    tmp = (double) pimu_data->gyr.z;
    gDataLogger_InsertVariable(&gDataLogger, "imu_gyrometer_z_raw", &tmp);
    tmp = (double) pimu_data->new_data;
    gDataLogger_InsertVariable(&gDataLogger, "imu_valid_data", &tmp);
    #else
    double tmp_ri = 0; // Remove warning
    tmp_ri = pimu_data->new_data; // Remove warning ???
    #endif //DATALOGGER_LOG_RAW_IMU
    
    #if DATALOGGER_LOG_EFFORTS
    tmp = (double) peff_data->F.x;
    gDataLogger_InsertVariable(&gDataLogger, "Vin0", &tmp);
    tmp = (double) peff_data->new_data;
    gDataLogger_InsertVariable(&gDataLogger, "Vin0_valid_data", &tmp);
    #endif
    
    #if DATALOGGER_LOG_MRA
    tmp = (double) pmra_data->v_ctl;
    gDataLogger_InsertVariable(&gDataLogger, "v_ctl", &tmp);
    tmp = (double) pmra_data->v_ctl_read;
    gDataLogger_InsertVariable(&gDataLogger, "v_ctl_read", &tmp);
    #endif
    
    
       
    #if DATALOGGER_LOG_CALIBRATED_IMU
    gDataLogger_InsertVariable(&gDataLogger, "imu_accelerometer_x_g", &(pimu_data->calib.acc.x));
    gDataLogger_InsertVariable(&gDataLogger, "imu_accelerometer_y_g", &(pimu_data->calib.acc.y));
    gDataLogger_InsertVariable(&gDataLogger, "imu_accelerometer_z_g", &(pimu_data->calib.acc.z));
    gDataLogger_InsertVariable(&gDataLogger, "imu_magnetometer_x_B", &(pimu_data->calib.mag.x));
    gDataLogger_InsertVariable(&gDataLogger, "imu_magnetometer_y_B", &(pimu_data->calib.mag.y));
    gDataLogger_InsertVariable(&gDataLogger, "imu_magnetometer_z_B", &(pimu_data->calib.mag.z));
    gDataLogger_InsertVariable(&gDataLogger, "imu_gyrometer_x_rads", &(pimu_data->calib.gyr.x));
    gDataLogger_InsertVariable(&gDataLogger, "imu_gyrometer_y_rads", &(pimu_data->calib.gyr.y));
    gDataLogger_InsertVariable(&gDataLogger, "imu_gyrometer_z_rads", &(pimu_data->calib.gyr.z));
    tmp = (double) pimu_data->new_data;
    gDataLogger_InsertVariable(&gDataLogger, "imu_calibrated_valid_data", &tmp);
    //tmp = pimu_measure->FlagValidAccerometerMeasure;
    //gDataLogger_InsertVariable(&gDataLogger, "imu_calibrated_valid_accelerometer_data", &tmp);
    //tmp = pimu_measure->FlagValidGyrometerMeasure;
    //gDataLogger_InsertVariable(&gDataLogger, "imu_calibrated_valid_gyrometer_data", &tmp);
    //tmp = pmagnetometer_measure->FlagValidMeasure;
    //gDataLogger_InsertVariable(&gDataLogger, "imu_calibrated_valid_magnetometer_data", &tmp);
    #else
    double tmp_ci = 0.0;
    tmp_ci = pimu_measure->ax; // Remove warning
    tmp_ci = pmagnetometer_measure->mx; // Remove warning
    #endif // DATALOGGER_LOG_CALIBRATED_IMU

 
    datalogger_unlock_mutex();

    return DATALOGGER_SUCCESS;
}

int datalogger_set_Ts(double Ts)
{
    static int datalogger_Ts_inserted = DATALOGGER_VARIABLE_NOT_INSERTED;

    datalogger_lock_mutex();

    if(datalogger_initialized != DATALOGGER_INITIALIZED)
    {
        datalogger_unlock_mutex();
        return DATALOGGER_ERROR_NOT_INITIALIZED;
    }

    #if DATALOGGER_LOG_TIME
    if(datalogger_Ts_inserted == DATALOGGER_VARIABLE_NOT_INSERTED)
    {
        datalogger_Ts_inserted = DATALOGGER_VARIABLE_INSERTED;
        gDataLogger_InsertVariable(&gDataLogger, "Ts", &Ts);
    }
    else
   	{
        datalogger_unlock_mutex();
    	return DATALOGGER_VARIBALE_ALREADY_INSERTED;
   	}
    #else
    double tmp = Ts; // Remove warning
    #endif // DATALOGGER_LOG_TIME

    datalogger_unlock_mutex();
    return DATALOGGER_SUCCESS;
}

int datalogger_status(void)
{
    datalogger_lock_mutex();
    if(datalogger_initialized != DATALOGGER_INITIALIZED)
    {
        datalogger_unlock_mutex();
        return DATALOGGER_ERROR_NOT_INITIALIZED;
    }

    datalogger_unlock_mutex();
    return datalogger_running;
}

int datalogger_start(void)
{
    #if DATALOGGER_MODULE_ENABLED
    datalogger_lock_mutex();
    if(datalogger_initialized != DATALOGGER_INITIALIZED)
    {
        datalogger_unlock_mutex();
        return DATALOGGER_ERROR_NOT_INITIALIZED;
    }

    datalogger_running = DATALOGGER_RUNNING;
    datalogger_unlock_mutex();
    #endif

    return DATALOGGER_SUCCESS;
}

int datalogger_stop(void)
{
    #if DATALOGGER_MODULE_ENABLED
    datalogger_lock_mutex();
    if(datalogger_initialized != DATALOGGER_INITIALIZED)
    {
        datalogger_unlock_mutex();
        return DATALOGGER_ERROR_NOT_INITIALIZED;
    }

    datalogger_running = DATALOGGER_NOT_RUNNING;
    datalogger_unlock_mutex();
    #endif

    return DATALOGGER_SUCCESS;
}

int datalogger_file_exists(const char *filename)
{
    FILE *file;
    if((file = fopen(filename, "r")))
    {
        fclose(file);
        return DATALOGGER_SUCCESS;
    }
    return DATALOGGER_FAILURE;
}

int datalogger_lock_mutex(void)
{
	#if ANU_COMPILE_FOR_XENOMAI
	rt_mutex_acquire(&datalogger_mutex,TM_INFINITE);
	#else
	pthread_mutex_lock(&datalogger_mutex);
	#endif
	return DATALOGGER_SUCCESS;
}

int datalogger_unlock_mutex(void)
{
	#if ANU_COMPILE_FOR_XENOMAI
	rt_mutex_release(&datalogger_mutex);
	#else
	pthread_mutex_unlock(&datalogger_mutex);
	#endif
	return DATALOGGER_SUCCESS;
}
