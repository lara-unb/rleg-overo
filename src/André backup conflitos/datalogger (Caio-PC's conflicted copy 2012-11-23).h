#ifndef DATALOGGER_H
#define DATALOGGER_H

// Module configuration
#define DATALOGGER_MODULE_ENABLED      1
#define DATALOGGER_DEBUG_MODE          0

// Sets if you want to overwrite (0) or to append _1, _2, etc. to each new datalogger file
#define DATALOGGER_ENABLE_NEW_FILE_ENNUMERATION		1
#define DATALOGGER_FILE_NAME            "rleg"
#define DATALOGGER_FOLDER               "matlabdatafiles"
#define DATALOGGER_STANDARD_QUEUE_SIZE     750

// What should be logged?
#define DATALOGGER_LOG_TIME                         	1
#define DATALOGGER_LOG_EXECTIMES                    	1
#define DATALOGGER_LOG_RAW_IMU                      	1
#define DATALOGGER_LOG_ANALOG_INPUTS			1
//define DATALOGGER_LOG_LOCAL_COORDINATE_SYSTEM	    1
//#define DATALOGGER_LOG_LOCAL_FIELDS                 1
//#define DATALOGGER_LOG_CALIBRATED_IMU               1
//#define DATALOGGER_LOG_ESTIMATOR                    1
//#define DATALOGGER_LOG_CONTROLLERS                  1

// Internal
#define DATALOGGER_NOT_INITIALIZED                  0
#define DATALOGGER_INITIALIZED                      1
#define DATALOGGER_NOT_RUNNING                      0
#define DATALOGGER_RUNNING                          1
#define DATALOGGER_VARIABLE_NOT_INSERTED            0
#define DATALOGGER_VARIABLE_INSERTED                1

// Returns
#define DATALOGGER_SUCCESS		                1
#define DATALOGGER_FAILURE		                -1
#define DATALOGGER_ERROR_NOT_INITIALIZED        -2
#define DATALOGGER_VARIBALE_ALREADY_INSERTED    -3

int datalogger_init(void);
int datalogger_close(void);
int datalogger_write_file(void);
int datalogger_update_IPC(void);
int datalogger_update(double t_s, double t_control_exec_s, double t_ui_exec_s, double t0_s, IMU_DATA_STRUCT *pimu_data, /*IMUMEASURE *pimu_measure, MAGNETOMETERMEASURE *pmagnetometer_measure, ESTIMATION_DATA_STRUCT *pestimation_data, CONTROL_DATA_STRUCT *pcontrol_data*/);
int datalogger_set_local_fields(CALIBRATION_LOCAL_FIELDS_STRUCT *plocal_fields_data);
int datalogger_set_local_coordinate_system(CALIBRATION_LOCAL_COORDINATE_SYSTEM_STRUCT *plocal_coordinate_system_data);
int datalogger_set_Ts(double Ts);
int datalogger_status(void);
int datalogger_start(void);
int datalogger_stop(void);
int datalogger_file_exists(const char * filename);

#endif //DATALOGGER_H
