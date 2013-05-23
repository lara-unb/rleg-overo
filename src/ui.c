#include <ncurses.h> //NCURSES
#include <math.h>
#include <string.h>
//#include "gmatrix.h"
#include "communication/communication.h"
//#include "calibration.h"
//#include "estimation.h"
//#include "control.h"
#include "datalogger.h"
//#if ANU_COMPILE_FOR_XENOMAI // Compile for Xenomai
//    #include "threads_xenomai.h"
//#else // Compile for Linux
//    #include "threads_linux.h"
//#endif
#include "main2.h"
#include "ui.h"

// From main
extern void exit_program(void);

// From threads
extern unsigned int telemetry_mode;

//WINDOW *scr;

int ui_init(void)
{
    #if UI_MODULE_ENABLED
	//NCURSES
	initscr();
	scrollok(stdscr,TRUE);
	//resizeterm(33,100);
	//wresize(scr,32,100);
	//erase();
	timeout(0); //No delay for getch();
    #endif

	return SUCCESS;
}

int ui_close(void)
{
    #if UI_MODULE_ENABLED
	clear();
	printw("Cleaning up...\n");
	refresh();
	endwin();
    #endif

	  return SUCCESS;
}

int ui_update(IMU_DATA_STRUCT *pimu_data, EFF_DATA_STRUCT *peff_data, MRA_DATA_STRUCT *pmra_data, int total, int failure)
{
	unsigned char user_char = 0;

	static enum {UI_OVERVIEW = 0, UI_IMU, UI_EFF, UI_MRA} ui_state;

	erase();
	//wresize(scr,32,100);
	attron(A_STANDOUT);

	mvaddstr(0,15,"RLEG Data");
	attroff(A_STANDOUT);

	switch(ui_state)
	{
		case UI_IMU:
			if(ui_imu_data(pimu_data) != SUCCESS) return FAILURE;
			break;
		case UI_EFF:
			if(ui_eff_data(peff_data) != SUCCESS) return FAILURE;
			break;
		case UI_MRA:
			if(ui_mra_data(pmra_data) != SUCCESS) return FAILURE;
			break;
	/*	case UI_BATTERY:
			if(ui_battery_data(pbattery_data) != SUCCESS) return FAILURE;
			break;
		case UI_LOCAL_DATA:
			if(ui_local_data(plocal_coordinate_system_data, plocal_fields_data, paltimeter_data) != SUCCESS) return FAILURE;
			break;
        case UI_ESTIMATION:
            if(ui_estimation(pestimation_data) != SUCCESS) return FAILURE;
            break;
        case UI_CONTROL:
            if(ui_control(pcontrol_data) != SUCCESS) return FAILURE;
            break;*/
		case UI_OVERVIEW:
        default:
			if(ui_overview_data(total, failure, pimu_data, peff_data, pmra_data) != SUCCESS) return FAILURE;
			break;
	}

	refresh();

	user_char = getch(); //Getting a char typed by the user
	switch(user_char)
	{
		case 'q': //Quit
			exit_program();
			break;
		case 'i': //IMU view
			ui_state = UI_IMU;
			break;
		case 'f': //PWM view
			ui_state = UI_EFF;
			break;
		case 'o': //Overview
			ui_state = UI_OVERVIEW;
			break;
		case 'm': //Battery
			ui_state = UI_MRA;
			break;
		//case 'e': // State estimator
		//    ui_state = UI_ESTIMATION;
		//    break;
        case 'd': //Datalogger start/stop
            if(datalogger_status() == DATALOGGER_RUNNING)
            {
                datalogger_stop();
            }
            else
            {
                datalogger_start();
            }
            break;
	    /*
        case 's': // Start Altimeter/IMU calibration
            if(calibration_get_status() == CALIBRATION_RUNNING)
            {
//                 calibration_stop_calibration();
            }
            else
            {
                calibration_start_calibration();
            }
            break;
		case 'f': //Init local fields
			calibration_init_local_coordinate_system(pgps_data, plocal_coordinate_system_data);
			calibration_init_local_fields(plocal_fields_data, pgps_data);
			break;
		case 't': // Start estimators
			estimation_initial_state_estimate(pestimation_data, pgps_measure, pimu_measure, pmagnetometer_measure, plocal_fields_data);
			break;
		case 'c':
		    ui_state = UI_CONTROL;
		    break;
		case '1':
		    pcontrol_data->roll_proportional_gain += 1.0;
		    break;
		case '2':
            pcontrol_data->roll_proportional_gain -= 1.0;
		    break;
		case '3':
            pcontrol_data->pitch_proportional_gain += 1.0;
            break;
		case '4':
            pcontrol_data->pitch_proportional_gain -= 1.0;
            break;
		case '5':
            pcontrol_data->yaw_proportional_gain += 1.0;
            break;
		case '6':
            pcontrol_data->yaw_proportional_gain -= 1.0;
            break;
		case 'm':
		    telemetry_mode = UI_MAVLINK_MODE;
		    ui_close();
		    break;
		    */
		default:
			break;
	}

	// HaCK! Remove warning
	//double tmp = 0;
	//tmp = pgps_measure->FlagValidPositionMeasure;

	return SUCCESS;
}

int ui_imu_data(IMU_DATA_STRUCT *pimu_data)
{
	mvprintw(2, 0, "Accelerometer X (raw): %d", pimu_data->acc.x);
	mvprintw(2, 40, "Accelerometer X (g): %lf", pimu_data->calib.acc.x);
	mvprintw(3, 0, "Accelerometer Y (raw): %d", pimu_data->acc.y);
	mvprintw(3, 40, "Accelerometer Y (g): %lf", pimu_data->calib.acc.y);
	mvprintw(4, 0, "Accelerometer Z (raw): %d", pimu_data->acc.z);
	mvprintw(4, 40, "Accelerometer Z (g): %lf", pimu_data->calib.acc.z);
	mvprintw(5, 0, "Gyrometer X (raw):     %d", pimu_data->gyr.x);
	mvprintw(5, 40, "Gyrometer X (rad/sec):   %lf", pimu_data->calib.gyr.x);
	mvprintw(6, 0, "Gyrometer Y (raw):     %d", pimu_data->gyr.y);
	mvprintw(6, 40, "Gyrometer Y (rad/sec):   %lf", pimu_data->calib.gyr.y);
	mvprintw(7, 0, "Gyrometer Z (raw):     %d", pimu_data->gyr.z);
	mvprintw(7, 40, "Gyrometer Z (rad/sec):   %lf", pimu_data->calib.gyr.z);
	mvprintw(8, 0, "Magnetometer X (raw):  %d", pimu_data->mag.x);
	mvprintw(8, 40, "Magnetometer X (B):     %lf", pimu_data->calib.mag.x);
	mvprintw(9, 0, "Magnetometer Y (raw):  %d", pimu_data->mag.y);
	mvprintw(9, 40, "Magnetometer Y (B):     %lf", pimu_data->calib.mag.y);
	mvprintw(10,0, "Magnetometer Z (raw):  %d", pimu_data->mag.z);
	mvprintw(10,40, "Magnetometer Z (B):     %lf", pimu_data->calib.mag.z);
	mvprintw(12,0,"Total Acceleromter: %lf",sqrt(pimu_data->calib.acc.x*pimu_data->calib.acc.x+pimu_data->calib.acc.y*pimu_data->calib.acc.y+pimu_data->calib.acc.z*pimu_data->calib.acc.z));
	mvprintw(13,0,"Total Gyrometer: %lf",sqrt(pimu_data->calib.gyr.x*pimu_data->calib.gyr.x+pimu_data->calib.gyr.y*pimu_data->calib.gyr.y+pimu_data->calib.gyr.z*pimu_data->calib.gyr.z));
	mvprintw(14,0,"Total Magnetometer: %lf",sqrt(pimu_data->calib.mag.x*pimu_data->calib.mag.x+pimu_data->calib.mag.y*pimu_data->calib.mag.y+pimu_data->calib.mag.z*pimu_data->calib.mag.z));
	//mvprintw(12, 0, "Accelerometer Magnitude (m/s^2): %lf", pimu_data->accelerometer_magnitude_ms2);
	//mvprintw(13, 0, "Magnetometer Magnitude (uT):     %lf", pimu_data->magnetometer_magnitude_uT);

	return SUCCESS;
}

int ui_eff_data(EFF_DATA_STRUCT *peff_data)
{
	mvprintw(2,0,"Fx (bits): %d",peff_data->F.x);
	//mvprintw(2,40,"Fx (N): %lf",peff_data->??);
	mvprintw(3,0,"Fy (bits): %d",peff_data->F.y);
	//mvprintw(3,40,"Fx (N): %lf",peff_data->??);
	mvprintw(4,0,"Fz (bits): %d",peff_data->F.z);
	//mvprintw(4,40,"Fx (N): %lf",peff_data->??);
	mvprintw(5,0,"Mx (bits): %d",peff_data->M.x);
	//mvprintw(5,40,"Mx (Nm): %lf",peff_data->??);
	mvprintw(6,0,"My (bits): %d",peff_data->M.y);
	//mvprintw(6,40,"My (Nm): %lf",peff_data->??);
	mvprintw(7,0,"Mz (bits): %d",peff_data->M.z);
	//mvprintw(7,40,"Mz (Nm): %lf",peff_data->??);

	return SUCCESS;
}

int ui_mra_data(MRA_DATA_STRUCT *pmra_data)
{
	mvprintw(2,0,"V_ctl (bits): %d",pmra_data->v_ctl);
	mvprintw(3,0,"V_ctl_read (bits): %d",pmra_data->v_ctl_read);
	//mvprintw(2,40,"Fx (N): %lf",peff_data->??);

	return SUCCESS;
}

int ui_overview_data(int total, int failures, IMU_DATA_STRUCT *pimu_data, EFF_DATA_STRUCT *peff_data, MRA_DATA_STRUCT *pmra_data)
{
	float error_rate = 0.0;
	double t = 0.0;
	double Ts = 0.0;
	double mean_exec_time = 0.0;
	double t0 = 0.0;

	get_time(&t, &Ts, &mean_exec_time, &t0);

	error_rate = (float)(((float)failures/(float)total)*100.0);
	mvprintw(2, 0, "Communication Statistics: Total = %d Failures = %d Error Rate = %3.3f",total, failures, error_rate);

	mvprintw(4, 0,  "IMU Accelerometer (bits):\tX:%4d\tY:%4d\tZ:%4d", pimu_data->acc.x, pimu_data->acc.y, pimu_data->acc.z);
	mvprintw(5, 0, "IMU Gyrometer (bits):\t\tX:%4d\tY:%4d\tZ:%4d", pimu_data->gyr.x, pimu_data->gyr.y, pimu_data->gyr.z);
	mvprintw(6, 0, "IMU Magnetometer (bits):\tX:%4d\tY:%4d\tZ:%4d", pimu_data->mag.x, pimu_data->mag.y, pimu_data->mag.z);
	
	mvprintw(8, 0,  "IMU Accelerometer (g):\t\tX:%8.5lf\tY:%8.5lf\tZ:%8.5lf", pimu_data->calib.acc.x, pimu_data->calib.acc.y, pimu_data->calib.acc.z);
	mvprintw(9, 0, "IMU Gyrometer (rad/s):\t\tX:%8.5lf\tY:%8.5lf\tZ:%8.5lf", pimu_data->calib.gyr.x, pimu_data->calib.gyr.y, pimu_data->calib.gyr.z);
	mvprintw(10, 0, "IMU Magnetometer (B):\t\tX:%8.5lf\tY:%8.5lf\tZ:%8.5lf", pimu_data->calib.mag.x, pimu_data->calib.mag.y, pimu_data->calib.mag.z);
	
	mvprintw(12, 0, "Temp (bits):\t%d", pimu_data->temp);
	mvprintw(12,40, "Temp (ºC):\t%lf", pimu_data->calib_temp);

	mvprintw(14,0,"Fx (bits): %d",peff_data->F.x);
	//mvprintw(2,40,"Fx (N): %lf",peff_data->??);
	//mvprintw(15,0,"Fy (bits): %d",peff_data->F.y);
	//mvprintw(3,40,"Fx (N): %lf",peff_data->??);
	//mvprintw(16,0,"Fz (bits): %d",peff_data->F.z);
	//mvprintw(4,40,"Fx (N): %lf",peff_data->??);
	//mvprintw(17,0,"Mx (bits): %d",peff_data->M.x);
	//mvprintw(5,40,"Mx (Nm): %lf",peff_data->??);
	//mvprintw(18,0,"My (bits): %d",peff_data->M.y);
	//mvprintw(6,40,"My (Nm): %lf",peff_data->??);
	//mvprintw(19,0,"Mz (bits): %d",peff_data->M.z);
	//mvprintw(7,40,"Mz (Nm): %lf",peff_data->??);
	
	mvprintw(16, 0, "Voltage Control Written (bits):\t%4d", pmra_data->v_ctl);
	mvprintw(17, 0, "Voltage Control Read (bits):\t%4d", pmra_data->v_ctl_read);

    mvprintw(19, 0, "Runtime: %4.2lf", t);
    if(datalogger_status() == DATALOGGER_NOT_RUNNING)
    {
        mvprintw(19, 40, "Datalogger stopped");
    }
    else
    {
        mvprintw(19, 40, "Datalogger runtime: %4.2lf", (t-t0));
    }
/*
    if(calibration_get_status() == CALIBRATION_NOT_RUNNING)
    {
        mvprintw(15, 0, "Calibration stopped");
    }
    else
    {
        mvprintw(15, 0, "Calibration running");
    }
*/
	mvprintw(21, 0, "I: IMU");
	mvprintw(21, 20, "F: Efforts");
	mvprintw(21, 40, "M: Magneto-rheological Actuator");
	mvprintw(22, 0, "O: Overview");
	mvprintw(22, 20, "D: Datalogger Start/Stop");
	mvprintw(23, 0, "Q: Quit");
    //mvprintw(33, 0, "E: State Estimator");
    //mvprintw(34, 0, "C: Control");


	//mvprintw(24, 40, "B: Battery");
	//mvprintw(25, 40, "L: Local fields/Local gravity");
	//mvprintw(27, 40, "F: Init local fields");
    //mvprintw(28, 40, "S: Calibration Start/Stop");
    //mvprintw(29, 40, "T: Start estimators");

	return SUCCESS;
}
/*
int ui_local_data(CALIBRATION_LOCAL_COORDINATE_SYSTEM_STRUCT *plocal_coordinate_system_data, CALIBRATION_LOCAL_FIELDS_STRUCT *plocal_fields_data, CALIBRATION_ALTIMETER_STRUCT *paltimeter_data)
{
	if(plocal_coordinate_system_data->system_initialized == CALIBRATION_LOCAL_COORDINATE_SYSTEM_INITIALIZED)
	{
		mvprintw(2, 0, "Local coordinate system origin info:");
		mvprintw(3, 0, "Latitude (deg):  %lf", plocal_coordinate_system_data->latitude_radians*180.0/M_PI);
		mvprintw(4, 0, "Longitude (deg): %lf", plocal_coordinate_system_data->longitude_radians*180.0/M_PI);
		mvprintw(5, 0, "Altitude (m):    %lf", plocal_coordinate_system_data->altitude_meters);
		mvprintw(3, 40,"X0 (m):  %lf", plocal_coordinate_system_data->x0);
		mvprintw(4, 40,"Y0 (m):  %lf", plocal_coordinate_system_data->y0);
		mvprintw(5, 40,"Z0 (m):  %lf", plocal_coordinate_system_data->z0);
	}
	else
	{
		mvprintw(2, 0, "Local coordinate system not initialized.");
	}

	if(plocal_fields_data->system_initialized == CALIBRATION_LOCAL_FIELDS_INITIALIZED)
	{
		mvprintw(7,  0, "Local Gravity (NED)");
		mvprintw(8,  0, "X (m/s2):  %lf", PGMATRIX_DATA(plocal_fields_data->local_gravity, 1, 1));
		mvprintw(9,  0, "Y (m/s2):  %lf", PGMATRIX_DATA(plocal_fields_data->local_gravity, 2, 1));
		mvprintw(10, 0, "Z (m/s2):  %lf", PGMATRIX_DATA(plocal_fields_data->local_gravity, 3, 1));
		mvprintw(11, 0, "Magnitude: %lf", plocal_fields_data->gravity_magnitude);
		mvprintw(7, 40, "Local Magnetic (NED)");
		mvprintw(8, 40, "X (uT):    %lf", PGMATRIX_DATA(plocal_fields_data->local_magnetic, 1, 1));
		mvprintw(9, 40, "Y (uT):    %lf", PGMATRIX_DATA(plocal_fields_data->local_magnetic, 2, 1));
		mvprintw(10,40, "Z (uT):    %lf", PGMATRIX_DATA(plocal_fields_data->local_magnetic, 3, 1));
		mvprintw(11,40, "Magnitude: %lf", plocal_fields_data->magnetic_magnitude);
	}
	else
	{
		mvprintw(7, 0, "Local fields not initialized.");
	}

	if(paltimeter_data->system_initialized == CALIBRATION_ALTIMETER_INITIALIZED)
	{
		mvprintw(13, 0, "Altimeter data:");
		mvprintw(14, 0, "Ground pressure (Pa): %lf", paltimeter_data->p0);
	}
	else
	{
		mvprintw(13, 0, "Altimeter not initialized.");
	}

	return UI_SUCCESS;
}

int ui_estimation(ESTIMATION_DATA_STRUCT *pestimator_data)
{
    mvprintw(2, 0, "Roll:   %3.2lf", pestimator_data->roll_angle_radians*180.0/M_PI);
    mvprintw(3, 0, "Pitch:  %3.2lf", pestimator_data->pitch_angle_radians*180.0/M_PI);
    mvprintw(4, 0, "Yaw:    %3.2lf", pestimator_data->yaw_angle_radians*180.0/M_PI);

    mvprintw(6, 0, "X:   %4.1lf", pestimator_data->x_position_meters);
    mvprintw(7, 0, "Y:   %4.1lf", pestimator_data->y_position_meters);
    mvprintw(8, 0, "Z:   %4.1lf", pestimator_data->z_position_meters);

    mvprintw(10, 0, "VX:   %2.2lf", pestimator_data->x_velocity_meters_second);
    mvprintw(11, 0, "VY:   %2.2lf", pestimator_data->y_velocity_meters_second);
    mvprintw(12, 0, "VZ:   %2.2lf", pestimator_data->z_velocity_meters_second);

    mvprintw(14, 0, "Q0:   %1.5lf", pestimator_data->q0);
    mvprintw(15, 0, "Q1:   %1.5lf", pestimator_data->q1);
    mvprintw(16, 0, "Q2:   %1.5lf", pestimator_data->q2);
    mvprintw(17, 0, "Q3:   %1.5lf", pestimator_data->q3);

//    mvprintw(2, 0, "Gyro integration");
//    mvprintw(3, 0, "Roll:   %3.2lf", pestimator_data->roll_angle_radians);
//    mvprintw(4, 0, "Pitch:  %3.2lf", pestimator_data->pitch_angle_radians);
//    mvprintw(5, 0, "Yaw:    %3.2lf", pestimator_data->yaw_angle_radians);
//
//    mvprintw(7, 0, "TRIAD");
//    mvprintw(8, 0, "Roll:   %3.2lf", pestimator_data->x_position_meters);
//    mvprintw(9, 0, "Pitch:  %3.2lf", pestimator_data->y_position_meters);
//    mvprintw(10, 0, "Yaw:    %3.2lf", pestimator_data->z_position_meters);

    return UI_SUCCESS;
}

int ui_control(CONTROL_DATA_STRUCT *pcontrol_data)
{
    mvprintw(2, 0, "Roll Gain:   %3.2lf", pcontrol_data->roll_proportional_gain);
    mvprintw(3, 0, "Pitch Gain:  %3.2lf", pcontrol_data->pitch_proportional_gain);
    mvprintw(4, 0, "Yaw Gain:    %3.2lf", pcontrol_data->yaw_proportional_gain);

    mvprintw(24, 0, "1: Roll +");
    mvprintw(25, 0, "2: Roll -");
    mvprintw(26, 0, "3: Pitch +");

    mvprintw(24, 40, "4: Pitch -");
    mvprintw(25, 40, "5: Yaw +");
    mvprintw(26, 40, "6: Yaw -");

    return UI_SUCCESS;
}*/
