#include <ncurses.h> //NCURSES
#include <math.h>
#include <string.h>
//#include "gmatrix.h"
#include "communication/communication.h"
//#include "calibration/calibration.h"
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

int ui_update(IMU_DATA_STRUCT *pimu_data, EFF_DATA_STRUCT *peff_data, MRA_DATA_STRUCT *pmra_data,ENC_DATA_STRUCT *enc_data, int total, int failure)
{
	unsigned char user_char = 0;

	static enum {UI_OVERVIEW = 0, UI_IMU, UI_EFF, UI_MRA , UI_ENC} ui_state;

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
        case UI_ENC:
          		if(ui_enc_data(enc_data) != SUCCESS) return FAILURE;
			break;
		case UI_OVERVIEW:
	        default:
			if(ui_overview_data(total, failure, pimu_data, peff_data, pmra_data, enc_data) != SUCCESS) return FAILURE;
			break;
	}

	ui_menu();

	refresh();

	user_char = getch(); //Getting a char typed by the user
	switch(user_char)
	{
	    case 'Q':
		case 'q': //Quit
			exit_program();
			break;
		case 'I':
		case 'i': //IMU view
			ui_state = UI_IMU;
			break;
		case 'F':
		case 'f': //EFF view
			ui_state = UI_EFF;
			break;
		case 'E':
 	    case 'e': // ENCODER view
		    ui_state = UI_ENC;
		    break;
		case 'O':
		case 'o': //Overview
			ui_state = UI_OVERVIEW;
			break;
		case 'M':
		case 'm': //MRA
			ui_state = UI_MRA;
			break;
		case 'D':
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
               default:
	         break;
	}

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
	//mvprintw(15, 0, "Encoder (raw): %lf", enc.data);

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

int ui_enc_data(ENC_DATA_STRUCT *enc_data)
{
   mvprintw(2,0,"Position (raw): %lf",enc_data->position);
   mvprintw(3,0,"Position (degree): %lf",enc_data->calib.position);
   return SUCCESS;
}

int ui_mra_data(MRA_DATA_STRUCT *pmra_data)
{
	mvprintw(2,0,"V_ctl (bits): %d",pmra_data->v_ctl);
	mvprintw(3,0,"V_ctl_read (bits): %d",pmra_data->v_ctl_read);
	//mvprintw(2,40,"Fx (N): %lf",peff_data->??);

	return SUCCESS;
}

int ui_overview_data(int total, int failures, IMU_DATA_STRUCT *pimu_data, EFF_DATA_STRUCT *peff_data, MRA_DATA_STRUCT *pmra_data, ENC_DATA_STRUCT *enc_data)
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

	mvprintw(12, 0, "Temp (bits):      %d", pimu_data->temp);
	mvprintw(12,40, "Temp (ºC):     %lf", pimu_data->calib_temp);

	mvprintw(13,0, "Encoder (bits):\t%d",enc_data->position);
	mvprintw(13,40, "Encoder (degree):\t%d",enc_data->calib.position);
	//mvprintw(15,0,"Fx (bits): %d",peff_data->F.x);

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
	
	mvprintw(15, 0, "Voltage Control Written (bits):\t%4d", pmra_data->v_ctl);
	mvprintw(16, 0, "Voltage Control Read (bits):\t%4d", pmra_data->v_ctl_read);

    mvprintw(18, 0, "Runtime: %4.2lf", t);

    if(datalogger_status() == DATALOGGER_NOT_RUNNING)
    {
        mvprintw(18, 40, "Datalogger stopped");
    }
    else
    {
        mvprintw(18, 40, "Datalogger runtime: %4.2lf", (t-t0));
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

	return SUCCESS;
}

int ui_menu(){
	mvprintw(20, 0, "I: IMU");
	mvprintw(20, 20, "F: Efforts");
	mvprintw(20, 40, "M: Magneto-rheological Actuator");
	mvprintw(21, 0, "E: Encoder");
	mvprintw(21, 20, "O: Overview");
	mvprintw(21, 40, "D: Datalogger Start/Stop");
	mvprintw(22, 0, "Q: Quit\n");

	return SUCCESS;
}
