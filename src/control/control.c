#include "control.h"

/** Function: control_main
 * Summary:      Calculate the control signal
 *
 * @param  t_task_1_global containing the ...
 * @param  imu_data containing data from accelerometer, gyrometer and magnetometer
 * @param  eff_data containing data from load cell
 * @param  mra_data struct containing the control signal
 *
 * @return  nothing
 *
void control_main (double time_global, IMU_DATA_STRUCT *imu_data, MRA_DATA_STRUCT *mra_data , ENC_DATA_STRUCT *enc_data){
	int t_gait = 0;
	int gait_cycle = 2000000;//Gait cycle takes 2 seconds
	int heelstrike_threshold = 3;

	//In case of heel strike the gait cycle starts
	//Heel strike detection: module of acceleration vector (which has components in x and z axis) higher than threshold.
	if ((((imu_data->acc.x)^2 + (imu_data->acc.z)^2)^0.5) > heelstrike_threshold){
	  t_gait = 0;
	//Between 10% and 60% of the gait cycle, actuation voltage is 0V
	} else if (t_gait>(0.1*gait_cycle) && t_gait<=(0.6*gait_cycle)){
	  //control voltage 0V
	  mra_data.v_ctl=0;
	//More than 60% of the gait cycle, set actuation voltage to 4V.
	} else if (t_gait>(0.6*gait_cycle) && t_gait<(0.8*gait_cycle)){
	  mra_data.v_ctl=4000;
	}

}


/*
 * @brief Calculate the control signal
 *
void control_byKneeAngle(&ang_data,&mra_data){
  if((ang_data->velocity)>0){
    mra_data.v_ctl = 4000;
  }

  if((ang_data->ang -ANG_MAX)< 0.1)
    mra_data.v_ctl = 6000;
}
*/

/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param time_global [description]
 * @param imu_data [description]
 * @param mra_data [description]
 * @param enc_data [description]
 * 
 * @TODO do some PWM control.
 */
void control_test(double time_global, IMU_DATA_STRUCT *imu_data, MRA_DATA_STRUCT *mra_data , ENC_DATA_STRUCT *enc_data){
	mra_data->v_ctl = (((int)time_global)%200 > 100)? V_CTRL_MAX:0;
}