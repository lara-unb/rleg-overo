/*Function: control_main
 * Summary:      Calculate the control signal
 * Parameters:   t_task_1_global containing the ...
 *               imu_data containing data from accelerometer, gyrometer and magnetometer
 *               eff_data containing data from load cell
 *               mra_data struct containing the control signal
 * Return:       nothing
 */

void control_main (t_task_1_global, &imu_data, &eff_data, &mra_data){
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
