////ROS
#include<ros/ros.h>
#include <ros/callback_queue.h>
//msgs
#include <schunk_msgs/Velocities.h>
//Srv
#include <schunk_low/GetPosVel.h>
#include <schunk_low/ControlMode.h>
#include <schunk_low/Stop.h>

//C
#include<stdlib.h>
#include<stdio.h>

///Other packages
#include<schunk_low/schunk_low_control.h> //For velocities callback msgs password

///Internal
//self
#include<schunk_high/cartesian_controller.h>

//Internal algebra
#include<schunk_high/gmatrix.h>
#include<schunk_high/gmatrix_plus.h>
#include<schunk_high/gmatrix_linalg.h>

#include<schunk_high/robot.h>
#include<schunk_high/dualquaternion.h>

/**
 * Calls schunk_low service to change Schunk's control mode.
 * @param mode Schunk's mode you want to change to.
 * @return true if the service was called succesfully, false otherwise.
 * */
bool Cartesian_controller::changeSchunkMode(int mode){

	//Create Service Object and initialize
	schunk_modeChanger_msg.request.mode = mode;

	//Call Service with initialized Service Object
	if (!schunk_modeChanger_client.call(schunk_modeChanger_msg))
	{
		ROS_ERROR_STREAM("Unable to change schunk_low control mode, aborting.");
		stopSchunk();
		return false;
	}

	modeChanger_queue.callAvailable(ros::WallDuration(0));

	return true;
}

/**
 * */
bool Cartesian_controller::stopSchunk(){

	ROS_ERROR_STREAM("STOPPING SCHUNK NOW!");

	//Call end to the rest of the program
	end_control = true;

	//Call Service with initialized Service Object
	if (!schunk_stopper_client.call(schunk_stopper_msg))
	{
		ROS_ERROR_STREAM("FAILED TO STOP SCHUNK! WILL TRY FOREVER");
		stopSchunk();
		return false;
	}

	stopper_queue.callAvailable(ros::WallDuration(0));

	return true;
}


/**
 * Updates the Schunk velocities msg to be sent to Schunk, but not before
 * saturating it. Other work in the velocities msg before sending
 * should be done here.
 * */
void Cartesian_controller::setSchunkVelocitiesAndSend(double sat_vel){

	for(int j = 0; j < r->dofs; j++){
		schunk_low_velocities_msg.velocities[j]=PGMATRIX_DATA(DThetas,j+1,1);

		if(schunk_low_velocities_msg.velocities[j] > sat_vel){
			schunk_low_velocities_msg.velocities[j] = sat_vel;
			PGMATRIX_DATA(DThetas,j+1,1) = sat_vel;
		}
		else if(schunk_low_velocities_msg.velocities[j] < -sat_vel){
			schunk_low_velocities_msg.velocities[j] = -sat_vel;
			PGMATRIX_DATA(DThetas,j+1,1) = -sat_vel;
		}
	}

	//Send
	schunk_low_velocities_msg.header.stamp = ros::Time::now();
	schunk_low_pub.publish(schunk_low_velocities_msg);
	publisher_queue.callAvailable(ros::WallDuration(0));

}

/**
 *
 * */
void Cartesian_controller::sendJointPositionsToSchunk(std::vector<double> positions){

	for(int j = 0; j < 8; j++){
		schunk_low_positions_msg.positions[j]=positions[j];
		ROS_INFO_STREAM("Positions sent : " << j << " = " << positions[j]);
	}

	//Send
	// Update schunk_low_positions_msg header
	schunk_low_positions_msg.header.stamp = ros::Time::now();
	// Publish
	schunk_low_pos_pub.publish(schunk_low_positions_msg);
	// Call!
	pos_publisher_queue.callAvailable(ros::WallDuration(0));

}

/**
 * Asks Schunk's its joint's positions and velocities, them it updates the internal PGMATRIXs Theta and DThetas.
 * @return true if the service was called succesfully, false otherwise.
 * */
bool Cartesian_controller::getSchunkPositionsAndVelocitiesAndUpdateThetasAndDThetas(){

	if(!schunk_posVelGetter_client.call(schunk_posVelGetter_msg)){
		ROS_ERROR_STREAM("Couldn't get schunk's positions and velocities, aborting");
		stopSchunk();
		return false;
	}
	else{
		for(int i = 0; i < r->dofs; i++){
			PGMATRIX_DATA(Thetas,i+1,1) = schunk_posVelGetter_msg.response.positions[i];
			PGMATRIX_DATA(DThetas,i+1,1) = schunk_posVelGetter_msg.response.velocities[i];
		}
	}

	posVelGetter_queue.callAvailable(ros::WallDuration(0));
	return true;
}

/**
 * De-allocs unfreed matrices in the end of the control loop.
 * */
void Cartesian_controller::controlCleanUp(){

	//Last Memory Free Procedure.
	//Jacobians
	PGMATRIX_FREE(SJ);
	//Errors
	PGMATRIX_FREE(Err);
	PGMATRIX_FREE(Err_ant);
	PGMATRIX_FREE(Err_int);
	//Var
	PGMATRIX_FREE(DThetas);
	PGMATRIX_FREE(Thetas);
	PGMATRIX_FREE(D_ref);
	PGMATRIX_FREE(D);
	//PID
	PGMATRIX_FREE(Kp);
	PGMATRIX_FREE(Ki);
	PGMATRIX_FREE(Kd);
	//Robot
	Robot_free(r);

}

Cartesian_controller::~Cartesian_controller(){
}

Cartesian_controller::Cartesian_controller(Robot* r, int refSize, double KP[], double KI[], double KD[], double T){

	/******************************
		 VARIABLES
	*******************************/

	this->r = r;
	this->end_control = false;

	/******************************
		  ROS VARIABLES
	*******************************/

	//Consts (No reason having a queue)
	ros_pub_buffer = 1;

	//Set Queues
	this->schunk_low_modeChanger_nh.setCallbackQueue(&this->modeChanger_queue);
	this->schunk_low_posVelGetter_nh.setCallbackQueue(&this->posVelGetter_queue);
	this->schunk_low_stopper_nh.setCallbackQueue(&this->stopper_queue);

	this->schunk_low_pos_publisher_nh.setCallbackQueue(&this->pos_publisher_queue);
	this->schunk_low_publisher_nh.setCallbackQueue(&this->publisher_queue);

	//Publisher
	this->schunk_low_pub                       = this->schunk_low_publisher_nh.advertise<schunk_msgs::Velocities>("schunk_velocities_control", ros_pub_buffer);
	this->schunk_low_pos_pub                   = this->schunk_low_pos_publisher_nh.advertise<schunk_msgs::Positions>("schunk_positions_control", ros_pub_buffer);

	//ServiceClients
	this->schunk_modeChanger_client  	       = this->schunk_low_modeChanger_nh.serviceClient<schunk_low::ControlMode>("schunk_set_control_mode");
	this->schunk_posVelGetter_client 	       = this->schunk_low_posVelGetter_nh.serviceClient<schunk_low::GetPosVel>("schunk_get_pos_vel", true);
	this->schunk_stopper_client                = this->schunk_low_stopper_nh.serviceClient<schunk_low::Stop>("schunk_stop");

	//Velocity object TODO: change this to something not constant.
	for(int i = 0; i < 8; i++){
		this->schunk_low_velocities_msg.velocities.push_back(0.0);
		this->schunk_low_positions_msg.positions.push_back(0.0);
	}

	//Velocity topic password
	this->schunk_low_velocities_msg.password = SchunkLowControl::kPasswordVelocityCallback;
	this->schunk_low_positions_msg.password  = SchunkLowControl::kPasswordPositionCallback;

	/******************************
		KINEMATIC VARIABLES
	*******************************/

	this->T = T;

	//Jacobian
	this->SJ = NULL;
	//PID Errors Initialize.
	this->Kp = PGMATRIX_CREATE_DIAG_FROM_ARRAY(refSize,KP);
	this->Ki = PGMATRIX_CREATE_DIAG_FROM_ARRAY(refSize,KI);
	this->Kd = PGMATRIX_CREATE_DIAG_FROM_ARRAY(refSize,KD);

	//Thetas.
	this->Thetas = PGMATRIX_CREATE_ZEROES(r->dofs,1);
	this->DThetas = PGMATRIX_CREATE_ZEROES(r->dofs,1);
	//Reference Signal.
	this->D_ref = PGMATRIX_CREATE_ZEROES(refSize,1);
	//Measured Values
	this->D = PGMATRIX_CREATE_ZEROES(refSize,1);
	//Errors
	this->Err     = PGMATRIX_CREATE_ZEROES(refSize,1);
	this->Err_ant = PGMATRIX_CREATE_ZEROES(refSize,1);
	this->Err_int = PGMATRIX_CREATE_ZEROES(refSize,1);
	//Aux
	this->aux1 = NULL;
	this->aux2 = NULL;
	this->aux3 = NULL;

	/******************************
		DUAL MODE VARIABLES
	*******************************/
	//int current_operation_mode = this->HIGH_MODE;

}
