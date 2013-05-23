////ROS
#include<ros/ros.h>
#include <ros/callback_queue.h>
//msgs
#include <schunk_msgs/Velocities.h>
//Srv
#include <schunk_low/GetVelocities.h>
#include <schunk_low/GetPositions.h>
#include <schunk_low/ControlMode.h>
#include <schunk_low/Stop.h>

//C
#include<stdlib.h>
#include<stdio.h>

///Internal
//self
#include<schunk_high/cartesian_velocity_controller.h>
#include<schunk_high/cartesian_controller.h>

//External packages
#include<schunk_low/schunk_low_control.h> //For Schunk control mode

//Internal algebra
#include<schunk_high/gmatrix.h>
#include<schunk_high/gmatrix_plus.h>
#include<schunk_high/gmatrix_linalg.h>

#include<schunk_high/robot.h>
#include<schunk_high/dualquaternion.h>

//Consts
const double pi  = 3.141592654;
const double pi2 = 1.57079632679489655;

int main(int argc, char** argv){

	ros::init(argc, argv, "schunk_Cartesian_velocity_controller");

	//KINEMATIC PID VALUES
	double Kpv[6] = {0.6,0.6,0.6,0.6,0.6,0.6};
	double Kiv[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
	double Kdv[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
	//SAMPLE TIME
	double T = 0.04;

	//Memory Alloc
	Robot* r = Robot_create();

	///Schunk's DH
	//               theta d        a   alpha
	Robot_addLink(r, 0,   0.300        ,0,  -pi2);
	Robot_addLink(r, 0,   0            ,0,   pi2);
	Robot_addLink(r, 0,   0.328        ,0,  -pi2);
	Robot_addLink(r, 0,   0            ,0,   pi2);
	Robot_addLink(r, 0,   0.2765       ,0,  -pi2);
	Robot_addLink(r, 0,   0            ,0,   pi2);
	Robot_addLink(r, 0,   0.40049      ,0,   0  );

	Cartesian_velocity_controller Cartesian_velocity_controller(r, Kpv, Kiv, Kdv, T);

	Cartesian_velocity_controller.control();

}

void Cartesian_velocity_controller::control(){

	/******************************
				ROS INIT
	*******************************/

	//Ros rate
	ros::Rate ros_rate(1/T);

	/*****************************
	   CHANGE SCHUNK MODE TO ASSURE VELOCITY CONTROL
	 *****************************/
	//Waits for the service to be available
	ros::service::waitForService("schunk_set_control_mode");

	ROS_INFO_STREAM("Changing Schunk's Control Mode");
	if(!changeSchunkMode(CONTROL_MODE_VELOCITY)){return;}
	ROS_INFO_STREAM("Mode Changed!");

	/*****************************
	   UPDATES SCHUNK'S INITIAL POSITION AND VELOCITY
	 *****************************/
	ROS_INFO_STREAM("Where is Schunk and how fast is it?");
	if(!getSchunkPositionsAndVelocitiesAndUpdateThetasAndDThetas()){return;}
	ROS_INFO_STREAM("Schunk found!");


	/******************************
	PRIVATE VARIABLE INITIALIZATION
	*******************************/

	//Update Robot Object
	Robot_updateLinksThetasByMatrix(r,Thetas);

	//Jacobians
	SJ  = Robot_getGJacobian(r);

	/******************************
			LOOP
	*******************************/

	ROS_INFO_STREAM("Starting!");
	//Take a deep breath and GO
	ros_rate.sleep();

	while(!end_control){

		//Last Error
		PGMATRIX_FREE(Err_ant);
		Err_ant = Err;

		/******************************
			/END GET NEW REFERENCES
		*******************************/
		//Current Error
		Err =     PGMATRIX_SUBTRACT_COPY_FREE(D_ref,D,FALSE,FALSE);
		//Error Integral
		aux1 =	  PGMATRIX_MULTIPLY_CONST_COPY_FREE(Err,T,FALSE);
		Err_int = PGMATRIX_ADD_COPY_FREE(Err_int,aux1,TRUE,TRUE);

		//Derivative    aux1 = Kd*(Err-Err_ant)*(1/T)
		aux1 = PGMATRIX_SUBTRACT_COPY_FREE(Err,Err_ant,FALSE,FALSE);
		aux1 = PGMATRIX_MULTIPLY_COPY_FREE(Kd,aux1,FALSE,TRUE);
		aux1 = PGMATRIX_MULTIPLY_CONST_COPY_FREE(aux1,(1/T),TRUE);
		//Integral      aux2 = Ki*(Err_int)
		aux2 = PGMATRIX_MULTIPLY_COPY_FREE(Ki,Err_int,FALSE,FALSE);
		//Proportional  aux3 = Kp*(Err)
		aux3 = PGMATRIX_MULTIPLY_COPY_FREE(Kp,Err,FALSE,FALSE);

		//Error Sum     aux1 = Kd*(Err-Err_ant)*(1/T) + Ki*(Err_int) + Kp*(Err)
		aux1 = PGMATRIX_ADD_COPY_FREE(aux1,aux2,TRUE,TRUE);
		aux1 = PGMATRIX_ADD_COPY_FREE(aux1,aux3,TRUE,TRUE);

		//Pseudo Inverse Calculation
		aux2 = 	PGMATRIX_DAMPED_LEASTSQUARES_PSEUDOINVERSE(SJ,0.01);

		//New DThetas = pinv(SJ)*(Error Sum)
		PGMATRIX_FREE(DThetas);
		DThetas = PGMATRIX_MULTIPLY_COPY_FREE(aux2,aux1,TRUE,TRUE);

		/******************************
			 GET NEW REFERENCES
		*******************************/
		//Receive callback to after update references
		updateReferences(0.1,0.35); //Refs should be received as 1

		/******************************
			COMUNICATE WITH SCHUNK
		*******************************/

		//Sends information to robot to move, then wait and measure
		setSchunkVelocitiesAndSend(0.5);

		//Wait
		ros_rate.sleep();

		///Measure Position and Speed
		//Position and Velocities, THETAS AND DTHETAS ARE UPDATED
		if(!getSchunkPositionsAndVelocitiesAndUpdateThetasAndDThetas()){return;}
		//Print
		printVel();

		/******************************
		/END	COMUNICATE WITH SCHUNK
		*******************************/

		//Updates Robot's Link's Thetas with current position
		Robot_updateLinksThetasByMatrix(r,Thetas);

		//Updates Jacobian with new position
		PGMATRIX_FREE(SJ);
		SJ  = Robot_getGJacobian(r);

		///Recalculation of measured data.
		//End effectors 6DOF velocity
		PGMATRIX_FREE(D);
		D = PGMATRIX_MULTIPLY_COPY_FREE(SJ,DThetas,FALSE,FALSE);
	}

	/******************************
			CLEAN UP
	*******************************/
	controlCleanUp();

	return;

}

/**
 * */
inline bool Cartesian_velocity_controller::printVel(){

	for(int j=0;j<6;j++){
		endEffector_vel.velocities[j] = PGMATRIX_DATA(Err,j+1,1);
	}

	//Updates End Effetor's Header Timestamp
	endEffector_vel.header.stamp = ros::Time::now();
	endEffector_vel_pub.publish(endEffector_vel);
	endEffector_vel_queue.callAvailable(ros::WallDuration(0));

	return true;
}

/**
 * Calls the last velocity callback and updates the PGMATRIX D_ref with the updated values.
 * */
inline void Cartesian_velocity_controller::updateReferences(double ref_gain, double ref_gain2){

	//Calls one of the velocities callbacks in the queue.
	reference_updater_queue.callAvailable(ros::WallDuration(0));

	//Update references matrix
	for(int j=0;j<3;j++){
		PGMATRIX_DATA(D_ref,j+1,1) = ref_gain*this->references[j];
	}
	for(int j=3;j<6;j++){
		PGMATRIX_DATA(D_ref,j+1,1) = ref_gain2*this->references[j];
	}

}

/**
 * Receive schunk_msgs::Velocities and updates this Cartesian_velocity_controller's internal variables. Should be
 * received as references in the interval [-1.0,1.0]
 * */
void Cartesian_velocity_controller::referenceUpdateCallback(const schunk_msgs::Velocities::ConstPtr& msg){

	if(msg->velocities.size() != 6){
		ROS_ERROR_STREAM("Velocity being sent to cartesian_velocity_control's topic is not size 6");
		stopSchunk();
		return;
	}
	for(int i = 0; i < 6; i++){
		if(this->references[i]> 1.0 || this->references[i]< -1.0){
			ROS_ERROR_STREAM("Reference sent bigger than 1.0 or lower than -1.0.");
			stopSchunk();
			return;
		}
		this->references[i] = msg->velocities[i];
	}
}

Cartesian_velocity_controller::~Cartesian_velocity_controller(){

}

Cartesian_velocity_controller::Cartesian_velocity_controller(Robot* r,double KP[], double KI[], double KD[], double T) : Cartesian_controller(r,6,KP,KI,KD,T){

	//Reference Velocities Buffer
	for(int j=0;j<r->dofs;j++){
		this->references.push_back(0.0);
	}

	//Initialize endEffector_vel msg
	for(int j=0;j<6;j++){
		endEffector_vel.velocities.push_back(0.0);
	}

	//Set Queues
	this->reference_updater_nh.setCallbackQueue(&this->reference_updater_queue);
	this->endEffector_vel_nh.setCallbackQueue(&this->endEffector_vel_queue);

	//Subscriber and queue init.
	this->reference_updater_subscriber  = this->reference_updater_nh.subscribe("schunk_cartesian_velocities_control", ros_pub_buffer, &Cartesian_velocity_controller::referenceUpdateCallback, this);

	//Publisher
	this->endEffector_vel_pub   = this->endEffector_vel_nh.advertise<schunk_msgs::Velocities>("schunk_endeffector_velocities", ros_pub_buffer);
}
