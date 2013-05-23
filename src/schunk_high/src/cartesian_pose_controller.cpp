////ROS
#include<ros/ros.h>
#include <ros/callback_queue.h>
//msgs
#include <schunk_msgs/Velocities.h>
#include <schunk_msgs/Positions.h>
//Srv
#include <schunk_low/GetPosVel.h>
#include <schunk_low/ControlMode.h>
#include <schunk_low/Stop.h>

//C
#include<stdlib.h>
#include<stdio.h>

///Internal
//self
#include<schunk_high/cartesian_pose_controller.h>

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

	ros::init(argc, argv, "schunk_cartesian_pose_controller");

	//KINEMATIC PID VALUES
	double Kpv[8] = {1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5};
	double Kiv[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	double Kdv[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	//SAMPLE TIME
	double T = 0.04;

	//Set schunk's initial joint's positions.
	std::vector<double> initialPositions;
	for(int i=0;i<8;i++){initialPositions.push_back(0);}
	//Initial position = [0 , pi2, 0, 0, 0, pi2, 0, 0]
	initialPositions[1] = pi2;
	initialPositions[5] = pi2;

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

	Cartesian_pose_controller cartesian_pose_Controller(initialPositions, r, Kpv, Kiv, Kdv, T);

	cartesian_pose_Controller.control();

}

void Cartesian_pose_controller::control(){
	/******************************

				ROS INIT
	*******************************/

	//Ros rate
	ros::Rate ros_rate(1/T);

	/*****************************
	   CHANGE SCHUNK MODE TO ASSURE POSITION CONTROL
	 *****************************/

	ros::service::waitForService("schunk_set_control_mode");

	ROS_INFO_STREAM("Changing Schunk's Control Mode");
	if(!changeSchunkMode(CONTROL_MODE_POSITION)){return;}
	ROS_INFO_STREAM("Mode Changed to position!");
	ROS_INFO_STREAM("Sending initial position.");
	sendJointPositionsToSchunk(this->initialPositions);
	ROS_INFO_STREAM("Sent! Sleeping...");
	sleep(20);

	/*****************************
	   CHANGE SCHUNK MODE TO ASSURE VELOCITY CONTROL
	 *****************************/

	ROS_INFO_STREAM("Changing Schunk's Control Mode");
	if(!changeSchunkMode(CONTROL_MODE_VELOCITY)){return;}
	ROS_INFO_STREAM("Mode Changed to velocity!");

	/*****************************
	   UPDATES SCHUNK'S INITIAL POSITION AND VELOCITY
	 *****************************/
	ROS_INFO_STREAM("Where is Schunk and how fast is it?");
	if(!getSchunkPositionsAndVelocitiesAndUpdateThetasAndDThetas()){return;}
	ROS_INFO_STREAM("Schunk found!");

	/******************************
	PRIVATE VARIABLE INITIALIZATION
	*******************************/

	//Update Robot Object. Thetas were updated.
	Robot_updateLinksThetasByMatrix(r,Thetas);
	//Update end effectors pose
	DQ* eep = Robot_FKM(r);
	//Current end effectors pose is now the reference pose.
	for(int j=0;j<4;j++){
		this->schunk_pose.positions[j]   = eep->p->v[j];
		this->schunk_pose.positions[j+4] = eep->d->v[j];
	}
	//Update D_ref and D
	updateReferences(1);
	PGMATRIX_SET_COLUMN_BY_DQ(D, 1, eep);
	//Here, both D and D_ref should be the same
	for(int j=0;j<8;j++){
		ROS_INFO_STREAM("Values are : D=" << PGMATRIX_DATA(D,j+1,1) << " and D_ref=" << PGMATRIX_DATA(D_ref,j+1,1) );
	}

	//Jacobians
	SJ  = Robot_getAJacobian(r);

	/******************************
			LOOP
	*******************************/

	ROS_INFO_STREAM("Take a deep breath and GO!");
	//Take a deep breath and GO
	ros_rate.sleep();

	while(ros::ok()){
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
		updateReferences(1);
		/******************************
			COMUNICATE WITH SCHUNK
		*******************************/

		//Sends information to robot to move, then wait and measure
		setSchunkVelocitiesAndSend(0.01);

		//Wait
		ros_rate.sleep();
		///Measure Position and Speed
		//Position and Velocities, THETAS AND DTHETAS ARE UPDATED
		if(!getSchunkPositionsAndVelocitiesAndUpdateThetasAndDThetas()){return;}
		printPose();

		/******************************
		/END	COMUNICATE WITH SCHUNK
		*******************************/

		//Updates Robot's Link's Thetas with current position
		Robot_updateLinksThetasByMatrix(r,Thetas);

		//Updates Jacobian with new position
		PGMATRIX_FREE(SJ);
		SJ  = Robot_getAJacobian(r);

		///Recalculation of measured data.
		//End effectors pose
		DQ_free(eep);
		eep = Robot_FKM(r);
		PGMATRIX_SET_COLUMN_BY_DQ(D, 1, eep);

	}


	/******************************
			CLEAN UP
	*******************************/
	controlCleanUp();

	return;

}

/**
 * */
bool Cartesian_pose_controller::printPose(){

	//for(int j=0;j<8;j++){
	//	schunk_pose.positions[j] = PGMATRIX_DATA(D,j+1,1);
	//}
	//Temp TODO: DELETE ALL THE STUFF BETWEEN ME
	DQ* dq = DQ_create(PGMATRIX_DATA(D,1,1),
			PGMATRIX_DATA(D,2,1),
			PGMATRIX_DATA(D,3,1),
			PGMATRIX_DATA(D,4,1),
			PGMATRIX_DATA(D,5,1),
			PGMATRIX_DATA(D,6,1),
			PGMATRIX_DATA(D,7,1),
			PGMATRIX_DATA(D,8,1));
	Q* trans = DQ_getT(dq);
	Q* rot = DQ_getR(dq);
	for(int j=0;j<4;j++){
		schunk_pose.positions[j]   = trans->v[j];
		schunk_pose.positions[j+4] = rot->v[j];
	}
	DQ_free(dq);
	Q_free(trans);
	Q_free(rot);
	//TODO: AND ME

	schunk_posePublisher.publish(schunk_pose);
	schunk_pose_queue.callAvailable(ros::WallDuration(0));

	return true;
}

/**
 * Calls the last velocity callback and updates the PGMATRIX D_ref with the updated values.
 * */
inline void Cartesian_pose_controller::updateReferences(double ref_gain){

	//Calls one of the velocities callbacks in the queue.
	reference_updater_queue.callAvailable(ros::WallDuration(0));

	//Update references matrix
	for(int j=0;j<8;j++){
		PGMATRIX_DATA(D_ref,j+1,1) = ref_gain*this->schunk_pose.positions[j];
	}

}

/**
 * Receive schunk_msgs::Velocities and updates this Cartesian_controller's internal variables.
 * */
void Cartesian_pose_controller::updateReferencesCallback(const schunk_msgs::Positions::ConstPtr& msg){

	if(msg->positions.size() != 8){
		ROS_ERROR_STREAM("Dualquaternion being sent to schunk_reference_positions's topic is not size 8");
		return;
	}

	for(int i = 0; i < 8; i++){
		this->schunk_pose.positions[i] = msg->positions[i];
	}
}


Cartesian_pose_controller::~Cartesian_pose_controller(){

}

Cartesian_pose_controller::Cartesian_pose_controller(std::vector<double> initialPositions, Robot* r, double KP[], double KI[], double KD[], double T): Cartesian_controller(r,8,KP,KI,KD,T){


	this->initialPositions = initialPositions;

	/******************************
		 PUBLIC VARIABLES
	*******************************/

	//Positiosn vector
	for(int j=0;j<8;j++){
		this->schunk_pose.positions.push_back(0.0);
	}

	/******************************
		  ROS VARIABLES
	*******************************/

	//Set Queues
	this->reference_updater_nh.setCallbackQueue(&this->reference_updater_queue);
	this->schunk_pose_nh.setCallbackQueue(&this->schunk_pose_queue);

	//Subscriber and queue init.
	this->reference_updater_subscriber         = this->reference_updater_nh.subscribe("schunk_reference_positions", ros_pub_buffer, &Cartesian_pose_controller::updateReferencesCallback, this);

	//Publisher
	this->schunk_posePublisher   = this->schunk_pose_nh.advertise<schunk_msgs::Positions>("schunk_endeffector_pose", ros_pub_buffer);

}
