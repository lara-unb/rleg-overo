////ROS
#include<ros/ros.h>
//msgs
#include <schunk_msgs/PositionsDuration.h>
#include <schunk_msgs/PosVelAcc.h>
#include <schunk_msgs/Velocities.h>
//Srv
#include <schunk_low/GetPositions.h>
#include <schunk_low/ControlMode.h>
#include <schunk_low/Stop.h>

//Test
#include<time.h>
#include<vector>

//C
#include<stdlib.h>
#include<stdio.h>

//Internal
#include<schunk_high/gmatrix.h>
#include<schunk_high/gmatrix_plus.h>
#include<schunk_high/gmatrix_linalg.h>

#include<schunk_high/robot.h>
#include<schunk_high/dualquaternion.h>


#ifndef PI
	#define PI (GMATRIXCONST_PI)
#endif
#ifndef PI2
	#define PI2 (1.57079632679489655000)
#endif
#ifndef FALSE
	#define FALSE 0
#endif
#ifndef TRUE
	#define TRUE 1
#endif

void Main_robot_init(Robot* r);

int main(int argc, char** argv) {

	std::vector<ros::Duration> diffs;

	//ROS RELATED
	int ros_pub_buffer = 100;

	//Init ROS
	ros::init(argc, argv, "schunk_high");

	//Return
	//ROS_ERROR_STREAM("SCHUNK_HIGH IS NOT SAFELY WORKING YET"); return 0;

	ROS_INFO_STREAM("Initializing...");

	//Node handle
	ros::NodeHandle ros_node;

	//Publishers
	ros::Publisher ros_pub = ros_node.advertise<schunk_msgs::PosVelAcc>("schunk_posvelacc_control", ros_pub_buffer);
	ros::Publisher ros_velpub = ros_node.advertise<schunk_msgs::Velocities>("schunk_velocities_control", ros_pub_buffer);

	//Subscribers
	ros::ServiceClient ros_client = ros_node.serviceClient<schunk_low::ControlMode>("schunk_set_control_mode");
	ros::ServiceClient schunk_getPositions = ros_node.serviceClient<schunk_low::GetPositions>("schunk_get_positions", true);

	//Set control mode to posVelAcc to we can place Schunk in the initial position.
	schunk_low::ControlMode ros_controlMode;
	ros_controlMode.request.mode = 1;
	if (!ros_client.call(ros_controlMode))
	{
		ROS_ERROR_STREAM("Unable to change schunk_low control mode.");
		return 1;
	}
	sleep(1);

	//Set get positions Service Object
	schunk_low::GetPositions ros_getPositions;

	//Set send msgs
	schunk_msgs::PosVelAcc ros_posDuration;
	for(int i = 0; i < 8; i++){
		ros_posDuration.positions.push_back(0.0);
		ros_posDuration.velocities.push_back(0.1);
		ros_posDuration.accelerations.push_back(1.0);
	}
	ros_posDuration.velocities[0]=0.5;

	ROS_INFO_STREAM("Done!");
	ROS_INFO_STREAM("Moving Schunk to Initial position.");

	//Initial position for insertion
	double theta_init[] = {0.7,0.0,0,PI2,0,PI2,0};

	//Initialize and move schunk
	ros_posDuration.positions[0]=theta_init[0];
	ros_posDuration.positions[1]=theta_init[1];
	ros_posDuration.positions[2]=theta_init[2];
	ros_posDuration.positions[3]=theta_init[3];
	ros_posDuration.positions[4]=theta_init[4];
	ros_posDuration.positions[5]=theta_init[5];
	ros_posDuration.positions[6]=theta_init[6];
	ros_posDuration.positions[7]=0.0;

	ros_pub.publish(ros_posDuration);
	ros::spinOnce();
	//Wait for schunk to reach its position
	sleep(16);//In seconds

	//Sees where schunk really is
	ROS_INFO_STREAM("Where is Schunk?");
	if(!schunk_getPositions.call(ros_getPositions)){
		ROS_ERROR_STREAM("Couldn't get schunk's positions, aborting");
		return 1;
	}
	else{
		theta_init[0] = ros_getPositions.response.positions.positions[0];
		theta_init[1] = ros_getPositions.response.positions.positions[1];
		theta_init[2] = ros_getPositions.response.positions.positions[2];
		theta_init[3] = ros_getPositions.response.positions.positions[3];
		theta_init[4] = ros_getPositions.response.positions.positions[4];
		theta_init[5] = ros_getPositions.response.positions.positions[5];
		theta_init[6] = ros_getPositions.response.positions.positions[6];
	}

	ROS_INFO_STREAM("Schunk found!");

	//Change mode to VELOCITY
	ROS_INFO_STREAM("Changing mode to velocity to begin control!");
	ros_controlMode.request.mode = 0;
	if (!ros_client.call(ros_controlMode))
	{
		ROS_ERROR_STREAM("Unable to change schunk_low control mode to velocity.");
		return 1;
	}

	ROS_INFO_STREAM("Done!");

	//Set velocity msgs
	schunk_msgs::Velocities ros_velocities;
	for(int i = 0; i < 8; i++){
		ros_velocities.velocities.push_back(0.0);
	}

	double vz_ref = -0.02;


	double L = 0.1;
	double T = 0.01;

	double Kpv[] = {2,2,2,0.1};
	double Kiv[] = {2,20,20,20};
	double Kdv[] = {0,0,0,0};

	//ROS RATE
	ros::Rate ros_rate(1/T);

	/******************************
	PRIVATE VARIABLE DECLARATION
	*******************************/
	int i = 0;
	///The Robot.
	Robot* r = NULL;
	///Jacobians
	PGMATRIX GJ = NULL;     ///Geometrical Jacobian.
	PGMATRIX OJP = NULL; 	///Orientation Jacobian Angle P.
	PGMATRIX SJ = NULL;     ///System Jacobian.
	///PID errors.
	PGMATRIX Kp = NULL;
	PGMATRIX Ki = NULL;
	PGMATRIX Kd = NULL;
	///Thetas.
	PGMATRIX Thetas = NULL;
	PGMATRIX DThetas = NULL;
	///Reference Signal.
	PGMATRIX D_ref = NULL;
	PGMATRIX Vz = NULL;
	double vz = 0;
	//Measured Values
	PGMATRIX D = NULL;
	double pitchAngle = 0; //Angle Pitch.
	double dist = 0;
	///Quaternions
	DQ* efp = NULL; 	//End Effector's pose.
	Q* eft = NULL;      //End Effector's translation.
	//Insertion Lengths
	double l = 0;
	//Errors
	PGMATRIX Err = NULL;
	PGMATRIX Err_ant = NULL;
	PGMATRIX Err_int = NULL;
	//Aux
	PGMATRIX aux1 = NULL;
	PGMATRIX aux2 = NULL;
	PGMATRIX aux3 = NULL;

	/******************************
	PRIVATE VARIABLE INITIALIZATION
	*******************************/

	//PID Errors Initialize.
	Kp = PGMATRIX_CREATE_DIAG_FROM_ARRAY(4,Kpv);
	Ki = PGMATRIX_CREATE_DIAG_FROM_ARRAY(4,Kiv);
	Kd = PGMATRIX_CREATE_DIAG_FROM_ARRAY(4,Kdv);

	//Robot initialize.
	r = Robot_create();
	///Robot's DH
	Main_robot_init(r);

	//Thetas
	Thetas =  PGMATRIX_CREATE_FROM_ARRAY(7,1,theta_init);
	DThetas = PGMATRIX_CREATE_ZEROES(7,1);

	//Initialize Error Matrices.
	Err =     PGMATRIX_CREATE_ZEROES(4,1);
	Err_ant = PGMATRIX_CREATE_ZEROES(4,1);
	Err_int = PGMATRIX_CREATE_ZEROES(4,1);

	//Update Robot
	Robot_updateLinksThetasByMatrix(r,Thetas);

	//Update End Effector Data
	efp = Robot_FKM(r);
	pitchAngle = DQ_getPitchAngle(efp);
	eft = DQ_getT(efp);

	//Define reference.
	D_ref = PGMATRIX_ALLOC(4,1);

	        PGMATRIX_DATA(D_ref,1,1) = pitchAngle;
	        PGMATRIX_DATA(D_ref,2,1) = eft->v[1];
		    PGMATRIX_DATA(D_ref,3,1) = eft->v[2];
			PGMATRIX_DATA(D_ref,4,1) = vz_ref;

	//Measured Differences initialization.
	D =     PGMATRIX_ALLOC(4,1);

	        PGMATRIX_DATA(D,1,1) = pitchAngle;
	        PGMATRIX_DATA(D,2,1) = eft->v[1];
		    PGMATRIX_DATA(D,3,1) = eft->v[2];
			PGMATRIX_DATA(D,4,1) = 0;

	//Jacobians
	OJP = Robot_getOJacobianAngleP(r);
	GJ  = Robot_getGJacobian(r);

	//Updates System Jacobian.
	SJ = PGMATRIX_CREATE_ZEROES(4,7);
	PGMATRIX_COPY_ROW(SJ, 1, OJP, 1);
	PGMATRIX_COPY_ROW(SJ, 2, GJ,  1);
	PGMATRIX_COPY_ROW(SJ, 3, GJ,  2);
	PGMATRIX_COPY_ROW(SJ, 4, GJ,  3);


	/******************************
			SIMULATION LOOP
	*******************************/

	ROS_INFO_STREAM("Initial State Data:");
	printf("\nReference.");
	PGMATRIX_PRINT(D_ref);
	printf("\nInitial System Jacobian.");
	PGMATRIX_PRINT(SJ);
	printf("\nInitial System Thetas.");
	PGMATRIX_PRINT(Thetas);

	ROS_INFO_STREAM("Simulating.");

	while(l < L && i < 100){
//		printf(".");

		//Last Error
		PGMATRIX_FREE(Err_ant);
		Err_ant = Err;
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
		//Proporcional  aux3 = Kp*(Err)
		aux3 = PGMATRIX_MULTIPLY_COPY_FREE(Kp,Err,FALSE,FALSE);

		//Error Sum     aux1 = Kd*(Err-Err_ant)*(1/T) + Ki*(Err_int) + Kp*(Err)
		aux1 = PGMATRIX_ADD_COPY_FREE(aux1,aux2,TRUE,TRUE);
		aux1 = PGMATRIX_ADD_COPY_FREE(aux1,aux3,TRUE,TRUE);

		//Pseudo Inverse Calculation
		aux2 = 		PGMATRIX_DAMPED_LEASTSQUARES_PSEUDOINVERSE(SJ,0.01);
					//PGMATRIX_PSEUDOINVERSE_PLUS(SJ);
					//PGMATRIX_RIGHT_PSEUDOINVERSE(SJ);

		//New DThetas = pinv(SJ)*(Error Sum)
		PGMATRIX_FREE(DThetas);
		DThetas = PGMATRIX_MULTIPLY_COPY_FREE(aux2,aux1,TRUE,TRUE);
		//New Thetas = DThetas*T + Old Thetas
		aux1 = PGMATRIX_MULTIPLY_CONST_COPY_FREE(DThetas,T,FALSE);
		Thetas = PGMATRIX_ADD_COPY_FREE(Thetas,aux1,TRUE,TRUE);



		/******************************
				COMUNICATE WITH SCHUNK
		*******************************/

		//Sends information to robot to move, then wait and measure
		//Set
		ros_velocities.velocities[0]=PGMATRIX_DATA(DThetas,1,1);
		ros_velocities.velocities[1]=PGMATRIX_DATA(DThetas,2,1);
		ros_velocities.velocities[2]=PGMATRIX_DATA(DThetas,3,1);
		ros_velocities.velocities[3]=PGMATRIX_DATA(DThetas,4,1);
		ros_velocities.velocities[4]=PGMATRIX_DATA(DThetas,5,1);
		ros_velocities.velocities[5]=PGMATRIX_DATA(DThetas,6,1);
		ros_velocities.velocities[6]=PGMATRIX_DATA(DThetas,7,1);

		//Saturate velocities for safety
		double sat_vel = 0.7;
		for(int j = 0; j < 7; j++){
			if(ros_velocities.velocities[j] > sat_vel){ros_velocities.velocities[j] = sat_vel;}
		}

		//Send
		ros_velpub.publish(ros_velocities);
		//Wait
		ros::spinOnce();
		ros_rate.sleep();
		//test
		diffs.push_back(ros_rate.cycleTime());
		//Measure
		if(!schunk_getPositions.call(ros_getPositions)){
			ROS_ERROR_STREAM("Couldn't get schunk's positions, aborting");
			return 1;
		}
		else{
			PGMATRIX_DATA(Thetas,1,1) = ros_getPositions.response.positions.positions[0];
			PGMATRIX_DATA(Thetas,2,1) = ros_getPositions.response.positions.positions[1];
			PGMATRIX_DATA(Thetas,3,1) = ros_getPositions.response.positions.positions[2];
			PGMATRIX_DATA(Thetas,4,1) = ros_getPositions.response.positions.positions[3];
			PGMATRIX_DATA(Thetas,5,1) = ros_getPositions.response.positions.positions[4];
			PGMATRIX_DATA(Thetas,6,1) = ros_getPositions.response.positions.positions[5];
			PGMATRIX_DATA(Thetas,7,1) = ros_getPositions.response.positions.positions[6];
		}

		/***************************************
			\END	COMUNICATE WITH SCHUNK
		****************************************/
		//Update
		Robot_updateLinksThetasByMatrix(r,Thetas);

		///Recalculation of measured data.
		//End Effector's Pose.
		DQ_free(efp);
		efp = Robot_FKM(r);
		//End Effector's Position.
		Q_free(eft);
		eft = DQ_getT(efp);
		//End Effector's 'Pitch' Angle
		pitchAngle = DQ_getPitchAngle(efp);
		//End Effector's Z Velocity (Geometrical Jacobian 3rd Line * DThetas)
		aux1 = PGMATRIX_CREATE_ZEROES(1,7);
		PGMATRIX_COPY_ROW(aux1,1,GJ,3);
		Vz = PGMATRIX_MULTIPLY_COPY_FREE(aux1,DThetas,TRUE,FALSE);
		vz = PGMATRIX_DATA(Vz,1,1);
		PGMATRIX_FREE(Vz);

		//Measured Data Matrix Update
		PGMATRIX_DATA(D,1,1) = pitchAngle;
		PGMATRIX_DATA(D,2,1) = eft->v[1];
		PGMATRIX_DATA(D,3,1) = eft->v[2];
		PGMATRIX_DATA(D,4,1) = vz;

		///Jacobian Update
		//Orientation Jacobian Angle P
		PGMATRIX_FREE(OJP);
		OJP = Robot_getOJacobianAngleP(r);
		//Geometrical Jacobian
		PGMATRIX_FREE(GJ);
		GJ  = Robot_getGJacobian(r);
		//System Jacobian.
		PGMATRIX_COPY_ROW(SJ,1,OJP,1);
		PGMATRIX_COPY_ROW(SJ,2,GJ,1);
		PGMATRIX_COPY_ROW(SJ,3,GJ,2);
		PGMATRIX_COPY_ROW(SJ,4,GJ,3);

		//l = l - vz*T;
		l = l -vz*T;

		i++;
	}


	for(int j = 0; j < 7; j++){
		ros_velocities.velocities[j] = 0.0;
	}
	//Send
	ros_velpub.publish(ros_velocities);
	//Wait
	ros::spinOnce();


	printf("\a\n\n\tEnded in %i interactions.",i);
	printf("\n\n\n\tFinal State Data:\n\n");
	printf("\nFinal D.");
	PGMATRIX_PRINT(D);
	printf("\nFinal System Jacobian.");
	PGMATRIX_PRINT(SJ);
	printf("\nFinal System Thetas.");
	PGMATRIX_PRINT(Thetas);


	/******************************
			CLEAN UP
	*******************************/

	for(int k = 0; k < diffs.size(); k++){
		ROS_INFO_STREAM(diffs[i].nsec);
	}

	//Last Memory Free Procedure.
	//Jacobians
	PGMATRIX_FREE(OJP);
	PGMATRIX_FREE(GJ);
	PGMATRIX_FREE(SJ);
	//Errors
	PGMATRIX_FREE(Err);
	PGMATRIX_FREE(Err_ant);
	PGMATRIX_FREE(Err_int);
	//Var
	Q_free(eft);
	DQ_free(efp);
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

	return 0;

}

/**
Robot's DH Parameters
*/
void Main_robot_init(Robot* r){
	//               theta d        a   alpha
	Robot_addLink(r, 0,   0.300        ,0,  -PI2);
	Robot_addLink(r, 0,   0            ,0,   PI2);
	Robot_addLink(r, 0,   0.328        ,0,  -PI2);
	Robot_addLink(r, 0,   0            ,0,   PI2);
	Robot_addLink(r, 0,   0.2765       ,0,  -PI2);
	Robot_addLink(r, 0,   0            ,0,   PI2);
	Robot_addLink(r, 0,   0.40049      ,0,   0  );
}
