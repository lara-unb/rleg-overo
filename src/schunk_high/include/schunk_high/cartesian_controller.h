#include<vector>
#include<schunk_high/robot.h>
#include <schunk_low/GetPosVel.h>

#ifndef CARTESIAN_CONTROLLER_HEADER_GUARD
#define CARTESIAN_CONTROLLER_HEADER_GUARD

class Cartesian_controller{
protected:

	//// ROS RELATED
	int ros_pub_buffer;

	//End
	bool end_control;

	//Node Handles and queues
	ros::NodeHandle schunk_low_modeChanger_nh;
	ros::CallbackQueue modeChanger_queue;
	ros::ServiceClient schunk_modeChanger_client;
	schunk_low::ControlMode schunk_modeChanger_msg;

	ros::NodeHandle schunk_low_posVelGetter_nh;
	ros::CallbackQueue posVelGetter_queue;
	ros::ServiceClient schunk_posVelGetter_client;
	schunk_low::GetPosVel schunk_posVelGetter_msg;

	ros::NodeHandle schunk_low_publisher_nh;
	ros::CallbackQueue publisher_queue;
	ros::Publisher schunk_low_pub;
	schunk_msgs::Velocities schunk_low_velocities_msg;

	ros::NodeHandle schunk_low_pos_publisher_nh;
	ros::CallbackQueue pos_publisher_queue;
	ros::Publisher schunk_low_pos_pub;
	schunk_msgs::Positions schunk_low_positions_msg;

	ros::NodeHandle schunk_low_stopper_nh;
	ros::CallbackQueue stopper_queue;
	ros::ServiceClient schunk_stopper_client;
	schunk_low::Stop schunk_stopper_msg;

	//// DUAL QUATERNION KINEMATICS CALCULATION Variables
	//Jacobians
	PGMATRIX SJ;
	//PID errors.
	PGMATRIX Kp;
	PGMATRIX Ki;
	PGMATRIX Kd;
	//Thetas.
	PGMATRIX Thetas;
	PGMATRIX DThetas;
	//Reference Signal.
	PGMATRIX D_ref;
	//Measured Values
	PGMATRIX D;
	//Errors
	PGMATRIX Err;
	PGMATRIX Err_ant;
	PGMATRIX Err_int;
	//Aux
	PGMATRIX aux1;
	PGMATRIX aux2;
	PGMATRIX aux3;

	////Sample period
	double T;

	////Schunk interface
	bool changeSchunkMode(int mode);
	bool getSchunkPositionsAndVelocitiesAndUpdateThetasAndDThetas();
	void setSchunkVelocitiesAndSend(double sat_vel);
	void sendJointPositionsToSchunk(std::vector<double> positions);
	bool stopSchunk();

	void controlCleanUp();

	//Dual mode variables
	//const static int HIGH_MODE = 0;
	//const static int LOW_MODE = 1;
	//int current_operation_mode;


public:

	//Constructor
	Cartesian_controller(Robot* r, int refSize, double KP[], double KI[], double KD[], double T);

	//Destructor
	~Cartesian_controller();

	//Variables
	Robot* r;

	//Main routine
	virtual void control()=0;

};

#endif
