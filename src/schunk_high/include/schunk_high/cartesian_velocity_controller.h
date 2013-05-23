#include<vector>
#include<schunk_high/robot.h>
#include<schunk_high/cartesian_controller.h>

#ifndef CARTESIAN_VELOCITY_CONTROLLER_HEADER_GUARD
#define CARTESIAN_VELOCITY_CONTROLLER_HEADER_GUARD

class Cartesian_velocity_controller: public Cartesian_controller{
protected:

	////ROS RELATED
	//Node Handles and queues
	//Reference related
	ros::NodeHandle reference_updater_nh;
	ros::CallbackQueue reference_updater_queue;
	//Topic Subscriber
	ros::Subscriber reference_updater_subscriber;

	//End Effector related
	ros::NodeHandle endEffector_vel_nh;
	ros::CallbackQueue endEffector_vel_queue;
	//Publisher and msg
	ros::Publisher endEffector_vel_pub;
	schunk_msgs::Velocities endEffector_vel;

	//Prints End Effector's Velocities in a topic
	inline bool printVel();

	//Updates the End Effetor's Reference Velocities
	void updateReferences(double ref_gain, double ref_gain2);

public:

	//Constructor
	Cartesian_velocity_controller(Robot* r, double KP[], double KI[], double KD[], double T);

	//Destructor
	~Cartesian_velocity_controller();

	//Main routine
	void control();

	//Callback
	void referenceUpdateCallback(const schunk_msgs::Velocities::ConstPtr& msg);
	//Reference Velocities Buffer
	std::vector<double> references;

};

#endif
