#include<vector>
#include<schunk_high/robot.h>
#include<schunk_high/cartesian_controller.h>

class Cartesian_pose_controller: public Cartesian_controller{
private:

	////ROS RELATED
	//Node Handles and queues

	//Topic Subscriber
	ros::NodeHandle reference_updater_nh;
	ros::CallbackQueue reference_updater_queue;
	ros::Subscriber reference_updater_subscriber;

	//Publisher and msg
	ros::Publisher schunk_posePublisher;
	ros::NodeHandle schunk_pose_nh;
	ros::CallbackQueue schunk_pose_queue;
	schunk_msgs::Positions schunk_pose;

	//Initial Positions To go To Vector
	std::vector<double> initialPositions;

	bool printPose();

	void updateReferences(double ref_gain);

public:

	//Constructor
	Cartesian_pose_controller(std::vector<double> initialPositions, Robot* r, double KP[], double KI[], double KD[], double T);

	//Destructor
	~Cartesian_pose_controller();

	//Main routine
	void control();

	//Callback
	void updateReferencesCallback(const schunk_msgs::Positions::ConstPtr& msg);

};
