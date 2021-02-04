#include "go.h"
#include "head.h"
#include <signal.h>

void sigintHandler(int sig){
	ros::NodeHandle shutdown;

	// TODO: need to be checked!!!!!!!!!!!!!!
	// check the name of move_base/cancel!!!!!!!!!!!1
	ros::Publisher runner_stop = shutdown.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
	actionlib_msgs::GoalID runner_stop_signal;
	std::cout << runner_stop_signal.stamp.nsec << std::endl;
	runner_stop.publish(runner_stop_signal);

	ros::Publisher shutdown_pub = shutdown.advertise<hit::Twist>("/cmd_vel", 1);
	hit::Twist shutdown_signal;
	shutdown_pub.publish(shutdown_signal);
	ROS_INFO("hit node has been shutdown!");
	ros::shutdown();
}

int main(int argc, char** argv){
	ros::init(argc, argv, "go_node", ros::init_options::NoSigintHandler);

	// This function must used after NodeHandle and don't know why
	signal(SIGINT, sigintHandler);

	hit::Head head;
	hit::Go go;
	
	// ros::MultiThreadedSpinner spinner(2);
	// spinner.spin();
	ros::spin();
	return 0;
}
