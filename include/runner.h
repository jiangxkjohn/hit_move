#ifndef _RUNNER_H_
#define _RUNNER_H_

#include "robot_pose.h"

namespace hit{

	class Runner{
		public:

		Runner(ros::NodeHandle& handle);
		~Runner();
		bool run2pose(const geometry_msgs::Pose& goal);

		private:

		RunActionClient* run_client_;
		// ros::ServiceClient path_client;
		ros::NodeHandle& _handle;
	};

};
#endif