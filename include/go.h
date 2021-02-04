#ifndef _GO_H_
#define _GO_H_

#include "walker.h"
#include "runner.h"
#include "robot_pose.h"

namespace hit{

	class Go{
		public:

		Go();
		~Go();

		private:

		void transGoal(const GoControlRequest& req);
		bool goCB(GoControlRequest& req, GoControlResponse& res);

		ros::ServiceServer go_server;
		RunActionClient* run_client_;        

		Walker* walker_;
		Runner* runner_;
		RobotPose* tf_pose_;
		geometry_msgs::Pose* goal_;
	};
};
#endif