#ifndef _ROBOT_POSE_H_
#define _ROBOT_POSE_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetPlan.h>
#include "hit/GoControl.h"
#include <tf/transform_listener.h>

namespace hit{

	typedef geometry_msgs::Pose Pose;
	typedef geometry_msgs::PoseWithCovarianceStamped AmclPose;
	typedef geometry_msgs::Quaternion Quat;
	typedef geometry_msgs::Twist Twist;
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> RunActionClient;
	typedef move_base_msgs::MoveBaseFeedbackConstPtr FbPtr;
	typedef actionlib::SimpleClientGoalState State;
	typedef move_base_msgs::MoveBaseResultConstPtr ResultPtr;

	
  float quat2theta(const Quat& quat);
	float thetaStandard(float theta);
  float thetaError(float goal, float theta);

  class RobotPose
	{
	private:
    tf::TransformListener* tf_pose_listener_;
		tf::StampedTransform tf_pose;
		geometry_msgs::Pose _pose;

	public:
		RobotPose(ros::NodeHandle& handle);
		void updatePose();
		const geometry_msgs::Pose& getPose();

		~RobotPose();
	};

	
};
#endif