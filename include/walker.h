#ifndef _WALKER_H_
#define _WALKER_H_

#include "pid.h"
#include <dynamic_reconfigure/server.h>
#include "hit/PIDConfig.h"

namespace hit{

  class Walker
	{
	private:
		PID _radiusPID, _anglePID;
    ros::NodeHandle& _handle;

		float max_vel_radius, max_vel_angle;
		float period;
		float max_radius_error, max_angle_error;

		ros::Publisher _cmd_pub;

    bool isReachGoal();
		void walk2theta(const geometry_msgs::Pose& goal);

		dynamic_reconfigure::Server<PIDConfig> _pidServer;
		void reconfigureCb(PIDConfig& config, uint32_t level);

		RobotPose& _tf_pose;

	public:
		Walker(ros::NodeHandle& handle, RobotPose& robot_pose);
		bool walk2Pose(const geometry_msgs::Pose& goal);
		void setVelocity(float m_v_r, float m_v_a);
		void rotate2theta(const Pose&);
		~Walker();
	};
	
};
#endif