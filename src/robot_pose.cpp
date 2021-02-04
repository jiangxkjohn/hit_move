#include "robot_pose.h"
#include <std_srvs/Empty.h>

namespace hit{

  float quat2theta(const Quat& quat){
    return thetaStandard(2 * atan2(quat.z, quat.w) * 180 / M_PI);
  }

  float thetaStandard(float theta){
    theta = float(fmod(theta, 360));
    if (theta > 180){
      theta = theta - 360;
    }else if (theta <= -180){
      theta = theta + 360;
    }
    return theta;
  }

  float thetaError(float goal, float theta){
    return thetaStandard(theta - goal);
  }

  RobotPose::RobotPose(ros::NodeHandle& handle){
    // initialize the amcl pose without setting it in rviz manually
    ROS_INFO("Waiting for amcl pose initialized...");
    std_srvs::EmptyRequest req;
    std_srvs::EmptyResponse res;
    if (!ros::service::call("/global_localization", req, res)){
      ROS_WARN("Initialize amcl pose failed! You need to set it up manually.");}
    // set the tf_pose_listener
    tf_pose_listener_ = new tf::TransformListener;
    tf_pose_listener_->waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
    // tf_pose_listener_->waitForTransform("/world", "/turtle1", ros::Time(0), ros::Duration(3.0));
    updatePose();
    ROS_INFO("TF pose has been initialized!");
  }

  void RobotPose::updatePose(){
    tf_pose_listener_->lookupTransform("/map", "/base_link", ros::Time(0), tf_pose);
    // tf_pose_listener_->lookupTransform("/world", "/turtle1", ros::Time(0), tf_pose);
    _pose.position.x = tf_pose.getOrigin().x();
    _pose.position.y = tf_pose.getOrigin().y();
    _pose.position.z = tf_pose.getOrigin().z();
    _pose.orientation.x = tf_pose.getRotation().x();
    _pose.orientation.y = tf_pose.getRotation().y();
    _pose.orientation.z = tf_pose.getRotation().z();
    _pose.orientation.w = tf_pose.getRotation().w();
  }

  const geometry_msgs::Pose& RobotPose::getPose(){
    updatePose();
    return _pose;
  }

  RobotPose::~RobotPose(){
    delete tf_pose_listener_;
  }

}