#include "go.h"
#include <cmath>
#include "walker.h"

namespace hit{

  Walker::Walker(ros::NodeHandle& handle, RobotPose& robot_pose):
  _handle(handle),
  _tf_pose(robot_pose){

    // set up dynamic reconfigure
    dynamic_reconfigure::Server<PIDConfig>::CallbackType cb = boost::bind(&Walker::reconfigureCb, this, _1, _2);
    _pidServer.setCallback(cb);

    // set the parameter of PID
    ROS_INFO("Start to initialize walker");
    float kp, ki, kd;
    ros::param::get("/hit/PID/radiusPID_KP", kp);
    ros::param::get("/hit/PID/radiusPID_KI", ki);
    ros::param::get("/hit/PID/radiusPID_KD", kd);
    _radiusPID = PID(kp, ki, kd);
    ros::param::get("/hit/PID/anglePID_KP", kp);
    ros::param::get("/hit/PID/anglePID_KI", ki);
    ros::param::get("/hit/PID/anglePID_KD", kd);
    _anglePID = PID(kp, ki, kd);

    // the control period
    ros::param::get("/hit/PID/PID_period", period);

    // set the max velocity
    ros::param::get("/hit/PID/max_velocity_radius", max_vel_radius);
    ros::param::get("/hit/PID/max_velocity_angle", max_vel_angle);

    // set the max error
    ros::param::get("/hit/PID/max_radius_error", max_radius_error);
    ros::param::get("/hit/PID/max_angle_error", max_angle_error);

    // cmd_vel publisher
    _cmd_pub = _handle.advertise<Twist>("/cmd_vel", 1);
    ROS_INFO("Walker node has been initialized successfully!");
  }

  void Walker::reconfigureCb(PIDConfig& config, uint32_t level){
    _radiusPID.setPID(config.radius_KP, config.radius_KI, config.radius_KD);
    ROS_INFO("Set radius PID: %.2f, %.2f, %.2f", config.radius_KP, config.radius_KI, config.radius_KD);
    _anglePID.setPID(config.angle_KP, config.angle_KI, config.angle_KD);
    ROS_INFO("Set angle PID: %.2f, %.2f, %.2f", config.angle_KP, config.angle_KI, config.angle_KD);
    setVelocity(config.max_velocity_radius, config.max_velocity_angle);
    max_radius_error = config.max_radius_error;
    max_angle_error = config.max_angle_error;
    period = config.PID_period;
  }

  bool Walker::isReachGoal(){
    if (fabs(_anglePID.getError()) > M_PI / 180){
      return false;};
    return true;
  }

  void Walker::rotate2theta(const Pose& goal){
    const Pose& pose = _tf_pose.getPose();
    float dx = goal.position.x - pose.position.x;
    float dy = goal.position.y - pose.position.y;
    float goal_theta = thetaStandard(atan2(dy, dx));
    do{
      geometry_msgs::Twist twist;
      const Pose& pose = _tf_pose.getPose();
      float current_theta = quat2theta(pose.orientation);
      float angle_signal = _anglePID.control(goal_theta, current_theta);
      // ROS_INFO("current_theta : %.4f, goal_theta : %.4f, angle_signal : %.4f", current_theta, goal_theta, angle_signal);
      twist.angular.z = std::max(std::min(angle_signal, max_vel_angle), -max_vel_angle);
      _cmd_pub.publish(twist);
      ros::Duration(period).sleep();
      // std::cout << fabs(_anglePID.getError()) << std::endl;
    }while (fabs(_anglePID.getError()) > max_angle_error && ros::ok());
    ROS_INFO("Rotate to the goal successfully!");
    geometry_msgs::Twist twist;
    _cmd_pub.publish(twist);
    _anglePID.cleanError();
    _radiusPID.cleanError();
  }

  void Walker::walk2theta(const Pose& goal){
    // ROS_INFO("---------------");
    float goal_theta = quat2theta(goal.orientation);
    do{
      geometry_msgs::Twist twist;
      const Pose& pose = _tf_pose.getPose();
      float current_theta = quat2theta(pose.orientation);
      float angle_signal = _anglePID.control(goal_theta, current_theta);
      // ROS_INFO("current_theta : %.4f, goal_theta : %.4f, angle_signal : %.4f", current_theta, goal_theta, angle_signal);
      twist.angular.z = std::max(std::min(angle_signal, max_vel_angle), -max_vel_angle);
      _cmd_pub.publish(twist);
      ros::Duration(period).sleep();
      // std::cout << fabs(_anglePID.getError()) << std::endl;
    }while (fabs(_anglePID.getError()) > max_angle_error && ros::ok());
  }

  bool Walker::walk2Pose(const Pose& goal){
    ROS_INFO("Start walking!");
    // std::cout << goal.position.x << " " << goal.position.y << " " << quat2theta(goal.orientation) << std::endl;
    do{
      geometry_msgs::Twist twist;
      const Pose& pose = _tf_pose.getPose();
      double dx = double(goal.position.x - pose.position.x);
      double dy = double(goal.position.y - pose.position.y);
      float current_theta = quat2theta(pose.orientation);
      float goal_theta = thetaStandard(atan2(dy, dx) * 180 / M_PI);
      if (fabs(thetaError(current_theta, goal_theta)) > 90){
        goal_theta = thetaStandard(goal_theta + 180);}
      float angle_signal = _anglePID.control(goal_theta, current_theta);
      twist.angular.z = std::max(std::min(angle_signal, max_vel_angle), -max_vel_angle);
      float linear_signal;
      linear_signal = _radiusPID.control(goal, pose);
      twist.linear.x = std::max(std::min(linear_signal, max_vel_radius), -max_vel_radius);
      // ROS_INFO("current_theta : %.4f, goal_theta : %.4f, angle_signal : %.4f, linear_signal : %.4f", current_theta, goal_theta, angle_signal, linear_signal);
      _cmd_pub.publish(twist);
      ros::Duration(period).sleep();
      // std::cout << _radiusPID.getError() << " " << _handle.ok() << ros::ok() << std::endl;
    }while (fabs(_radiusPID.getError()) > max_radius_error && ros::ok());
    // ROS_INFO("Finish the straight line");
    walk2theta(goal);

    geometry_msgs::Twist twist;
    _cmd_pub.publish(twist);
    ROS_INFO("Walk to the goal successfully!");
    _anglePID.cleanError();
    _radiusPID.cleanError();
    return true;
  }

  void Walker::setVelocity(float m_v_r, float m_v_a){
    max_vel_radius = m_v_r;
    max_vel_angle = m_v_a;
  }

  Walker::~Walker(){
  }

};