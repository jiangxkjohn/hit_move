#include "pid.h"
#include <cmath>

namespace hit{

  PID::PID(){}

  PID::PID(float kp, float ki, float kd):
  e_sum(0),
  e_(0),
  e(0),
  KP(kp),
  KI(ki),
  KD(kd){}

  float PID::control(const Pose& goal, const Pose& pose){
    double dx = double(goal.position.x - pose.position.x);
    double dy = double(goal.position.y - pose.position.y);
    float theta_real = quat2theta(pose.orientation);
    float theta_goal = thetaStandard(atan2(dy, dx) * 180 / M_PI);
    float e_theta = thetaError(theta_goal, theta_real);
    // ROS_INFO("-----");
    // ROS_INFO("theta_goal : %.4f, theta_real : %.4f, e_theta : %.4f", theta_goal, theta_real, e_theta);
    // ROS_INFO("fabs(e_theta) : %.4f", fabs(e_theta));
    // the deviation between moving direction and goal orientation
    if (fabs(e_theta) < 90){
      e = - sqrt(pow(dx, double(2)) + pow(dy, double(2)));}
    else{
      e = sqrt(pow(dx, double(2)) + pow(dy, double(2)));}
    e_sum = e_sum + e;
    e_ = e;
    return float(KP * e + KI * e_sum + KD * (e - e_));
  }

  float PID::control(const float theta_goal, const float theta_current){
    // ROS_INFO("current_theta : %.4f, goal_theta : %.4f", theta_current, theta_goal);
    e = thetaError(theta_goal, theta_current);
    e_sum = e_sum + e;
    e_ = e;
    return float(KP * e + KI * e_sum + KD * (e - e_));
  }

  float PID::getError(){
    return e;
  }

  void PID::cleanError(){
    e = 0;
    e_ = 0;
    e_sum = 0;
  }

  void PID::setPID(float kp, float ki, float kd){
    KP = kp;
    KI = ki;
    KD = kd;
  }

  

  PID::~PID(){}
};