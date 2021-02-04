#include "go.h"
#include <iostream>
#include <cmath>

namespace hit{

  Go::Go():
  walker_(NULL),
  runner_(NULL),
  tf_pose_(NULL),
  goal_(new Pose()){

    ros::NodeHandle private_nh("~");

    ROS_INFO("Start to activate Go");

    // set the server for go node
    go_server = private_nh.advertiseService("/hit/move_service", &Go::goCB, this);

    // set the tf_pose
    tf_pose_ = new RobotPose(private_nh);

    // set the walker
    walker_ = new Walker(private_nh, *tf_pose_);

    // set the runner
    runner_ = new Runner(private_nh);

    ROS_INFO("Go has been initialised!");
  }

  void Go::transGoal(const GoControlRequest& req){
    geometry_msgs::Pose pose = tf_pose_->getPose();
    double theta = quat2theta(pose.orientation);
    switch (req.relative)
    {
      case 0:
        ROS_INFO("non relative");
        goal_->position.x = req.x;
        goal_->position.y = req.y;
        goal_->orientation.z = req.z;
        goal_->orientation.w = req.w;
        break;
      case 1:
        ROS_INFO("relative");
        goal_->position.x = pose.position.x + req.x * cos(theta * M_PI / 180) - req.y * sin(theta * M_PI / 180);
        goal_->position.y = pose.position.y + req.x * sin(theta * M_PI / 180) + req.y * cos(theta * M_PI / 180);
        goal_->orientation.z = sin((theta + req.z) * M_PI / 180 / 2);
        goal_->orientation.w = cos((theta + req.z) * M_PI / 180 / 2);
        std::cout << theta << " " << quat2theta(goal_->orientation) << std::endl;
        break;
      default:
        ROS_INFO("Wrong relative code");
    }
  }

  bool Go::goCB(GoControlRequest& req, GoControlResponse& res){
    transGoal(req);
    switch (req.type)
    {
    case 0:
      ROS_INFO("Have received the Run command");
      res.result = runner_->run2pose(*goal_);
      std::cout << res.result << std::endl;
      break;
    case 1:
      ROS_INFO("Have received the Walk command");
      res.result = walker_->walk2Pose(*goal_);
      res.result = 1;
      break;
    case 2:
      ROS_INFO("Have received the rotation command");
      walker_->rotate2theta(*goal_);
      res.result = 1;
      break;
    default:
      ROS_WARN("Wrong type");
      break;
    }
    return true;
  }

  Go::~Go(){
    delete runner_;
    delete walker_;
    delete tf_pose_;
    delete goal_;
    std::cout << "the pointer in go has been exterminated" << std::endl;
  }

};