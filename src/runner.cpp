#include "runner.h"
#include <actionlib_msgs/GoalID.h>
#include <std_srvs/Empty.h>

namespace hit{

   Runner::Runner(ros::NodeHandle& handle):
   _handle(handle){
    // set the path service client from move_base
    ROS_INFO("Waiting for runner...");
    run_client_ = new RunActionClient("move_base", true);
    run_client_->waitForServer();
    ROS_INFO("runner has been activated");

  }

  bool Runner::run2pose(const geometry_msgs::Pose& goal){
    std_srvs::EmptyRequest req;
    std_srvs::EmptyResponse res;
    ros::service::call("/move_base/clear_costmaps", req, res);

    move_base_msgs::MoveBaseGoal run_goal;
    run_goal.target_pose.header.frame_id = "map";
    run_goal.target_pose.pose = goal;
    run_client_->sendGoal(run_goal);
    bool result = run_client_->waitForResult(ros::Duration(60.0));
    if (result){
      ROS_INFO("Run to goal successfully!");
    }else{
      // TODO: need to be checked!!!!!!!!!!!!!!
      // check the name of move_base/cancel!!!!!!!!!!!1
      ROS_WARN("Fail to run to the goal");
      ros::Publisher runner_stop = _handle.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
      actionlib_msgs::GoalID runner_stop_signal;
      runner_stop.publish(runner_stop_signal);
      ROS_WARN("Fail to run to the goal");}
    return result;
  }

  Runner::~Runner(){
    delete run_client_;
  }

}
