#ifndef _HEAD_H_
#define _HEAD_H_
#include <ros/ros.h>
#include "hit/NodControl.h"
#include "/home/nav/catkin_ws/devel/include/tcp_server/SetHeadControl.h"
// #include "tcp_server/SetHeadControl.h"

namespace hit{

  class Head
  {
  private:
    ros::ServiceServer nod_server;
    ros::Publisher angle_pub;
    
    std::map<int, int> head_pose_list;
    bool nodCB(NodControlRequest& req, NodControlResponse& res);

  public:
    Head();
    ~Head();
  };
}
#endif
