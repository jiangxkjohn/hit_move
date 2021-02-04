#include "head.h"

typedef tcp_server::SetHeadControl SetHead;

namespace hit{

  Head::Head()
  {
    ROS_INFO("Start to activate Head");
    XmlRpc::XmlRpcValue scanner_params;
    ros::param::get("hit/head/head_pose_list", scanner_params);
    for (int i = 0; i < scanner_params.size(); i++)
    {
      head_pose_list[scanner_params[i]["code"]] = scanner_params[i]["angle"];
    }
    ros::NodeHandle head_nh("~/head");
    nod_server = head_nh.advertiseService("/hit/nod_service", &Head::nodCB, this);
    angle_pub = head_nh.advertise<SetHead>("/set_head_control", 10);
    ROS_INFO("Head has been initialised!");
  }

  bool Head::nodCB(NodControlRequest& req, NodControlResponse& res)
  {
    SetHead head_pub_msgs;
    head_pub_msgs.moto_valid_bit = 1;

    if (req.code != -1){
      head_pub_msgs.head_angles = head_pose_list[req.code];
    }else{
      head_pub_msgs.head_angles = req.angle;
    }
    angle_pub.publish(head_pub_msgs);
    res.result = 1;
    // ros::Rate loop_rate(0.5);
    ROS_INFO("Move to position %d", - head_pub_msgs.head_angles / 10);
    return true;
  }

  Head::~Head()
  {
    std::cout << "The head part has been stopped" << std::endl;
  }

}
