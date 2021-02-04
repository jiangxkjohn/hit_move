#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from agv_driver.msg import Qrcode
from hit.srv import *
from math import atan2
from random import random

def randomMaxMin(_max, _min = 0):
    seed = random()
    if seed > 0.5:
        return 2 * (_max - _min) * (seed - 0.5) + _min
    else:
        return 2 * (_max - _min) * (seed - 0.5) - _min

class load_pose():
    def __init__(self):
        rospy.init_node("auto_accurate")
        self.pose_sub = rospy.Subscriber("/amcl_pose", \
            PoseWithCovarianceStamped, self.poseCb)
        self.goal_client = rospy.ServiceProxy("/hit/move_service", GoControl)
        self.qr_sub = rospy.Subscriber("/qrcode_data", \
            Qrcode, self.qrCb)
        self.goal_client.wait_for_service()
        rospy.loginfo("Server has been started")
        rospy.sleep(3)
        self.loadPose()

    def loadPose(self):
        self.poseTruth = self.pose
        self.qrTruth = self.qr
        rospy.loginfo("Load the pose")

    def qrCb(self, msg):
        self.qr = msg

    def poseCb(self, amcl_pose):
        self.pose = self.pose = amcl_pose  

    def sendRequest(self):
        self.goal_client.wait_for_service()
        request = GoControlRequest()
        request.type = 1
        request.x = self.poseTruth.pose.pose.position.x
        request.y = self.poseTruth.pose.pose.position.y
        request.z = self.poseTruth.pose.pose.orientation.z
        request.w = self.poseTruth.pose.pose.orientation.w
        # print("theta : ", atan2(request.z, request.w) * 2)
        # print("position : ", request.x, request.y)
        self.goal_client.call(request)
        # self.goal_client(request)
        rospy.sleep(0.5)
        dx = self.qr.x_pos - self.qrTruth.x_pos
        dy = self.qr.y_pos - self.qrTruth.y_pos
        dTheta = self.qr.angle - self.qrTruth.angle
        rospy.loginfo("dx : {:.4f}, dy : {:.4f}, theta : {:.4f}".format(dx, dy, dTheta))
            
        print("---------------------------------------")
        
    def autoMove(self):
        request = GoControlRequest()
        request.type = 1
        request.relative = 1
        
        x = randomMaxMin(1.2, 0.5)
        y = randomMaxMin(0.5 * x, 0)
        
        theta_zero = atan2(y, x) / 3.1415926 * 180
        if theta_zero > 90:
            theta_zero = 180 - theta_zero
        elif theta_zero < -90:
            theta_zero = 180 + theta_zero

        theta = 20 * random() + theta_zero

        request.x = x
        request.y = y
        request.z = theta
        rospy.loginfo("x : {:.4f}, y : {:.4f}, theta : {:.4f}".format(x, y, theta))
        self.goal_client.call(request)

if __name__ == "__main__":
    lp = load_pose()
    for i in range(10):
        rospy.loginfo("Test : " + str(i))
        lp.autoMove()
        lp.sendRequest()
        