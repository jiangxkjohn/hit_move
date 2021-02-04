#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from people_msgs.msg import PositionMeasurementArray
from time import sleep

class Talker:
    def __init__(self):
        rospy.init_node("talker")
        self.flag = False
        self.talker_pub = rospy.Publisher("/TTSTopic", String, queue_size=1)
        self.Subscriber = rospy.Subscriber("/cmd_vel", Twist, self.cmdCb)
        self.Subscriber = rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray, self.peopleCb)

    def cmdCb(self, cmd_vel):
        if cmd_vel.linear.x == 0 and cmd_vel.angular.z == 0:
            self.flag = False
            # print("stop")
        else:
            self.flag = True
            # print("moving")

    def peopleCb(self, people_msg):
        # print("receive people")
        if len(people_msg.people) > 0 and self.flag:
            for person in people_msg.people:    
                # print(person.pos.x)
                if person.pos.x < 1.5 and abs(person.pos.y) < 0.5:
                    print("talker")
                    self.talker_pub.publish(String("110"))
                    sleep(7)
                    self.flag = False
        
if __name__ == "__main__":
    talker = Talker()
    rospy.spin()
