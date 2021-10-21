#!/usr/bin/env python3
import roslib
import rospy
import os

import std_msgs.msg
from std_msgs.msg import UInt8


class Start():
    pub_decided_mode = rospy.Publisher('/detect/traffic_light', UInt8, queue_size=1)

    isPublished = False

    def controller(self, msg):
        pass
        print("traffic_light")
        print(msg)
        # os.system('rostopic pub -1 /core/decided_mode std_msgs/UInt8 "data: 2"')
        if not self.isPublished:
            self.pub_decided_mode.publish(msg)
            self.isPublished = True

    def test(self, msg):
        pass
        print("decided_mode")
        print(msg)

    def subscriber(self):

        # pub_decided_mode.publish("data: 3")
        rospy.Subscriber('/detect/traffic_light', UInt8, self.controller, queue_size=1)
        rospy.Subscriber('/core/decided_mode', UInt8, self.test, queue_size=1)
        rospy.spin()

    def main(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('start')
    Start().subscriber()


