#!/usr/bin/env python3
import roslib
import rospy
import os
from std_msgs.msg import UInt8

pub_decided_mode = rospy.Publisher('/core/decided_mode', UInt8)


def controller(msg):
    pass
    print(msg.data)
    # os.system('rostopic pub -1 /core/decided_mode std_msgs/UInt8 "data: 2"')
    pub_decided_mode.publish(msg.data)


def test(msg):
    pass
    print(msg.data)


def subscriber():
    rospy.init_node('start', anonymous=True)
    # pub_decided_mode.publish("data: 3")
    rospy.Subscriber('/detect/traffic_light', UInt8, controller, queue_size=1)
    rospy.Subscriber('/core/decided_mode', UInt8, test, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    subscriber()
