#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import UInt8


def controller(msg):
    pass
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)


def subscriber():
    rospy.init_node('start', anonymous=True)

    rospy.Subscriber('/detect/traffic_light', UInt8, controller)
    rospy.spin()


if __name__ == '__main__':
    subscriber()
