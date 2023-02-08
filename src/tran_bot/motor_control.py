#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist


def test_callback(msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg)


def listener_test():
    rospy.init_node('motor_control_test')
    rospy.Subscriber('cmd_vel',
                     Twist,
                     #self._cmd_vel_callback
                     test_callback)
    rospy.spin()

if __name__ == '__main__':
    listener_test()
