#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from motor_class import Motor
import math


class Tranbot:

    def __init__(self,
                pinRightFwd, pinRightRev, pinLeftFwd, pinLeftRev,
                wheel_diameter=.065, wheel_base=0.15,
                left_max_rpm=200.0, right_max_rpm=200.0,
                frequency=20):

        self._frequency = frequency
        self._left_max_rpm = left_max_rpm
        self._right_max_rpm = right_max_rpm
        self._wheel_diameter = wheel_diameter
        self._wheel_base = wheel_base
        self._rightWheel = Motor(pinRightFwd, pinRightRev, frequency)
        self._leftWheel = Motor(pinLeftFwd, pinLeftRev, frequency)

        self.speed = 0.0
        self.spin = 0.0
        self.close = 0.30  # start slowing down when this close
        self.tooclose = 0.10   # no forward motion when this close
        self.distance = 100.0


    def test_callback(self, msg):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg)

    
    def cmd_vel_callback(self, msg):
        self.speed = msg.linear.x
        self.spin = msg.angular.z
        self._set_motor_speeds()


    def cmd_vel_subscriber(self):
        rospy.Subscriber('cmd_vel',
                        Twist,
                        #self._cmd_vel_callback
                        self.cmd_vel_callback)



    def _set_motor_speeds(self):
        # TODO: inject a stop() if no speeds seen for a while
        #
        # Scary math ahead.
        #
        # First figure out the speed of each wheel based on spin: each wheel
        # covers self._wheel_base meters in one radian, so the target speed
        # for each wheel in meters per sec is spin (radians/sec) times
        # wheel_base divided by wheel_diameter
        #
        right_twist_mps = self.spin * self._wheel_base / self._wheel_diameter
        left_twist_mps = -1.0 * self.spin * \
            self._wheel_base / self._wheel_diameter
        #
        # Now add in forward motion.
        #
        left_mps = self.speed + left_twist_mps
        right_mps = self.speed + right_twist_mps
        #
        # Convert meters/sec into RPM: for each revolution, a wheel travels
        # pi * diameter meters, and each minute has 60 seconds.
        #
        left_target_rpm = (left_mps * 60.0) / (math.pi * self._wheel_diameter)
        right_target_rpm = (right_mps * 60.0) / \
            (math.pi * self._wheel_diameter)
        #
        left_percentage = (left_target_rpm / self._left_max_rpm) * 100.0
        right_percentage = (right_target_rpm / self._right_max_rpm) * 100.0
        #
        # clip to +- 100%
        left_percentage = max(min(left_percentage, 100.0), -100.0)
        right_percentage = max(min(right_percentage, 100.0), -100.0)
        #
        # Add in a governor to cap forward motion when we're about
        # to collide with something (but still backwards motion)
        governor = 1.0
        if self.distance < self.tooclose:
            governor = 0.0
        elif self.distance < self.close:
            governor = (self.distance - self.tooclose) / \
                (self.close - self.tooclose)
        if right_percentage > 0:
            right_percentage *= governor
        if left_percentage > 0:
            left_percentage *= governor
        #
        self._rightWheel.run(right_percentage)
        self._leftWheel.run(left_percentage)




def main():
    rospy.init_node('motor_control_test')

    bot_1 = Tranbot(pinRightFwd=10, pinRightRev=9,
                pinLeftFwd=8, pinLeftRev=7, left_max_rpm=195,
                right_max_rpm=202)

    bot_1.cmd_vel_subscriber()
    rospy.spin()



if __name__ == '__main__':
    main()