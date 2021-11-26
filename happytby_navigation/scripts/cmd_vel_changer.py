#!/usr/bin/env python
from __future__ import print_function

import roslib
import rospy
from geometry_msgs.msg import Vector3, Twist

class cmd_vel_changer:
    def __init__(self):
        rospy.Subscriber("cmd_vel_navi_raw", Twist, self.CmdVelCallback)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel_navi", Twist, queue_size=1)

    def CmdVelCallback(self, data):
        cmd_vel_changed = data
        cmd_vel_changed.angular.z *= -1
        self.cmd_vel_pub.publish(cmd_vel_changed)

if __name__ == '__main__':
    rospy.init_node('cmd_vel_changer', anonymous=False)
    cvc = cmd_vel_changer()
    while not rospy.is_shutdown():
        try:
            rospy.spin()

        except rospy.ROSInterruptException:
            print('map_navigation node terminated.')
