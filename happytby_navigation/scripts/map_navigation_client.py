#!/usr/bin/env python
from __future__ import print_function

import roslib
import rospy
import actionlib
import tf
import os
from math import sqrt, pi
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler, quaternion_from_matrix, quaternion_multiply



class map_navigation:
    def __init__(self):
        goal_list = rospy.get_param('~Goal')
        for i in range(len(goal_list)):
            print(str(i) + ':' + str(goal_list[i]))
        self.goal_input = int(input("Destination? (0~3)"))
        self.MoveToGoal(goal_list[self.goal_input])

    def MoveToGoal(self, dst):
        os.system('rosservice call /move_base/clear_costmaps')
        ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            print('Waiting for the move_base action server to come up')
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = dst

        print('Sending goal location', dst)
        ac.send_goal(goal)
        ac.wait_for_result(rospy.Duration(60))
        if(ac.get_state() ==  GoalStatus.SUCCEEDED):
            print('You have reached Goal[' + str(self.goal_input) + ']')
        else:
            print('The robot failed to reach Goal[' + str(self.goal_input) + ']')

if __name__ == '__main__':
    try:
        rospy.init_node('map_navigation', anonymous=False)
        map_nav = map_navigation()
        rospy.spin()

    except rospy.ROSInterruptException:
        print('map_navigation node terminated.')