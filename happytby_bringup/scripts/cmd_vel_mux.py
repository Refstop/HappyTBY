#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

global pub

cmd_vel_cur = Twist()

def parkcb(data):
    cmd_vel_cur = data
    pub.publish(cmd_vel_cur)

def tlcb(data):
    cmd_vel_cur = data
    pub.publish(cmd_vel_cur)

def tscb(data):
    cmd_vel_cur = data
    pub.publish(cmd_vel_cur)

def overcb(data):
    cmd_vel_cur = data
    pub.publish(cmd_vel_cur)

def linecb(data):
    cmd_vel_cur = data
    pub.publish(cmd_vel_cur)


if __name__ == '__main__': 
  try:
    rospy.init_node('cmd_vel_mux')
    rospy.Subscriber("cmd_vel_park", Twist, parkcb, queue_size=1)
    rospy.Subscriber("cmd_vel_tl", Twist, tlcb, queue_size=1)
    rospy.Subscriber("cmd_vel_ts", Twist, tscb, queue_size=1)
    rospy.Subscriber("cmd_vel_over", Twist, overcb, queue_size=1)
    rospy.Subscriber("cmd_vel_line", Twist, linecb, queue_size=1)
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass