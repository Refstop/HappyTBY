#!/usr/bin/env python
from __future__ import print_function

""" This is a script that autonomously parks the Neato either in parallel or perpendicular mode."""

import rospy
from sensor_msgs.msg import Image


def callback(data):
    print(data.encoding)

if __name__ == '__main__': 
  try:
    rospy.init_node('aaa')
    rospy.Subscriber("main_camera/image_raw", Image, callback, queue_size=1)
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
    
