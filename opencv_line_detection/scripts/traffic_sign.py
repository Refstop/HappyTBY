#!/usr/bin/env python
from __future__ import print_function

import roslib
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes



class traffic_sign:
    def __init__(self):
        self.img_sub = rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,self.imgCallback)
        self.darknet_sub = rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,self.Callback)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel_ts", Twist, queue_size=1)
        self.tl_pub = rospy.Publisher("isTL", Bool, queue_size=1)
        self.NoOvertaking_pub = rospy.Publisher("isNoOver", Bool, queue_size=1)
        self.Park_pub = rospy.Publisher("isNoPark", Bool, queue_size=1)

        self.bbox = 10
        self.cv_image = []

    def imgCallback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    # 0: trafficlight 1: NoChuwall 2: NoPark 3: Stop 4: GoStraight 5: TurnRight 6: TurnLeft 7: Park 8: Cross
    def Callback(self, data):
        self.bbox = data.bounding_boxes[0]
        if bbox.id == 0:
            print('0: trafficlight')
            self.tl_pub.publish(Bool(True))
        elif bbox.id == 1:
            print('1: NoChuwall')
            self.NoOvertaking_pub.publish(Bool(True))
        elif bbox.id == 2:
            print('2: NoPark')
            self.Park_pub.publish(Bool(False))
        elif bbox.id == 3:
            print('3: Stop')
            self.cmd_vel_pub.publish(Twist(linear=Vector3(0.0,0.0,0.0), angular=Vector3(0.0,0.0,0.0)))
        elif bbox.id == 4:
            print('4: Gostraight')
            self.cmd_vel_pub.publish(Twist(linear=Vector3(0.2,0.0,0.0), angular=Vector3(0.0,0.0,0.0)))
        elif bbox.id == 5 or bbox.id == 6:
            dx = int((bbox.xr-bbox.xl)/2), dy = int((bbox.yr-bbox.yl)/2)
            left_side = np.sum(self.cv_image[bbox.xl : bbox.xl + dx, bbox.yl + dy : bbox.yr])
            right_side = np.sum(self.cv_image[bbox.xl + dx : bbox.xr, bbox.yl + dy : bbox.yr])
            if left_side > right_side:
                print('5: TurnRight')
                self.DriveByTravelTime(Twist(linear=Vector3(0.2,0.0,0.0), angular=Vector3(0.0,0.0,1)), rospy.Duration(5.8))
            elif left_side < right_side:
                print('6: TurnLeft')
                self.DriveByTravelTime(Twist(linear=Vector3(0.2,0.0,0.0), angular=Vector3(0.0,0.0,-1)), rospy.Duration(5.8))
        elif bbox.id == 7:
            print('7: Park')
            self.Park_pub.publish(Bool(True))
        elif bbox.id == 8:
            print('8: Cross')
            self.cmd_vel_pub.publish(Twist(linear=Vector3(0.08,0.0,0.0), angular=Vector3(0.0,0.0,0.0)))

    def DriveByTravelTime(self, vel, travelTime):
        now = rospy.Time.now()
        while rospy.Time.now() - now <= travelTime:
            self.pub.publish(vel)

if __name__ == '__main__':
    try:
        rospy.init_node('traffic_sign', anonymous=False)
        ts = traffic_sign()
        rospy.spin()

    except rospy.ROSInterruptException:
        print('map_navigation node terminated.')