#!/usr/bin/env python
from __future__ import print_function

import cv2
import numpy as np
import sys
import math
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import rospy
import roslib
from geometry_msgs.msg import Vector3, Twist

class TrafficLight:
    def __init__(self):
        self.img_sub = rospy.Subscriber('/main_camera/image_raw', Image, self.ImageCallback)
        rospy.Subscriber('isTL', Bool, self.TLCallback)
        self.pub = rospy.Publisher('cmd_vel_tl', Twist, queue_size=10)
        self.STOP = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))
        self.SLOW = Twist(linear=Vector3(0.1,0,0), angular=Vector3(0,0,0))
        self.STRAIGHT = Twist(linear=Vector3(0.3,0,0), angular=Vector3(0,0,0))
        self.TURNLEFT = Twist(linear=Vector3(0.3,0,0), angular=Vector3(0,0,-1))
        self.isTL = False
        self.bridge = CvBridge()

    def TLCallback(self, data):
        self.isTL = True

    def ImageCallback(self, data):
        # if(not self.isTL): return

        try:
            src = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        img = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
        img = cv2.medianBlur(img, 5)
        row,col,ch=src.shape
        cimg = src.copy() # numpy function

        circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 20, param1=150, param2=40, minRadius=20, maxRadius=80)
        position_y = [], position_x = [], radius = []
        if circles is not None:
            a, b, c = circles.shape
            if b>2:
                for i in range(b):
                    cv2.circle(cimg, (int(circles[0][i][0]), int(circles[0][i][1])), int(circles[0][i][2]), (0, 0, 255), 3, cv2.LINE_AA)
                    position_y.append(int(circles[0][i][1]))
                    position_x.append(int(circles[0][i][0]))
                    radius.append(int(circles[0][i][2]))
        position_y.sort(), position_x.sort()
        print(position_y), print(position_x)
        radius_min = np.min(radius)
        
        if (abs(np.mean(position_x)-position_x[0])<20): # horizontal
            for i in position_y:
                print(i,"circle's BGR value is",cimg[i,position_x[0]])
                color_y = cimg[i,position_x[0]]
                if color_y[1]>200 and color_y[2] > 200:
                    print("yellow")
                    # self.vel_msg.linear.x = 0.1
                    self.DriveByTravelTime(self.SLOW, rospy.Duration(3))
                elif color_y[2]>200 and color_y[1]<60:
                    print("red")
                    # self.vel_msg.linear.x = 0
                    self.DriveByTravelTime(self.STOP, rospy.Duration(3))
                elif color_y[1]>100:
                    if cimg[i+(int)(radius_min/2)][position_x[0]+(int)(radius_min/2)][0] > 200:
                        print("left")
                        # self.vel_msg.linear.x = 1
                        # self.vel_msg.angular.z = 0.65
                        self.DriveByTravelTime(self.TURNLEFT, rospy.Duration(6))
                    else:
                        print("green")
                        # self.vel_msg.linear.x = 1
                        self.DriveByTravelTime(self.STRAIGHT, rospy.Duration(3))
        elif (abs(np.mean(position_y)-position_y[0])<20):

            for i in position_x:
                print(i,"circle's BGR value is",cimg[position_y[0],i])
                color_x = cimg[position_x[0],i]
                if color_x[1]>200 and color_x[2] > 200:
                        print("yellow")
                        # self.vel_msg.linear.x = 0.1
                        self.DriveByTravelTime(self.SLOW, rospy.Duration(3))
                elif color_x[2]>200 and color_x[1]<60:
                    print("red")
                    # self.vel_msg.linear.x = 0
                    self.DriveByTravelTime(self.STOP, rospy.Duration(3))
                elif color_x[1]>100:
                    if cimg[position_y[0]+(int)(radius_min/2)][i+(int)(radius_min/2)][0] > 200:
                        print("left")
                        # self.vel_msg.linear.x = 1
                        # self.vel_msg.angular.z = 0.65
                        self.DriveByTravelTime(self.TURNLEFT, rospy.Duration(6))
                    else:
                        print("green")
                        # self.vel_msg.linear.x = 1
                        self.DriveByTravelTime(self.STRAIGHT, rospy.Duration(3))
        cv2.imshow("detected circles", cimg)
        cv2.waitKey(0)

    def DriveByTravelTime(self, vel, travelTime):
        now = rospy.Time.now()
        while rospy.Time.now() - now <= travelTime:
            self.pub.publish(vel)
    
if __name__ == '__main__':
    rospy.init_node("traffic_light",anonymous= True)

    while not rospy.is_shutdown():
        try:
            tl = TrafficLight()
            rospy.spin()
        except rospy.ROSInterruptException:
            print('map_navigation node terminated.')