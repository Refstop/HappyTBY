from __future__ import print_function

import cv2
import numpy as np
import sys
import math
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import image
import rospy
import roslib
from geometry_msgs.msg import Twist

fn = cv2.VideoCapture(0)
fn.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
fn.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
fn.set(cv2,cv2.CAP_PROP_FPS, 30)

rospy.init_node("traffic_light",anonymous= True)
image_pub = rospy.Publisher("traffic_light_result",image,queue_size=1)

rospy.init_node('traffic_light_move', anonymous=True)
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
vel_msg =Twist()
    
if __name__ == '__main__':
    print(__doc__)


    while not rospy.is_shutdown():
        
        src = fn.read()
        img = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
        img = cv2.medianBlur(img, 5)
        row,col,ch=src.shape
        cimg = src.copy() # numpy function

        circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 50,param1=150, param2=40, minRadius=20, maxRadius=80)
        position_y = []
        position_x = []
        radius = []
        if circles is not None:
            a, b, c = circles.shape
            if b>2:
                for i in range(b):
                    cv2.circle(cimg, (int(circles[0][i][0]), int(circles[0][i][1])), int(circles[0][i][2]), (0, 0, 255), 3, cv2.LINE_AA)
                    position_y.append(int(circles[0][i][1]))
                    position_x.append(int(circles[0][i][0]))
                    radius.append(int(circles[0][i][2]))
                    #position_x += int(circles[0][i][0])
                    #print(i,"circle's BGR value is",cimg[int(circles[0][2][1]),int(circles[0][2][0])])
                    #print(i,"circle's BGR value+1 is",cimg[int(circles[0][i][1]+int(circles[0][i][2])),int(circles[0][i][0]+int(circles[0][i][2]))])
        # if len(position_x)==1:
        #     color_1 = cimg[position_y,position_x]
        #     if color_1[1]>200 and color_y[2] >200:
        #         print("yellow")
        #     elif color_1[2]>200 and color_1[1]<60:
        #         print("red")
        #     elif color_1[1]>100:
        #             # color_yupper = cimg[i+int(radius_mean)-5,position_x[0]]
        #             # color_yunder = cimg[i-int(radius_mean)+5,position_x[0]]
        #             # print(color_yupper)
        #             # print(np.mean(int(color_yunder[1])+int(color_yupper[1])+int(color_y[1])))
        #             if cimg[position_y+(int)(radius_min/2)][position_x+(int)(radius_min/2)][0] > 200:
        #                 print("left")
        #             else:
        #                 print("green")
        position_y.sort()
        position_x.sort()
        print(position_y)
        print(position_x)
        # radius_mean = np.mean(radius)
        radius_min = np.min(radius)
        
        if (abs(np.mean(position_x)-position_x[0])<20):
            print("anj")
            for i in position_y:
                
                print(i,"circle's BGR value is",cimg[i,position_x[0]])
                color_y = cimg[i,position_x[0]]
                if color_y[1]>200 and color_y[2] > 200:
                    print("yellow")
                    vel_msg.linear.x = 0.1
                elif color_y[2]>200 and color_y[1]<60:
                    print("red")
                    vel_msg.linear.x = 0
                elif color_y[1]>100:
                    # color_yupper = cimg[i+int(radius_mean)-5,position_x[0]]
                    # color_yunder = cimg[i-int(radius_mean)+5,position_x[0]]
                    # print(color_yupper)
                    # print(np.mean(int(color_yunder[1])+int(color_yupper[1])+int(color_y[1])))
                    if cimg[i+(int)(radius_min/2)][position_x[0]+(int)(radius_min/2)][0] > 200:
                        print("left")
                        vel_msg.linear.x = 1
                        vel_msg.angular.z = 0.65
                    else:
                        print("green")
                        vel_msg.linear.x = 1
        elif (abs(np.mean(position_y)-position_y[0])<20):
            for i in position_x:
                print(i,"circle's BGR value is",cimg[position_y[0],i])
                color_x = cimg[position_x[0],i]
                if color_x[1]>200 and color_x[2] > 200:
                        print("yellow")
                        vel_msg.linear.x = 0.1
                elif color_x[2]>200 and color_x[1]<60:
                    print("red")
                    
                    vel_msg.linear.x = 0
                elif color_x[1]>100:
                    if cimg[position_y[0]+(int)(radius_min/2)][i+(int)(radius_min/2)][0] > 200:
                        print("left")
                        vel_msg.linear.x = 1
                        vel_msg.angular.z = 0.65
                    else:
                        print("green")
                        vel_msg.linear.x = 1
        cv2.imshow("detected circles", cimg)
        if cv2.waitKey(0) == 27:
            break 
       
       
        cv2.destroyAllWindows()