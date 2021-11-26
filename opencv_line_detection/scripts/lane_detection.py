#!/usr/bin/env python
from __future__ import print_function

import roslib,rospy,sys,cv2,time
import numpy as np
# roslib.load_manifest('lane_follower')
from std_msgs.msg import Int32
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Vector3

bridge = CvBridge()
pub = rospy.Publisher('lane_detection', Int32, queue_size=1) #ros-lane-detection
pub_image = rospy.Publisher('lane_detection_image',Image,queue_size=1)
pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

# STOP = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))

def callback(data):
    global x_last
    global z_last
    vel_msg = Twist(linear = Vector3(0.0, 0.0, 0.0), angular = Vector3(0.0, 0.0, 0.0))
    # convert image to cv2 standard format
    img = bridge.compressed_imgmsg_to_cv2(data)
    
    # start time
    start_time = cv2.getTickCount()

    # Gaussian Filter to remove noise
    img = cv2.medianBlur(img,5)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # print img.shape = (200,350,3)
    rows,cols,channels = img.shape
    rows_to_take = 150
    cols_to_take = 150

    # ROI
    roi_mask = np.zeros(img.shape,dtype=np.uint8)
    roi_mask[10:rows,0:cols] = 255
    street = cv2.bitwise_and(img,roi_mask)
    
    stop_roi_mask = np.zeros(gray.shape,dtype=np.uint8)
    stop_roi_mask[150:rows,150:250] = 255

    right_roi_mask = np.zeros(gray.shape,dtype=np.uint8)
    right_roi_mask[rows-rows_to_take:rows,cols-cols_to_take:cols] = 255
    right_roi = cv2.bitwise_and(img,img,right_roi_mask)
    
    left_roi_mask = np.zeros(gray.shape,dtype=np.uint8)
    left_roi_mask[rows-rows_to_take:rows,0:cols_to_take] = 255
    left_roi = cv2.bitwise_and(img,img,left_roi_mask)

    # define range of color in HSV
    hsv = cv2.cvtColor(street,cv2.COLOR_BGR2HSV)
    
    sensitivity = 160 # range of sensitivity=[90,150]
    lower_white = np.array([0,0,255-sensitivity])
    upper_white = np.array([255,sensitivity,255])
    
    white_mask = cv2.inRange(hsv,lower_white,upper_white)
    white_mask = cv2.erode(white_mask, None, iterations=2)
    white_mask = cv2.dilate(white_mask, None, iterations=2)

    lower_yellow = np.array([0,100,100]) #0,100,100
    upper_yellow = np.array([40,255,255]) #30,255,255

    yellow_mask = cv2.inRange(hsv,lower_yellow,upper_yellow)
    yellow_mask = cv2.erode(yellow_mask, None, iterations=2)
    yellow_mask = cv2.dilate(yellow_mask, None, iterations=2)
    
    # mask AND original img
    whitehsvthresh = cv2.bitwise_and(right_roi,right_roi,mask=white_mask)
    yellowhsvthresh = cv2.bitwise_and(street,street,mask=yellow_mask)
    
    # Canny Edge Detection 
    right_edges = cv2.Canny(whitehsvthresh,100,200)
    left_edges = cv2.Canny(yellowhsvthresh,100,200)
    
    right_edges = cv2.bitwise_and(right_edges,right_roi_mask)
    left_edges = cv2.bitwise_and(left_edges,left_roi_mask)

    # Standard Hough Transform
    right_lines = cv2.HoughLines(right_edges,0.8,np.pi/180,10)
    left_lines = cv2.HoughLines(left_edges,0.8,np.pi/180,10)
    
    xm = cols/2
    ym = rows
	    
    # Draw right lane
    theta_critic = 1.57
    theta_tollerance = 0.5
    x = []
    i = 0
    if right_lines is not None:
        right_lines = np.array(right_lines[0])
        for rho, theta in right_lines:
            if(np.abs(theta)<theta_critic-theta_tollerance or np.abs(theta)>theta_critic+theta_tollerance):
                a=np.cos(theta)
                b=np.sin(theta)
                x0,y0=a*rho,b*rho
                y3 = 140
                x3 = int(x0+((y0-y3)*np.sin(theta)/np.cos(theta)))
                x.insert(i,x3)
                i+1
                pt1=(int(x0+1000*(-b)),int(y0+1000*(a)))
                pt2=(int(x0-1000*(-b)),int(y0-1000*(a)))
                cv2.line(img,pt1,pt2,(255,0,0),2)

    if len(x) != 0:
        xmin = x[0]
        for k in range(0,len(x)):
            if x[k] < xmin and x[k] > 0:
                xmin = x[k]
        kr = int(np.sqrt(((xmin-xm)*(xmin-xm))+((y3-ym)*(y3-ym))))
    else:
        kr = 0
        xmin = 0

    # Draw left lane
    theta_tollerance_left=0.2
    x = []
    i = 0
    
    if left_lines is not None:
        left_lines = np.array(left_lines[0])
        for rho, theta in left_lines:
            if(np.abs(theta)<theta_critic-theta_tollerance_left or np.abs(theta)>theta_critic+theta_tollerance_left):
                a=np.cos(theta)
                b=np.sin(theta)
                x0,y0=a*rho,b*rho
                y3 = 140
                x3 = int(x0+((y0-y3)*np.sin(theta)/np.cos(theta)))
                x.insert(i,x3)
                i+1
                pt1=(int(x0+1000*(-b)),int(y0+1000*(a)))
                pt2=(int(x0-1000*(-b)),int(y0-1000*(a)))
                cv2.line(img,pt1,pt2,(0,255,0),2)

    if len(x) != 0:
        xmax = x[0]
        for k in range(0,len(x)):
            if x[k] > xmax and x[k]<cols:
                xmax = x[k]
            kl = int(np.sqrt(((xmax-xm)*(xmax-xm))+((y3-ym)*(y3-ym))))
    else:
        kl = 0
        xmax = 0

    error = kr - kl

    #end time
    end_time = cv2.getTickCount()
    time_count= (end_time - start_time) / cv2.getTickFrequency()
    
    #rospy.loginfo(time_count)
    if right_lines is not None and left_lines is not None:
        # rospy.loginfo(error)
        if error > 150:
            error = 150
        elif error < -150:
            error =-150
        print("g")
        
        message = error
        if 0<error<=150:
            vel_msg.linear.x = 0.24 
            vel_msg.angular.z = error/400
        elif -150<error<0:
            vel_msg.linear.x = 0.24 
            vel_msg.angular.z = -error/400
        
        # elif error<-150:
        #     vel_msg.linear.x = 0.24
        #     vel_msg.angular.z = -(kr/230)
        else:
            vel_msg.linear.x = 0.24 
            vel_msg.angular.z = -error/300
        

        x_last = vel_msg.linear.x
        z_last = vel_msg.angular.z
        rospy.loginfo(error)

    elif left_lines is not None and right_lines is None:
        rospy.loginfo("Turn Right")
        message = -152 #turn right
        if(kl>410):
            kl=410
        elif(kl==0):
            vel_msg.linear.x = 0.24
            vel_msg.angular.z = 0.3
        
        elif(0<kl<=410):
            vel_msg.linear.x = 0.24
            vel_msg.angular.z = kl/230
        x_last = vel_msg.linear.x
        z_last = vel_msg.angular.z
        print(kl)
    elif left_lines is None and right_lines is not None:
        rospy.loginfo("Turn Left")
        message = 153 #turn left
        if(kr>410):
            kr=410
        if(kr==0):  
            vel_msg.linear.x = 0.24
            vel_msg.angular.z = -0.3
        elif(0<kl<410):
            vel_msg.linear.x = 0.24
            vel_msg.angular.z = -(kr/230)
        
        x_last = vel_msg.linear.x
        z_last = vel_msg.angular.z
        print(kr)
        
    elif left_lines is None and right_lines is None:
        rospy.loginfo("No line")
        message = 0 #no line found
        vel_msg.linear.x = 0.8*x_last
        vel_msg.angular.z = 1.2*z_last
    else:
        message = 155 #no line found
        vel_msg.linear.x = 0.8*x_last
        vel_msg.angular.z = 1.2*z_last
    pub.publish(message)
    image = bridge.cv2_to_imgmsg(img, "bgr8")

    pub_image.publish(image)
    pub_cmd_vel.publish(vel_msg)

def lane_detection():
    rospy.init_node('lane-detection',anonymous=True)
    rospy.Subscriber("/main_camera/image_raw/compressed",CompressedImage,callback,queue_size=1,buff_size=2**24)
    #rospy.Subscriber("/usb_camera/image_raw/compressed", CompressedImage,callback,queue_size=1,buff_size=2**24)
    # rospy.on_shutdown(STOP)
    try:
        rospy.loginfo("Enetering ROS Spin")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    try:
        lane_detection()
    except rospy.ROSInterruptException:
        pass