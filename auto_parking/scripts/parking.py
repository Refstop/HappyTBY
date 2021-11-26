#!/usr/bin/env python
""" This is a script that autonomously parks the Neato either in parallel or perpendicular mode."""

import rospy
import numpy as np
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import LaserScan
import math

"""Global Variables"""
STOP = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))
Speed = 0.3
lw = 0.257

class ParkingNode(object):
    """Class for parking neato in either mode """
    def __init__(self):
        """Constructor for the class
        initialize topic subscription and 
        instance variables
        """
        self.r = rospy.Rate(5)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # self.pnum = rospy.get_param('park_number')
        # self.length_of_spot = rospy.get_param('lenc1') # length of parking spot = 37
        # self.lengtg_of_move = rospy.get_param('lenc2') # length of moving car
        # self.time_for_stop = rospy.get_param('time_for_stop')

        self.pnum = 1
        self.length_of_spot = 37
        self.time_for_stop = 5

        # Instance Variables
        self.widthOfSpot = 0.45
        self.twist = None
        self.radius = None
        self.start_park(self.pnum)
        # self.align_width()
        # rospy.signal_shutdown("Done parking.")
        # rospy.on_shutdown(self.stop)
        

               
       
    def stop(self):
        """This method publishes a twist to make the Neato stop."""
        self.publisher.publish(STOP)
        
    
    def align_with_origin1(self):
        self.drive_arc(0.3, 0.51, rospy.Duration(9), 1)
        self.twist = STOP

    def align_with_origin2(self, pnum):
        p_travelTime = rospy.Duration((self.length_of_spot * self.pnum) / Speed)
        if pnum is 1:
            # second parking spot
            self.drive_arc(Speed, 0, p_travelTime, 1)
        elif pnum is 2:
            # third parking spot
            self.drive_arc(Speed, 0, p_travelTime, 1)
        self.twist = STOP
        
    def drive_arc(self, speed, omega, travelTime, sign):
        '''given the omega, travel time and direction, drive in the corresponding arc'''
        # The third parameter (sign) represents whether the forward velocity of the twist will be positive or negative
        now = rospy.Time.now()
        while rospy.Time.now() - now <= travelTime:
            self.twist = Twist(linear=Vector3(sign*(speed),0,0), angular=Vector3(0,0,omega))
            self.publisher.publish(self.twist)
	    #print self.twist


    def start_park(self, pnum):
        self.drive_arc(0.01, 0, rospy.Duration(1), 1)  
        self.align_with_origin1()
        self.drive_arc(0.03, 0, rospy.Duration(1), 1)
        self.align_with_origin2()
        self.park(1) # move in
        self.park(-1) # move out
        self.drive_arc(0.03, 0, rospy.Duration(1), 1)
        self.align_with_origin2
        self.align_with_origin1

    def park(self, sign): # sign * omega
        self.drive_arc(0.3, 0, rospy.Duration(0.4), 1)
        self.drive_arc(0.3, 1, rospy.Duration(1.8), 1)
        self.drive_arc(0.3, 1, rospy.Duration(0.9), -1)

        self.drive_arc(0.3, 1, rospy.Duration(1.1), 1)
        self.drive_arc(0.3, 1, rospy.Duration(2), -1)

        self.drive_arc(0.3, 1, rospy.Duration(1.5), 1)
        self.drive_arc(0.3, 1, rospy.Duration(1), -1)
        self.drive_arc(0.3, 1, rospy.Duration(1), 1)

        self.drive_arc(0, 0, rospy.Duration(self.time_for_stop), 1) # stop for 5sec

        
    def run(self):
        """ This function is the main run loop."""
        rospy.on_shutdown(self.stop)
        while not rospy.is_shutdown():
            self.park()
            if self.twist:
                self.publisher.publish(self.twist)
            self.r.sleep()

            
if __name__ == '__main__':            
    rospy.init_node('parking')
    parking_node = ParkingNode()
    parking_node.stop()
    # rospy.on_shutdown(parking_node.stop)
    # parking_node.run()
