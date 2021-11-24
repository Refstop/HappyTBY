#!/usr/bin/env python

""" This is a script that autonomously parks the Neato either in parallel or perpendicular mode."""

import rospy
import numpy as np
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import LaserScan
import math

"""Global Variables"""
LENGTH_OF_SPOT = 0.40 # The parking spots are half a meter long.
STOP = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))
LENGTH_OF_CAR = 0.19
widthofCAR = 0.38
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

        # Instance Variables
        self.widthOfSpot = 0.45
        self.twist = None
        self.radius = None
        self.park()
        # self.align_width()
        # rospy.signal_shutdown("Done parking.")
        # rospy.on_shutdown(self.stop)

               
       
    def stop(self):
        """This method publishes a twist to make the Neato stop."""
        self.publisher.publish(STOP)
        
    
    # def align_with_origin(self):
    #     """After stopping next to the second parked Neato, 
    #     this function will align us properly so that we can successfully drive our circle."""
    #     dist = self.widthOfSpot/2 + self.radius
    #     travelTime = rospy.Duration(dist/0.2 + 0.5) # 0.2 is speed
    #     self.drive_arc(0.2, 0, travelTime, -1)
    #     self.twist = STOP

        
    def drive_arc(self, Speed, omega, travelTime, sign):
        '''given the omega, travel time and direction, drive in the corresponding arc'''
        # The third parameter (sign) represents whether the forward velocity of the twist will be positive or negative
        now = rospy.Time.now()
        while rospy.Time.now() - now <= travelTime:
            self.twist = Twist(linear=Vector3(sign*(Speed),0,0), angular=Vector3(0,0,omega))
            self.publisher.publish(self.twist)
	    #print self.twist


    def park(self):
        self.drive_arc(0.3, 0, rospy.Duration(0.2), 1)
        self.drive_arc(0.3, 1, rospy.Duration(1.8), 1)
        self.drive_arc(0.3, 1, rospy.Duration(1.2), -1)

        self.drive_arc(0.3, 1, rospy.Duration(1.6), 1)
        self.drive_arc(0.3, 1, rospy.Duration(0.58), -1)

        self.drive_arc(0.3, 1, rospy.Duration(1.5), 1)
        self.drive_arc(0.3, 1, rospy.Duration(0.65), -1)

        # self.drive_arc(0.3, 1, rospy.Duration(5.6), 1)

    def deg2r(self, degree):
        radius = math.sqrt(math.pow(lw/2, 2)+ math.pow(lw * 1/math.tan(math.radians(degree)), 2))
        print "radius : ", radius
        return radius

        
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
