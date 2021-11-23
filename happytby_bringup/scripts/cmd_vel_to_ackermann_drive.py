#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
  if omega == 0 or v == 0:
    return math.pi/2 * 180/math.pi

  
  # max_omega = 0.3
  # if omega >= max_omega:
  #   omega = max_omega
  # elif omega <= -max_omega:
  #   omega = -max_omega
  # return (-omega/0.3*40 + math.pi/2) * 180/math.pi
  radius = v / omega
  return (-math.atan(wheelbase / radius) + math.pi/2) * 180/math.pi
  


def cmd_callback(data):
  global wheelbase
  global ackermann_cmd_topic
  global frame_id
  global pub
  
  v = data.linear.x
  steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)
  
  msg = AckermannDriveStamped()
  msg.header.stamp = rospy.Time.now()
  msg.header.frame_id = frame_id
  msg.drive.steering_angle = steering
  msg.drive.speed = linear_speed_to_motor_input(v)
  
  pub.publish(msg)

def linear_speed_to_motor_input(linear_speed):
  max_speed = 0.3
  if linear_speed >= max_speed:
    linear_speed = max_speed
  elif linear_speed <= -max_speed:
    linear_speed = -max_speed

  return int(linear_speed*(255/max_speed))


if __name__ == '__main__': 
  try:
    
    rospy.init_node('cmd_vel_to_ackermann_drive')
    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
    ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd')
    wheelbase = rospy.get_param('~wheelbase', 0.257)
    frame_id = rospy.get_param('~frame_id', 'odom')
    
    rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)
    pub = rospy.Publisher(ackermann_cmd_topic, AckermannDriveStamped, queue_size=1)
    
    rospy.loginfo("Node 'cmd_vel_to_ackermann_drive' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f", "/cmd_vel", ackermann_cmd_topic, frame_id, wheelbase)
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass