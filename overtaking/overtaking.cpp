#include <ros/ros.h> 
#include <sensor_msgs/Image.h> 
#include "geometry_msgs/Twist.h" 



using namespace cv; 

using namespace std; 

sensor_msgs::LaserScan laser; 
int state = 0;



void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laser = msg->ranges[0];
}

void velCallback(geometry_msgs::Twist::ConstPtr& vel)
{
   geometry_msgs::Twist new_vel 
   geometry_msgs::Twist msg = *vel;
   if (state == 1) {
     new_vel.angular.z = -0.65;
     new_vel.linear.x = 1; 
     state = 0;   
   }
   else 
   {    
       new_vel.angular.z = 0.65;
       new_vel.linear.x = 1;
       state = 1; 
   }
   pub.publish(new_vel);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "overtaking"); 
  ros::NodeHandle nh; 
  

  image_transport::ImageTransport it(nh);
  ros::Subscriber sub = nh.subscribe("/scan", 1, scanCallback);
  
    
  ros::Subscriber sub = n.subscribe("cmd_vel", 0, velCallback);
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Rate rate(10);.
  ROS_INFO("Starting to move forward"); 
  while (ros::ok()) { 
           
          ros::spinOnce(); 
          rate.sleep();
          }
}