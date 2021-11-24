#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

using namespace std;

float laser = 1000;
int state = 0;
bool isNoOver = false;

void IsNoOverCallback(const std_msgs::Bool::ConstPtr &msg) {
    IsNoOver = true;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    laser = msg->ranges[0];
    if(laser < 0.2) {
        double traveltime = 2;
        if (state == 1) {
            double now = ros::Time::now().toSec();
            while(ros::Time::now() - now <= traveltime) {
                new_vel.angular.z = -1;
                new_vel.linear.x = 0.3; 
                pub.publish(new_vel);
            }
            state = 0;
        }
        else {
            double now = ros::Time::now().toSec();
            while(ros::Time::now() - now <= traveltime) {
                new_vel.angular.z = 1;
                new_vel.linear.x = 0.3; 
                pub.publish(new_vel);
            }
            state = 1;
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "overtaking"); 
    ros::NodeHandle nh; 
    ros::Subscriber scan_sub = nh.subscribe("scan", 1, scanCallback);
    ros::Subscriber isNoOver_sub = nh.subscribe("isNoOver", 1, IsNoOverCallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_over", 10);
    ros::Rate rate(10);
    ROS_INFO("Starting to move forward"); 
    while (ros::ok()) {
        if(IsNoOver) break;
        ros::spinOnce(); 
        rate.sleep();
    }
}