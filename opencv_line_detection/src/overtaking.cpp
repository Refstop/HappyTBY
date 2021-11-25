#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h> 

using namespace std;

class overtaking {
    private:
    float laser;
    bool isNoOver;
    bool isObstacle;
    bool chuwalling;
    bool isStop;
    int checking;
    geometry_msgs::Twist new_vel;
    std_msgs::Bool wpstart;
    ros::NodeHandle nh; 
    ros::Subscriber scan_sub;
    ros::Subscriber isNoOver_sub;
    ros::Subscriber isStop_sub;
    ros::Publisher cmd_vel_pub;
    ros::Publisher waypoint_pub;

    public:
    overtaking() {
        laser = 1000;
        isNoOver = false;
        isObstacle = true;
        chuwalling = false;
        isStop = false;
        checking = 0; 
        scan_sub = nh.subscribe("/scan", 1, &overtaking::scanCallback, this);
        isNoOver_sub = nh.subscribe("/isNoOver", 1, &overtaking::IsNoOverCallback, this);
        isStop_sub = nh.subscribe("/isStop", 1, &overtaking::IsStopCallback, this);
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_over", 10);
        waypoint_pub = nh.advertise<std_msgs::Bool>("isWaypointStart", 10);
    }
    void IsNoOverCallback(const std_msgs::Bool::ConstPtr &msg) {
        isNoOver = msg->data; //true
    }

    void IsStopCallback(const std_msgs::Bool::ConstPtr &msg) {
        isStop = msg->data; //true
    }
    
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
        if(laser > 0.3 && isStop && chuwalling) {
            wpstart.data = true;
            waypoint_pub.publish(wpstart);
            exit(0);
        }
        if(isNoOver) exit(0);
        double traveltime = 0, now = 0;
        laser = msg->ranges[0];
        if(checking == 0) {
            if(laser < 0.3) {
                if(isObstacle) {
                    traveltime = 5;
                    now = ros::Time::now().toSec();
                    while(ros::Time::now() - now <= traveltime) {
                        new_vel.angular.z = 0;
                        new_vel.linear.x = 0; 
                        cmd_vel_pub.publish(new_vel);
                    }
                    checking = 1;
                    return;
                }
            }
        }
        else if(checking == 1) {
            if(laser < 0.3) {
                isObstacle = true;
            }
            else {
                isObstacle = false;
            }
        }
    
        if(~isObstacle) {
            traveltime = 2;
            now = ros::Time::now().toSec();
            while(ros::Time::now() - now <= traveltime) {
                new_vel.angular.z = 1;
                new_vel.linear.x = 0.3; 
                cmd_vel_pub.publish(new_vel);
            }
            now = ros::Time::now().toSec();
            while(ros::Time::now() - now <= traveltime) {
                new_vel.angular.z = 1;
                new_vel.linear.x = -0.3; 
                cmd_vel_pub.publish(new_vel);
            }
            checking = 0;
            chuwalling = true;
        }
    }
}
    
    

int main(int argc, char **argv) {
    ros::init(argc, argv, "overtaking"); 
    ros::Rate rate(10);
    overtaking ot;
    ros::spin();
    // while (ros::ok()) {
    //     ros::spinOnce(); 
    //     rate.sleep();
    // }
    return 0;
}