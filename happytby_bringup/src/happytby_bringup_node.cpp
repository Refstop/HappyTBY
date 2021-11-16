#define DEBUG 0
#define DEBUG_ROS_INFO 1
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
// #include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <sstream>
#include <cmath>

//i2c address
#define ADDRESS 0x05

//I2C bus
static const char * deviceName = "/dev/i2c-0";

#define MAX_L_STEER - 30
#define MAX_R_STEER 30
#define STEER_NEUTRAL_ANGLE 90

#define RAD2DEG(x)((x) * 180. / M_PI)
#define DEG2RAD(x)((x) / 180. * M_PI)
// unit : m
#define Sonar_Detect_Range 0.3
#define LIDAR_Obastacle 0.6
#define LIDAR_Wall_Detection 0.30

#define WHEEL_RADIUS 0.0365
#define COOUNT_PER_REV 2600
int Base_Speed = 130;

//////////////////////////////// Odometry ////////////////////////////

struct BaseSensorData {
    int encoder = 0;
    int encoder_old = 0;
} myBaseSensorData;

struct OdomCaculateData {
    //motor params
    // float distance_ratio = (67.6 * 3.14159 / 1000) / 2200; //0.000176; unit: m/encoder  67.5
    float distance_ratio = (2*WHEEL_RADIUS*M_PI) / COOUNT_PER_REV;
    //odom result
    float position_x = 0.0; //unit: m
    float position_y = 0.0; //unit: m
    float oriention = 0.0; //unit: rad
    float velocity_linear = 0.0; //unit: m/s
    float velocity_angular = 0.0; //unit: rad/s
} myOdomCaculateData;

int steering_angle = STEER_NEUTRAL_ANGLE;
int motor_speed = 0;

int steering_angle_old = STEER_NEUTRAL_ANGLE;
int motor_speed_old = 0;

float sonar = 0;
long encoder_temp = 0;
long encoder = 0;

unsigned char protocol_data[7] = {'#', 0, 0, 0, 0, 0, '*'}; // start byte '#' - end bytte '*'
char frameid[] = "/sonar_range";

int file_I2C;

int open_I2C(void) {
    int file;

    if ((file = open(deviceName, O_RDWR)) < 0) {
        fprintf(stderr, "I2C: Failed to access %s\n", deviceName);
        exit(1);
    }
    printf("I2C: Connected\n");

    printf("I2C: acquiring buss to 0x%x\n", ADDRESS);
    if (ioctl(file, I2C_SLAVE, ADDRESS) < 0) {
        fprintf(
            stderr,
            "I2C: Failed to acquire bus access/talk to slave 0x%x\n",
            ADDRESS
        );
        exit(1);
    }

    return file;
}

void close_I2C(int fd) {
    close(fd);
}

void CarControlCallback(const ackermann_msgs::AckermannDriveStamped & msg) {
    steering_angle = (int)msg.drive.steering_angle;

    if (steering_angle >= 30 + STEER_NEUTRAL_ANGLE)
        steering_angle = STEER_NEUTRAL_ANGLE + 30;
    if (steering_angle <= -30 + STEER_NEUTRAL_ANGLE)
        steering_angle = STEER_NEUTRAL_ANGLE - 30;
    
    Base_Speed = (int)msg.drive.speed;
    motor_speed = Base_Speed;
    if (motor_speed >= 250) 
        motor_speed = 250;
    if (motor_speed <= -250) 
        motor_speed = -250;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr & scan) {
    int count = (int)(360. / RAD2DEG(scan -> angle_increment));
    int sum = 0;
    int sum_l = 0,
    sum_r = 0;
    double dist_y = 0.0;

    for (int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan -> angle_min + scan -> angle_increment * i);

        if (((degree >= 90 - 15) && (degree <= 90 + 15))) {
            dist_y = fabs(scan -> ranges[i] * sin(DEG2RAD(degree)));
            if ((dist_y <= LIDAR_Obastacle) && (dist_y > LIDAR_Wall_Detection)) {
                sum++;
            }
        }

        if (((degree >= 80 - 15) && (degree <= 80 + 15))) {
            if (fabs(scan -> ranges[i] * sin(DEG2RAD(degree))) <= LIDAR_Wall_Detection) {
                sum_r++;

            }
        }
        if ((degree >= 280 - 15) && (degree <= 280 + 15)) {
            dist_y = fabs(scan -> ranges[i] * sin(DEG2RAD(degree)));
            //printf("%6.3lf \n",dist_y);
            if (dist_y <= LIDAR_Wall_Detection) {
                sum_l++;

            }
        }
    }
    ROS_INFO("sum = %d", sum_l);

    if (sum_r >= 10) {
        //steering_angle  -= 1;
        ROS_INFO("Right Wall Detected !!");
    }

    if (sum_l >= 10) {
        //steering_angle  += 1;
        ROS_INFO("Left Wall Detected !!");
    }

    if (sum >= 10) {
        motor_speed = 0;
        ROS_INFO("Obstacle Detected !!");
    } else 
        motor_speed = Base_Speed;

}

unsigned long timestamp_now_nsec_old = 0;

void odometry_cal(void) {
    int d_encoder = myBaseSensorData.encoder - myBaseSensorData.encoder_old;
    float d_x, d_y, d_r, d_th;

    // d_encoder = (left_diff + right_diff)/2
    d_r = d_encoder * myOdomCaculateData.distance_ratio;

    if (motor_speed == 0) {
        d_th = 0;
    } else {
        d_th = DEG2RAD((steering_angle - STEER_NEUTRAL_ANGLE) * 0.008);
    }
    d_x = d_r * cos(myOdomCaculateData.oriention - d_th);
    d_y = d_r * sin(myOdomCaculateData.oriention - d_th);
    myOdomCaculateData.position_x += d_x; //unit: m
    myOdomCaculateData.position_y += d_y; //unit: m
    myOdomCaculateData.oriention -= d_th; //unit: rad

    ros::Time timestamp_now = ros::Time::now();
    unsigned long timestamp_now_nsec = timestamp_now.toNSec();
    unsigned long d_time = (timestamp_now_nsec - timestamp_now_nsec_old) / 1000000.0;
    timestamp_now_nsec_old = timestamp_now_nsec;

    myOdomCaculateData.velocity_linear = d_r/float(d_time);
    myOdomCaculateData.velocity_angular = d_th/float(d_time);

    if (DEBUG == 1) 
        printf(" %6.3lf %6.3lf %6.3lf \n", myOdomCaculateData.position_x, myOdomCaculateData.position_y, RAD2DEG(myOdomCaculateData.oriention));
}

int main(int argc, char ** argv) {
    char buf[2];
    ros::init(argc, argv, "happytby_bringup");

    ros::NodeHandle n;
    std::string cmd_vel_topic = "cmd_vel";
    std::string odom_pub_topic = "odom";
    /*other*/
    ros::param::get("~cmd_vel_topic", cmd_vel_topic);
    ros::param::get("~odom_pub_topic", odom_pub_topic);

    ros::Subscriber sub1 = n.subscribe("/ackermann_cmd", 10, &CarControlCallback);
    ros::Subscriber sub3 = n.subscribe<sensor_msgs::LaserScan>( "/scan", 1000, &scanCallback);

    // ros::Publisher sonar_pub = n.advertise<sensor_msgs::Range>( "/RangeSonar", 10);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>(odom_pub_topic, 20);
    
    ////////////////   sonar _sensor ////////////////////
    sensor_msgs::Range sonar_msg;
    sonar_msg.header.frame_id = frameid;
    sonar_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    sonar_msg.field_of_view = (30.0 / 180.0) * 3.14;
    sonar_msg.min_range = 0.0;
    sonar_msg.max_range = 1.550; //[unit :m]

    //////////////////  odometry  ////////////////////
    std::string odom_frame_id = "odom";
    std::string odom_child_frame_id = "base_footprint";

    ros::Rate loop_rate(20); // 10
    file_I2C = open_I2C();
    if (file_I2C < 0) {
        ROS_ERROR_STREAM("Unable to open I2C");
        return -1;
    } else {
        ROS_INFO_STREAM("I2C is Connected");
    }
    
    int count = 0;
    static tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    nav_msgs::Odometry odom;
    geometry_msgs::Quaternion odom_quat;

    //covariance matrix
    float covariance[36] = {
        0.01, 0, 0, 0, 0, 0, // covariance on gps_x
        0, 0.01, 0, 0, 0, 0, // covariance on gps_y
        0, 0, 99999, 0, 0, 0, // covariance on gps_z
        0, 0, 0, 99999, 0, 0, // large covariance on rot x
        0, 0, 0, 0, 99999, 0, // large covariance on rot y
        0, 0, 0, 0, 0, 0.01
    };

    for (int i = 0; i < 36; i++) {
        odom.pose.covariance[i] = covariance[i];
    }

    while (ros::ok()) {
        std_msgs::String msg;
        std_msgs::Int16 steerangle;
        std_msgs::Int16 carspeed;
        std::stringstream ss;
        std::string data;
        data = std::to_string(steering_angle);

        steerangle.data = steering_angle;
        carspeed.data = motor_speed;
        ss << data;
        msg.data = ss.str();
        ROS_INFO("Steer : %s", msg.data.c_str());

        data = std::to_string(motor_speed);
        msg.data = data;
        ROS_INFO("Speed : %s", msg.data.c_str());
        
        if ((steering_angle != steering_angle_old) || (motor_speed != motor_speed_old)) {
            // write
            protocol_data[0] = '#';
            protocol_data[1] = 'C';
            protocol_data[2] = (steering_angle & 0xff00) >> 8;
            protocol_data[3] = (steering_angle & 0x00ff);
            protocol_data[4] = (motor_speed & 0xff00) >> 8;
            protocol_data[5] = (motor_speed & 0x00ff);
            protocol_data[6] = '*';

            write(file_I2C, protocol_data, 7);

            // read
            read(file_I2C, buf, 8);
            sonar = (buf[1] * 256 + buf[2]) / 1000.0;

            encoder = encoder_temp = 0;
            long temp[4] = {0};
            temp[0] = buf[3];
            

            if (((buf[3] & 0x80) >> 7) == 1) { //check MSB bit is 1 -> negative
                temp[0] = (long)255 - buf[3];
                temp[1] = (long)255 - buf[4];
                temp[2] = (long)255 - buf[5];
                temp[3] = (long)256 - buf[6];

                encoder_temp = (temp[0] << 24) + (temp[1] << 16) + (temp[2] << 8) + (temp[3]);
            }
            else {
                temp[1] = (long)buf[4];
                temp[2] = (long)buf[5];
                temp[3] = (long)buf[6];

                encoder_temp = (temp[0] << 24) + (temp[1] << 16) + (temp[2] << 8) + (temp[3]);
            }
            
            encoder = encoder_temp;
        }
        else {
            protocol_data[0] = '#';
            protocol_data[1] = 'S';
            protocol_data[2] = (steering_angle_old & 0xff00) >> 8;
            protocol_data[3] = (steering_angle_old & 0x00ff);
            protocol_data[4] = (motor_speed_old & 0xff00) >> 8;
            protocol_data[5] = (motor_speed_old & 0x00ff);
            protocol_data[6] = '*';

            write(file_I2C, protocol_data, 7);
            read(file_I2C, buf, 8);
            sonar = (buf[1] * 256 + buf[2]) / 1000.0;

            encoder = encoder_temp = 0;
            long temp[4] = {0};
            temp[0] = buf[3];

            if (((buf[3] & 0x80) >> 7) == 1) { //check MSB bit is 1 -> negative
                temp[1] = (long)buf[4];
                temp[2] = (long)buf[5];
                temp[3] = (long)buf[6];

                encoder_temp = (temp[0] << 24) + (temp[1] << 16) + (temp[2] << 8) + (temp[3]);
                // ROS_INFO("%d %d %d %d\n", (temp[0]<<24), (temp[1]<<16), (temp[2]<<8), temp[3]);
                // ROS_INFO("aaa %d\n", (temp[0]<<24)+(temp[1]<<16)+(temp[2]<<8)+temp[3]);
                // ROS_INFO("encoder %d \n", encoder_temp);
            }
            else {
                temp[1] = (long)buf[4];
                temp[2] = (long)buf[5];
                temp[3] = (long)buf[6];

                encoder_temp = (temp[0] << 24) + (temp[1] << 16) + (temp[2] << 8) + (temp[3]);
                // ROS_INFO("%d %d %d %d\n", (temp[0]<<24)+(temp[1]<<16)+(temp[2]<<8)+temp[3]);
                // ROS_INFO("bbb %d\n", (temp[0]<<24)+(temp[1]<<16)+(temp[2]<<8)+temp[3]);
                // ROS_INFO("encoder %d \n", encoder_temp);
            }
            encoder = encoder_temp;

        }

        steering_angle_old = steering_angle;
        motor_speed_old = motor_speed;
        
        if (DEBUG == 1) {
            printf("sonar %6.3lf \n", sonar);
            printf("encoder temp %ld \n", encoder_temp);
            printf("encoder %ld \n", encoder);
        }

        //////////////////// sonar obstacle detectioin ////////////////
        if ((sonar > 0) && (sonar <= Sonar_Detect_Range)) {

            ROS_INFO("Sonar Obstacle detection : %4.1lf", sonar);
            protocol_data[0] = '#';
            protocol_data[1] = 'C';
            protocol_data[4] = 0;
            protocol_data[5] = 0;
            write(file_I2C, protocol_data, 7);
            read(file_I2C, buf, 8);

            sonar = (buf[2] * 256 + buf[3]) / 1000.0;

            encoder = encoder_temp = 0;
            long temp[4] = {0};
            temp[0] = buf[3];

            if (((buf[3] & 0x80) >> 7) == 1) { //check MSB bit is 1 -> negative
                temp[0] = (long)(255 - buf[3]);
                temp[1] = (long)(255 - buf[4]);
                temp[2] = (long)(255 - buf[5]);
                temp[3] = (long)(256 - buf[6]);

                encoder_temp = -(temp[0] + temp[1] + temp[2] + temp[3]);
            }
            else {
                temp[1] = (long)buf[4];
                temp[2] = (long)buf[5];
                temp[3] = (long)buf[6];

                encoder_temp = temp[0] + temp[1] + temp[2] + temp[3];
            }
            encoder = encoder_temp;
        } else {
            motor_speed = Base_Speed;
        }


        sonar_msg.header.stamp = ros::Time::now();
        sonar_msg.range = sonar;
        // sonar_pub.publish(sonar_msg);
        // ROS_INFO("Sonar :%5.1lf", sonar);

        myBaseSensorData.encoder = encoder;
        odometry_cal();
        myBaseSensorData.encoder_old = myBaseSensorData.encoder;

        //odom_oriention trans to odom_quat
        odom_quat = tf::createQuaternionMsgFromYaw(myOdomCaculateData.oriention); //yaw trans quat

        //pub tf(odom->base_footprint)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = odom_frame_id;
        odom_trans.child_frame_id = odom_child_frame_id;
        odom_trans.transform.translation.x = myOdomCaculateData.position_x;
        odom_trans.transform.translation.y = myOdomCaculateData.position_y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //pub odom
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = odom_frame_id;
        odom.child_frame_id = odom_child_frame_id;
        odom.pose.pose.position.x = myOdomCaculateData.position_x;
        odom.pose.pose.position.y = myOdomCaculateData.position_y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom.twist.twist.linear.x = myOdomCaculateData.velocity_linear;
        odom.twist.twist.linear.y = odom.twist.twist.linear.z = 0;
        odom.twist.twist.angular.z = myOdomCaculateData.velocity_angular;
        odom.twist.twist.angular.x = odom.twist.twist.angular.y = 0;
        odom_broadcaster.sendTransform(odom_trans);
        odom_pub.publish(odom);

        loop_rate.sleep();
        ros::spinOnce();
        ++count;
    }

    motor_speed = 0;
    protocol_data[0] = '#';
    protocol_data[1] = 'C';
    protocol_data[2] = (steering_angle & 0xff00) >> 8;
    protocol_data[3] = (steering_angle & 0x00ff);
    protocol_data[4] = (motor_speed & 0xff00) >> 8;
    protocol_data[5] = (motor_speed & 0x00ff);
    protocol_data[6] = '*';
    write(file_I2C, protocol_data, 7);
    read(file_I2C, buf, 8);
    close_I2C(file_I2C);
    return 0;
}