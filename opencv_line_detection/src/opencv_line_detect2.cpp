#include <ros/ros.h> 
#include <sensor_msgs/Image.h> 
#include <image_transport/image_transport.h> 
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h> 
#include "geometry_msgs/Twist.h" 
#define ASSIST_BASE_LINE 420
#define ASSIST_BASE_WIDTH 60
#define PERSPECTIVE_IMG_W 640
#define PERSPECTIVE_IMG_H 480
using namespace cv; 
using namespace std;

geometry_msgs::Twist msg;
sensor_msgs::ImagePtr img_msg;

Mat Region_of_Interest_crop(Mat image, Point* points)
{
    Mat img_roi_crop;
    Rect bounds(0, 0, image.cols, image.rows);
    Rect r(points[0].x, points[0].y, image.cols, points[2].y - points[0].y);
    img_roi_crop = image(r & bounds);
    return img_roi_crop;

}

 

void ImageCallbak(const sensor_msgs::Image::ConstPtr &img) 
{
  double spdCurve = 0; 
  cv_bridge::CvImagePtr cv_ptr; 
  ROS_INFO("Image(%d, %d)", img->width, img->height);

  try {
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Error to convert!");
    return;
  }

  Point points[4];
  points[0] = Point(0, ASSIST_BASE_LINE - ASSIST_BASE_WIDTH);
  points[1] = Point(0, ASSIST_BASE_LINE + ASSIST_BASE_WIDTH);
  points[2] = Point(640, ASSIST_BASE_LINE + ASSIST_BASE_WIDTH);
  points[3] = Point(640, ASSIST_BASE_LINE - ASSIST_BASE_WIDTH);
  double sum=0; 

  int cnt=0; 

  Mat src = cv_ptr->image; 

  //flip(src,src,0);
  //flip(src,src,1);
  Mat dst, color_dst,gray;

  // imshow( "Detected Lines",src);


  
  Mat adapt; 

//  flip(src,src,1);
//  flip(src,src,0);
  cvtColor( src, gray, COLOR_BGR2GRAY ); 

  adaptiveThreshold(gray, adapt, 255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY_INV,7,10); 

  //adapt=Region_of_Interest_crop(adapt, points);
  GaussianBlur(adapt, adapt, Size(7,7), 0); 
  Canny( adapt, dst, 200, 250, 3 ); 
  cvtColor( dst, color_dst, COLOR_GRAY2BGR );

  vector<Vec4i> lines; 

  HoughLinesP( dst, lines, 1, CV_PI/180, 110, 30, 30);

  for( size_t i = 0; i < lines.size(); i++ )

  { 
    // if(lines.size()>20){
    //   return;
    // }

      double m=(double(lines[i][3]-lines[i][1]))/double((lines[i][2]-lines[i][0])); 
      if(m<-0.6 || 0.6<m) {
        line( color_dst, Point(lines[i][0], lines[i][1]),
        Point( lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
        sum+=m; 
        cnt++; 
      }
      // else if(-0.1< m <0.1){
      //   msg.linear.x=0;
      // }
  }
  ROS_INFO("count of m = %d\n", cnt);
  if(cnt<10 && cnt>=6) { 

    ROS_INFO("sum of m = %f\n", sum);
    spdCurve = sum / 80; 
    msg.linear.x = 0.3; 
  }
  else if(cnt<7 && cnt>=4) { 
    ROS_INFO("sum of m = %f\n", sum);
    spdCurve = sum / 60; 
    msg.linear.x = 0.2; 
  }
  else {
    int i,j;
    int x1=0,y1=0,x2=0,y2=0, pCnt=0, y_mean=0;
    double mCurve;
    for(pCnt=0, i=290, j=479; j>0; j--){
      if(dst.at<uchar>(j,i) > 50){ pCnt ++; if(pCnt >= 3){x2=i; y2=j; break;}}
      if(j==101){x2=320; y2=0; break;}
    }
    ROS_INFO("%d, %d, %d\n", x2, y2, pCnt);
    for(pCnt=0, i=450, j=479; j>0; j--){
      if(dst.at<uchar>(j,i) > 50){ pCnt ++; if(pCnt >= 3){x1=i; y1=j; break;}}
      if(j==101){x1=480; y1=0; break;}
    }
    ROS_INFO("%d, %d %d\n", x1, y1, pCnt);
    y_mean=(y1+y2)/2;
    ROS_INFO("%d\n", y_mean);
    if(y2-y1){mCurve = double(x2-x1)/double(y2-y1);}
    ROS_INFO("inverse of m = %f\n", mCurve);
    if(mCurve>0) {
      spdCurve=mCurve*(1.17);

    }
    else if(mCurve<0) {
      spdCurve=mCurve*(1.17);   
    }
    if(200-y_mean > 0){
      ROS_INFO("part 1");
      msg.linear.x = 0.2+double((160-y_mean))/double(500);
    }
    else{
      ROS_INFO("part 2"); 
      msg.linear.x = 0.22+double((160-y_mean))/double(150);
    }

    //ROS_INFO("linear.x_spd = %f\n",0.1+double(190-y_mean)/double(600));

  }
  // else 
  // {
  //   spdCurve=0;
  //   msg.linear.x=1;
  // }

  img_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, color_dst).toImageMsg();
  //imshow( "Detected Lines",color_dst);
  // imwrite("tby.jpg",color_dst);
  msg.angular.z = -spdCurve;

  //waitKey(1);

}

 

int main(int argc, char **argv) {
  ros::init(argc, argv, "main"); 

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh); 
  image_transport::Subscriber sub_img = it.subscribe("/main_camera/image_raw", 1, ImageCallbak); 
  image_transport::Publisher pub_img = it.advertise("/opencv_line_detect/image", 1); 
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Rate rate(10);
  ROS_INFO("Starting to move forward"); 
  while (ros::ok()) { 
    pub.publish(msg);
    pub_img.publish(img_msg);
          
    ros::spinOnce(); 

    rate.sleep();

  }

}
