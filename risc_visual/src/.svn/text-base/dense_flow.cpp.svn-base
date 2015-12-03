
/*======================================================
	Created by:  	D. Spencer Maughan
	Last updated: 	May 2014
	File name: 		dense_flow.cpp
	Organization:	RISC Lab, Utah State University
 ======================================================*/

#include <ros/ros.h>
#include "ardrone_autonomy/Navdata.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <cv.h>

#define PI 3.14159265

using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;
 
//Setup windows
static const char WINDOW1[] = "Edges";

//Variables
cv::Mat frame_old;
cv::Mat frame_new;
cv::Mat flow;
int winsize = 15;
int iterations = 3;
int polyn = 5;
int blursize = 2;
int lowThreshold = 8000;

//needed to publish an image
image_transport::Publisher pub;


void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("ros_opencv::edge.cpp::cv_bridge exception: %s", e.what());
        return;
    }
if (countNonZero(frame_old) < 1)
 {
frame_old = cvCreateMat(cv_ptr->image.rows,cv_ptr->image.cols,0);
}
int const max_lowThreshold = 10000;
int ratio = 3;
cvtColor( cv_ptr->image, cv_ptr->image, CV_BGR2GRAY );
int kernel_size = 7;
blur( cv_ptr->image, cv_ptr->image, Size(blursize*2+1,blursize*2+1) );

  /// Canny detector

Canny( cv_ptr->image, cv_ptr->image, lowThreshold, lowThreshold*ratio, kernel_size );
// Optical flow
frame_new = cv_ptr->image;
cv::calcOpticalFlowFarneback(frame_old, cv_ptr->image, flow, 0.5, 1, winsize , iterations, polyn, polyn*0.22, 0);
createTrackbar("blur size", WINDOW1, &blursize,10 );
createTrackbar("thrshold", WINDOW1, &lowThreshold, max_lowThreshold);
createTrackbar("winsize", WINDOW1, &winsize,30 );
createTrackbar("iterations", WINDOW1, &iterations,10 );
createTrackbar("polyn", WINDOW1, &polyn,10 );
frame_old = frame_new;

cv::Mat xy[2]; //X,Y
cv::split(flow, xy);

//calculate angle and magnitude
cv::Mat magnitude, angle;
cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);

//translate magnitude to range [0;1]
double mag_max;
cv::minMaxLoc(magnitude, 0, &mag_max);
magnitude.convertTo(magnitude, -1, 1.0/mag_max);

//build hsv image
cv::Mat _hsv[3], hsv;
_hsv[0] = angle;
_hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
_hsv[2] = magnitude;
cv::merge(_hsv, 3, hsv);

cv::cvtColor(hsv, cv_ptr->image, cv::COLOR_HSV2BGR);
cv::imshow (WINDOW1,cv_ptr->image);
cv::waitKey(3);
        pub.publish(cv_ptr->toImageMsg());
}
 
int main(int argc, char **argv)

{
        ros::init(argc, argv, "vertical_finder");
 
        ros::NodeHandle nh;

        image_transport::ImageTransport it(nh);

    cv::namedWindow(WINDOW1, CV_WINDOW_AUTOSIZE);

      image_transport::Subscriber sub = it.subscribe("ardrone/image_raw", 1, imageCallback);


    cv::destroyWindow(WINDOW1);

        pub = it.advertise("camera/dense", 1);
    
        ros::spin();
   
    ROS_INFO("ros_opencv::main.cpp::No error.");
 
}
