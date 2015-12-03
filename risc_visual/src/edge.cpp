
/*======================================================
	Created by:  	D. Spencer Maughan
	Last updated: 	May 2014
	File name: 		edge.cpp
	Organization:	RISC Lab, Utah State University
 ======================================================*/


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ardrone_autonomy/Navdata.h"
#include <iostream>
#include <stdio.h>
#include <math.h>

#define PI 3.14159265

using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;
 
//Setup windows
static const char WINDOW1[] = "Edges";

//Declare trackbar variables

int blursize = 2;
int lowThreshold = 800;
//Global Variables
float theta;

//needed to publish an image
image_transport::Publisher pub;


void Roll(ardrone_autonomy::Navdata Navdata)
{
theta = Navdata.Rotx;
}


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

//createTrackbar("blur size", WINDOW1, &blursize,10 );

//createTrackbar("thrshold", WINDOW1, &lowThreshold, max_lowThreshold);

  //Draw vertical line
double x1;
double y1;
double x1;
double y1;
double w = cv_ptr->image.width;
double h = cv_ptr->image.height;
x1 = -0.5*w*sin(theta * PI / 180.0)+0.5*w;
y1 = -0.5*w*cos(theta * PI / 180.0)+0.5*h;
x2 = 0.5*w*sin(theta * PI / 180.0)+0.5*w;
y2 = 0.5*w*cos(theta * PI / 180.0)+0.5*h;

int thickness = 2;
int lineType = 8;
line( cv_ptr->image, Point(x1,y1), Point(x2,y2), Scalar( 0, 255, 0 ),thickness, lineType );

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

	ros::Subscriber blub = it.subscribe("/ardrone/navdata",1, Roll);

      image_transport::Subscriber sub = it.subscribe("ardrone/image_raw", 1, imageCallback);


    cv::destroyWindow(WINDOW1);

        pub = it.advertise("camera/vert_line", 1);
    
        ros::spin();
   
    ROS_INFO("ros_opencv::main.cpp::No error.");
 
}

