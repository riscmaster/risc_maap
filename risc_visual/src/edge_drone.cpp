
/*======================================================
	Created by:  	D. Spencer Maughan
	Last updated: 	May 2014
	File name: 		edge_drone.cpp
	Organization:	RISC Lab, Utah State University
 ======================================================*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;
 
//Setup windows
static const char WINDOW1[] = "Edges";

//Declare trackbar variables

int blursize = 2;
int lowThreshold = 800;
//Global Variables
//Mat image_old;

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

int const max_lowThreshold = 10000;
int ratio = 3;
cvtColor( cv_ptr->image, cv_ptr->image, CV_BGR2GRAY );
int kernel_size = 7;
createTrackbar("blur size", WINDOW1, &blursize,10 );
blur( cv_ptr->image, cv_ptr->image, Size(blursize*2+1,blursize*2+1) );
createTrackbar("thrshold", WINDOW1, &lowThreshold, max_lowThreshold);

  /// Canny detector

Canny( cv_ptr->image, cv_ptr->image, lowThreshold, lowThreshold*ratio, kernel_size );

//show image

 cv::imshow (WINDOW1,cv_ptr->image);
cv::waitKey(3);

cvtColor( cv_ptr->image, cv_ptr->image, CV_GRAY2BGR );
        pub.publish(cv_ptr->toImageMsg());
//image_old = cv_ptr->image;
}
 
int main(int argc, char **argv)

{
        ros::init(argc, argv, "edge_finder");
 
        ros::NodeHandle nh;

        image_transport::ImageTransport it(nh);

    cv::namedWindow(WINDOW1, CV_WINDOW_AUTOSIZE);

        image_transport::Subscriber sub = it.subscribe("ardrone/image_raw", 1, imageCallback);

    cv::destroyWindow(WINDOW1);

        pub = it.advertise("camera/image_edge", 1);
    
        ros::spin();
   
    ROS_INFO("ros_opencv::main.cpp::No error.");
 
}

