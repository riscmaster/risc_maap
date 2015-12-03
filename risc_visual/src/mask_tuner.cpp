
/*======================================================
	Created by:  	D. Spencer Maughan
	Last updated: 	July 2014
	File name: 		mask_tuner.cpp
	Organization:	RISC Lab, Utah State University
 ======================================================*/

#include <ros/ros.h>
#include "sensor_msgs/RegionOfInterest.h"
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
#include <vector>
#include <string>
#include <algorithm>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>
#define PI 3.14159265

using namespace std;
namespace enc = sensor_msgs::image_encodings;
 
//Setup windows
static const char WINDOW1[] = "Blobs";

	/*========================
		Trackbar Variables
	 =======================*/
 
int colorhigh = 180;
int colorlow = 0;
int saturation_high = 255;
int saturation_low = 0;
int value_high = 255;
int value_low = 0;
int blur = 2;
int erosion_size = 1;

	/*========================
		Global Variables
	 =======================*/

cv::Mat image;
cv::RNG rng(12345);
image_transport::Publisher pub;

	/*======================================
     *     Allow combination of Masks
     =====================================*/

cv::Mat combineMasks(cv::Mat mask1, cv::Mat mask2)
{
    cv::Mat new_Mask(cv::Size(mask1.cols,mask1.rows),CV_8U);
    for (int i=0; i<mask1.rows; i++)
    {
        for(int j=0; j<mask1.cols; j++)
        {
            new_Mask.at<uchar>(i,j) = mask1.at<uchar>(i,j)|mask2.at<uchar>(i,j);
        }
    }
    return new_Mask;
}

	/*======================================
     *     Replace Max_Element Function
     =====================================*/

float Max(float Val[3])
{
    float V;
    if ((Val[0] > Val[1]) && (Val[0] > Val[2])){ V = Val[0];} 
    if ((Val[0] > Val[1]) && (Val[0] == Val[2])){ V = Val[0];} 
    if ((Val[0] == Val[1]) && (Val[0] > Val[2])){ V = Val[0];} 
    if ((Val[1] > Val[0]) && (Val[1] > Val[2])){ V = Val[1];} 
    if ((Val[1] > Val[0]) && (Val[1] == Val[2])){ V = Val[1];} 
    if ((Val[1] == Val[0]) && (Val[1] > Val[2])){ V = Val[1];} 
    if ((Val[2] > Val[0]) && (Val[2] > Val[1])){ V = Val[2];} 
    if ((Val[2] > Val[0]) && (Val[2] == Val[1])){ V = Val[2];} 
    if ((Val[2] == Val[0]) && (Val[2] > Val[1])){ V = Val[2];} 
    return V;
}

    /*======================================
     *     Replace Min_Element Function
     =====================================*/
     
float Min(float Val[3])
{
    float V;
    if ((Val[0] < Val[1]) && (Val[0] < Val[2])){ V = Val[0];} 
    if ((Val[0] < Val[1]) && (Val[0] == Val[2])){ V = Val[0];} 
    if ((Val[0] == Val[1]) && (Val[0] < Val[2])){ V = Val[0];} 
    if ((Val[1] < Val[0]) && (Val[1] < Val[2])){ V = Val[1];} 
    if ((Val[1] < Val[0]) && (Val[1] == Val[2])){ V = Val[1];} 
    if ((Val[1] == Val[0]) && (Val[1] < Val[2])){ V = Val[1];} 
    if ((Val[2] < Val[0]) && (Val[2] < Val[1])){ V = Val[2];} 
    if ((Val[2] < Val[0]) && (Val[2] == Val[1])){ V = Val[2];} 
    if ((Val[2] == Val[0]) && (Val[2] < Val[1])){ V = Val[2];} 
    return V;
}

    /*======================================
     *   Replace Opecv cvtColor Function
     =====================================*/
     
cv::Mat BGR2HSV(cv::Mat BGR)
{
    cv::Mat HSV(cv::Size(BGR.cols,BGR.rows),CV_8UC3);

    for(int i=0; i<BGR.rows; i++)
    {
            for(int j=0; j<BGR.cols; j++) 
            {
                float Val[3];
                float H,S,V;

                Val[0]= (BGR.at<cv::Vec3b>(i,j)[0]);
                Val[1]= (BGR.at<cv::Vec3b>(i,j)[1]);
                Val[2]= (BGR.at<cv::Vec3b>(i,j)[2]);

                Val[0]= Val[0]/255;
                Val[1]= Val[1]/255;
                Val[2]= Val[2]/255;


                // Value

                V = Max(Val);
                float delta = V - Min(Val);
          
                // Saturation
                if (V != 0)
                {
                S = delta/V;
                
                //Hue
                if (V == Val[0])
                {
                    H = 240 + (60*(Val[2]-Val[1]))/delta;
                }
                
                if (V == Val[1])
                {
                    H = 120 + (60*(Val[0]-Val[2]))/delta;
                }
                
                if (V == Val[2])
                { 
                    H = (60*(Val[1]-Val[0]))/delta;
                }
                
                if (H<0){H = H + 360;}}
                
                if (V == 0){V = 0;} 
                
                if (V > 1){V = 1;}

                HSV.at<cv::Vec3b>(i,j)[0] = H/2;
                HSV.at<cv::Vec3b>(i,j)[1] = 255*S;
                HSV.at<cv::Vec3b>(i,j)[2] = V*255;
            }
    }
    return HSV;
}


	/*=========================================
		Function Called for Every New Image
	 ========================================*/

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
int h = cv_ptr->image.rows;
int w = cv_ptr->image.cols;

	/*======================
		Create Trackbars
	 =====================*/

cv::createTrackbar("colorhigh", WINDOW1, &colorhigh, 180); 
cv::createTrackbar("colorlow", WINDOW1, &colorlow, 180); 
cv::createTrackbar("saturation_high", WINDOW1, &saturation_high, 255); 
cv::createTrackbar("satruation_low", WINDOW1, &saturation_low, 255); 
cv::createTrackbar("value_high", WINDOW1, &value_high, 255); 
cv::createTrackbar("value_low", WINDOW1, &value_low, 255); 
cv::createTrackbar("erosion_size", WINDOW1, &erosion_size, 20); 
cv::createTrackbar("blur", WINDOW1, &blur, 100); 

	/*================
		Create Mask
	 ================*/

cv::Mat threshy(cv::Size(h,w),CV_8U);
cv::Mat hsvframe = BGR2HSV(cv_ptr->image); 
cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(2*erosion_size +1, 2*erosion_size + 1), cv::Point(erosion_size, erosion_size));
if (colorhigh >= colorlow)
    {
    cv::inRange(hsvframe,cv::Scalar(colorlow,saturation_low,value_low),cv::Scalar(colorhigh,saturation_high,value_high),threshy);
    cv::dilate(threshy,threshy,element);
    cv::erode(threshy,threshy,element);
    cv::GaussianBlur(threshy,threshy,cv::Size(2*blur+1,2*blur+1),2,2);
    }
if (colorhigh < colorlow)
    {
    cv::Mat threshy1(cv::Size(h,w),CV_8U);
    cv::Mat threshy2(cv::Size(h,w),CV_8U);
    cv::inRange(hsvframe,cv::Scalar(colorlow,saturation_low,value_low),cv::Scalar(180,saturation_high,value_high),threshy1);
    cv::inRange(hsvframe,cv::Scalar(0,saturation_low,value_low),cv::Scalar(colorhigh,saturation_high,value_high),threshy2);
    threshy = combineMasks(threshy1,threshy2);
    cv::dilate(threshy,threshy,element);
    cv::erode(threshy,threshy,element);
    cv::GaussianBlur(threshy,threshy,cv::Size(2*blur+1,2*blur+1),2,2);
    }
	/*====================================
		  Show Mask and Color Wheel
	 ====================================*/

cv::imshow(WINDOW1,threshy);                                    
cv::waitKey(3);                                                   
cv::imshow("colorwheel",image);                                    
cv::waitKey(3);                                                   
}                                                                                           
                                                                                            
int main(int argc, char **argv)                                                             
                                                                                            
{
        ros::init(argc, argv, "blob_finder");
 
        ros::NodeHandle nh;

        image = cv::imread("/home/ece/ros_ws/src/risc_visual/ColorWheel.jpg");
        cv::namedWindow("colorwheel", CV_WINDOW_AUTOSIZE);
                                                                                            
        image_transport::ImageTransport it(nh);
                                                                                            
        cv::namedWindow(WINDOW1, CV_WINDOW_AUTOSIZE);
                                                                                            
        image_transport::Subscriber sub = it.subscribe("ardrone/image_raw", 1, imageCallback);
        cv::destroyAllWindows();                          
         
        pub = it.advertise("/target", 1);
     
        ros::spin();
    
        ROS_INFO("ros_opencv::main.cpp::No error.");
  
 }
