
/*======================================================
	Created by:  	D. Spencer Maughan
	Last updated: 	August 2014
	File name: 	detect_objects.cpp
	Organization:	RISC Lab, Utah State University
 ======================================================*/

#include <ros/ros.h>
//  Messages
#include "risc_msgs/Observed_rois.h"
#include "risc_msgs/Cortex.h"
#include "risc_msgs/Landmarks.h"
#include "risc_msgs/Risc_rois.h"
#include "risc_msgs/Risc_roi.h"
#include "ardrone_autonomy/Navdata.h"
// For images and processing
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv.h>
// For Transforms
#include <tf/transform_listener.h>
// Useful math/analysis functions
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <string>
#include <algorithm>
#define PI 3.14159265

using namespace std;
namespace enc = sensor_msgs::image_encodings;

    /*=================
     *    Globals
     ================*/

risc_msgs::Cortex states;
risc_msgs::Landmarks landmarks;
risc_msgs::Observed_rois R;
ros::Publisher pub; 
image_transport::Publisher im1;
image_transport::Publisher im2;
tf::TransformListener* listener = NULL;

    /*=============================
     *    Return Pixel Locations
     ============================*/

risc_msgs::Risc_roi Point2Pix(geometry_msgs::PointStamped P)
{
                    float range_azim = 80.7492398;
                    float range_elev = 36.8064474;
                    float azim = atan2(P.point.x,P.point.y)*180/PI;
                    float elev = atan2(P.point.z,sqrt(P.point.y*P.point.y+P.point.x*P.point.x))*180/PI;
                    int x=0;
                    int y=0;
                    if(abs(azim)<=range_azim/2 && abs(elev)<=range_elev/2)
                   {
                    x = cvRound(azim/range_azim*640+320);
                    y = cvRound(-elev/range_elev*360+180);
                   }
                   risc_msgs::Risc_roi roi;
                   roi.x = x;
                   roi.y = y;
                   roi.visible = true;
                    if(abs(azim)>range_azim/2 or abs(elev)>=range_elev/2)
                   { roi.visible = false;}

                    return roi;
}

    /*===================
     * Get Quad States 
     ==================*/

void States(risc_msgs::Cortex x){states = x;}

    /*===================
     * Get Landmarks 
     ==================*/

void Landmarks(risc_msgs::Landmarks x){landmarks = x;}


    /*====================================================
     *  Draw Crosshairs to designate center and angles
     ===================================================*/

cv::Mat drawCross(cv::Mat image)
{
	int h = image.rows;
	int w = image.cols;
    cv::line(image,cv::Point(0,h/2),cv::Point(floor(.45*w),h/2),cv::Scalar(0,0,0),3,8); 
    cv::line(image,cv::Point(floor(.55*w),h/2),cv::Point(w,h/2),cv::Scalar(0,0,0),3,8); 
    cv::line(image,cv::Point(w/2,0),cv::Point(w/2,floor(.4*h)),cv::Scalar(0,0,0),3,8); 
    cv::line(image,cv::Point(w/2,floor(.6*h)),cv::Point(w/2,h),cv::Scalar(0,0,0),3,8); 
    int thick[12] = {1,1,2,1,1,1,2,1,1,1,1,2};
    float line[12] = {16/2,16/3,16/4,16/5,16/6,16/7,16/8,16/9,16/10,16/11,16/12,16/14};
    int angle1[4] = {-10,80,170,260};
    int angle2[4] = {10,100,190,280};
    for (int i=0; i<4; i++)
    {
        for ( int j=0; j<12; j++)
        {

    cv::ellipse(image,cv::Point(w/2,h/2),cv::Size(floor(h/line[j]),floor(h/line[j])),0,angle1[i],angle2[i],cv::Scalar(0,0,0),thick[j],8,0);
        }}
    cv::ellipse(image,cv::Point(w/2,h/2),cv::Size(2,2),0,0,360,cv::Scalar(0,0,255),-1,8,0);
    return image;
}

    /*=========================================
     *  Function Called for quad1 Image
     ========================================*/

void imageCallbackquad1(const sensor_msgs::ImageConstPtr& original_image)
{
if(states.Obj.size()!=0 && landmarks.Obj.size() !=0)
{
//ROS_INFO("quad1");
     ros::Time time = ros::Time::now();

    /*===================
     *    CV Bridge
     ==================*/

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
	cv::Scalar pink_color = cv::Scalar( 0, 0, 100) ;
	cv::Scalar orange_color = cv::Scalar( 0, 153, 255) ;
	cv::Scalar blue_color = cv::Scalar( 200, 0, 100) ;
	cv::Scalar green_color = cv::Scalar( 0, 204, 204) ;
 /*===================
  *    Create ROI
  ==================*/

//ROS_INFO("quad1--1");
 risc_msgs::Risc_rois roi;
 int bodies = states.Obj.size();
 roi.quads.resize(bodies-1);

for (int j = 0; j<bodies-1; j++)
{
           //=============================================#
           //    Get Cortex Frame Positions of Regions    #
           //=============================================#

//ROS_INFO("quad1--2");
                geometry_msgs::PointStamped Q;
                Q.point.x =0; 
                Q.point.y =0;
                Q.point.z =0;
//ROS_INFO("quad1--2.5");
                Q.header.frame_id = states.Obj[1].name;

 risc_msgs::Risc_roi roiq;
//ROS_INFO("quad1--3");
try {
 listener->waitForTransform( states.Obj[1].name, states.Obj[0].name+"/camera", ros::Time(0), ros::Duration(3.0) );
 listener->transformPoint(states.Obj[0].name+"/camera", Q,Q);
 roiq = Point2Pix(Q);
//ROS_INFO("quad1--3.5");
 roiq.name = states.Obj[1].name;

} catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
}
//ROS_INFO("quad1--3.75");
 roi.quads[j] = roiq;
//ROS_INFO("quad1--3.85");
cv::circle(cv_ptr->image, cv::Point(  roiq.x  ,roiq.y), 4, cv::Scalar(0,255,255) , 2, 8, 0);
}




           //==========================#
           //    Make landmark ROIs    #
           //==========================#
//ROS_INFO("quad1--4");
     int len = landmarks.Obj.size();
     roi.landmarks.resize(len);;

for(int i = 0; i<len; i++)
{
 risc_msgs::Risc_roi roil;

 geometry_msgs::PointStamped P;
 P.point.x =landmarks.Obj[i].x; 
 P.point.y =landmarks.Obj[i].y;
 P.point.z =landmarks.Obj[i].z;
 P.header.frame_id = "/cortex";


try {
//ROS_INFO("quad1--5");
 listener->waitForTransform( "/cortex", states.Obj[0].name+"/camera", ros::Time(0), ros::Duration(3.0) );
 listener->transformPoint(states.Obj[0].name+"/camera", P,P);
 roil = Point2Pix(P);
 roil.name = landmarks.Obj[i].name;

} catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
}

roi.landmarks[i] = roil;
cv::circle(cv_ptr->image, cv::Point(  roil.x  ,roil.y), 4, cv::Scalar(255,255,255) , 2, 8, 0);
}

/*===================
 *    Publish
 ==================*/

//ROS_INFO("quad1--6");
R.Obj[0] = roi;
R.Obj[0].name = states.Obj[0].name;
R.header.stamp = time;
pub.publish(R);                      
    cv_ptr->image = drawCross(cv_ptr->image);
    im1.publish(cv_ptr->toImageMsg());
}}                                                                                           
    /*=========================================
     *  Function Called for quad2 Image
     ========================================*/

void imageCallbackquad2(const sensor_msgs::ImageConstPtr& original_image)
{
if(states.Obj.size()!=0 && landmarks.Obj.size() !=0)
{
//ROS_INFO("quad2");
     ros::Time time = ros::Time::now();

    /*===================
     *    CV Bridge
     ==================*/

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
	cv::Scalar pink_color = cv::Scalar( 0, 0, 100) ;
	cv::Scalar orange_color = cv::Scalar( 0, 153, 255) ;
	cv::Scalar blue_color = cv::Scalar( 200, 0, 100) ;
	cv::Scalar green_color = cv::Scalar( 0, 204, 204) ;
 /*===================
  *    Create ROI
  ==================*/

//ROS_INFO("quad2--1");
 risc_msgs::Risc_rois roi;
 int bodies = states.Obj.size();
 roi.quads.resize(bodies-1);

for (int j = 0; j<bodies-1; j++)
{
           //=============================================#
           //    Get Cortex Frame Positions of Regions    #
           //=============================================#

//ROS_INFO("quad2--2");
                geometry_msgs::PointStamped Q;
                Q.point.x =0; 
                Q.point.y =0;
                Q.point.z =0;
//ROS_INFO("quad2--2.5");
                Q.header.frame_id = states.Obj[0].name;

 risc_msgs::Risc_roi roiq;
//ROS_INFO("quad2--3");
try {
 listener->waitForTransform( states.Obj[0].name, states.Obj[1].name+"/camera", ros::Time(0), ros::Duration(3.0) );
 listener->transformPoint(states.Obj[1].name+"/camera", Q,Q);
 roiq = Point2Pix(Q);
//ROS_INFO("quad2--3.5");
 roiq.name = landmarks.Obj[0].name;

} catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
}
//ROS_INFO("quad2--3.75");
 roi.quads[j] = roiq;
//ROS_INFO("quad2--3.85");
cv::circle(cv_ptr->image, cv::Point(  roiq.x  ,roiq.y), 4, cv::Scalar(255,255,255) , 2, 8, 0);
}




           //==========================#
           //    Make landmark ROIs    #
           //==========================#
//ROS_INFO("quad2--4");
     int len = landmarks.Obj.size();
     roi.landmarks.resize(len);;

for(int i = 0; i<len; i++)
{
 risc_msgs::Risc_roi roil;

 geometry_msgs::PointStamped P;
 P.point.x =landmarks.Obj[i].x; 
 P.point.y =landmarks.Obj[i].y;
 P.point.z =landmarks.Obj[i].z;
 P.header.frame_id = "/cortex";


try {
//ROS_INFO("quad2--5");
 listener->waitForTransform( "/cortex", states.Obj[1].name+"/camera", ros::Time(0), ros::Duration(3.0) );
 listener->transformPoint(states.Obj[1].name+"/camera", P,P);
 roil = Point2Pix(P);
 roil.name = landmarks.Obj[i].name;

} catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
}

roi.landmarks[i] = roil;
cv::circle(cv_ptr->image, cv::Point(  roil.x  ,roil.y), 4, cv::Scalar(255,255,255) , 2, 8, 0);
}

/*===================
 *    Publish
 ==================*/

//ROS_INFO("quad2--6");
R.Obj[1] = roi;
R.Obj[1].name = states.Obj[1].name;
R.header.stamp = time;
pub.publish(R);                      
    cv_ptr->image = drawCross(cv_ptr->image);
    im2.publish(cv_ptr->toImageMsg());
}}
int main(int argc, char **argv)                                                             
                                                                                            
{
        ros::init(argc, argv, "blob_finder");
        ros::NodeHandle nh;
        pub = nh.advertise<risc_msgs::Observed_rois>("rois", 1000);
        listener = new (tf::TransformListener);
        R.Obj.resize(2);

        image_transport::ImageTransport it(nh);

        ros::Subscriber stub = nh.subscribe("/cortex",1,States);  
        ros::Subscriber snub = nh.subscribe("/landmarks",1,Landmarks);  

        image_transport::Subscriber sub = it.subscribe("quad1/ardrone/image_rect_color", 1, imageCallbackquad1);
        image_transport::Subscriber slub = it.subscribe("quad2/ardrone/image_rect_color", 1, imageCallbackquad2);
        im1 = it.advertise("/quad1/ardrone/image_modi",1);
        im2 = it.advertise("/quad2/ardrone/image_modi",1);
        
        cv::destroyAllWindows();                          
     
        ros::spin();
    
        ROS_INFO("ros_opencv::main.cpp::No error.");
  
 }
