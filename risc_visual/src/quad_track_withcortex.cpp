
/*======================================================
	Created by:  	D. Spencer Maughan
	Last updated: 	July 2014
	File name: 	quad_track_withcortex.cpp
	Organization:	RISC Lab, Utah State University
 ======================================================*/

#include <ros/ros.h>
#include "risc_msgs/Observed_rois.h"
#include "risc_msgs/Cortex.h"
#include "risc_msgs/Risc_rois.h"
#include "risc_msgs/Risc_roi.h"
#include "ardrone_autonomy/Navdata.h"
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>
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

    /*=================
     *    Globals
     ================*/

ros::Publisher pub; 
risc_msgs::Cortex states;
int size_limit;
static const char WINDOW1[] = "quads";
tf::TransformListener* listener = NULL;
int X = 807492;
int Y = 368064;
int MAX = 10000000;

 
    /*=============================
     *    Return Pixel Locations
     ============================*/
risc_msgs::Risc_roi Point2Pix(geometry_msgs::PointStamped P)
{
                    float range_azim = X*.0001;//80.7492398;
                    float range_elev = Y*.0001;//36.8064474;
                    float azim = atan2(P.point.x,P.point.y)*180/PI; 
                    float elev = -atan2(P.point.z,P.point.y)*180/PI;
                    float x=-1;
                    float y=-1;
                    if(abs(azim)<=range_azim/2 && abs(elev)<=range_elev/2)
                   {
                    x = cvRound(azim*2/range_azim*640+320);
                    y = cvRound(elev*2/range_elev*360+180);
                   }
                   risc_msgs::Risc_roi roi;
                   roi.x = x;
                   roi.y = y;
                   roi.visible = true;
                   if(roi.x <0 && roi.y < 0)
                   { roi.visible = false;} 

                    return roi;
}

    /*===================
     * Get Quad States 
     ==================*/

void States(risc_msgs::Cortex x){states = x;}

    /*=========================================
     *  Function Called for Every New Image
     ========================================*/

void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
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
int bodies = states.Obj.size();
cv::createTrackbar("X",WINDOW1,&X,MAX);
cv::createTrackbar("Y",WINDOW1,&Y,MAX);

risc_msgs::Observed_rois Obsrv;
Obsrv.header.stamp = ros::Time(0);
Obsrv.header.frame_id = "/cortex";
Obsrv.Obj.resize(bodies);
// loop through each quad
for (int i=0; i<bodies; i++)
{
Obsrv.Obj[i].name = states.Obj[i].name;
Obsrv.Obj[i].quads.resize(4*(bodies-1));
// get pixel location of other quad
for (int j=0; j<bodies; j++)
{
if(i!=j && states.Obj[j].visible)
{
           //=============================================#
           //    Get Cortex Frame Positions of Regions    #
           //=============================================#

                geometry_msgs::PointStamped I;
                I.point.x = .125;
                I.point.y = .125;
                I.point.z = 0;
                I.header.frame_id = states.Obj[j].name; 

                geometry_msgs::PointStamped II;
                II.point.x = -.125;
                II.point.y = .125;
                II.point.z = 0;
                II.header.frame_id = states.Obj[j].name; 

                geometry_msgs::PointStamped III;
                III.point.x = -.125;
                III.point.y = -.125;
                III.point.z = 0;
                III.header.frame_id = states.Obj[j].name; 

                geometry_msgs::PointStamped IV;
                IV.point.x = .125;
                IV.point.y = -.125;
                IV.point.z = 0;
                IV.header.frame_id = states.Obj[j].name; 
risc_msgs::Risc_roi I_roi;
risc_msgs::Risc_roi II_roi;
risc_msgs::Risc_roi III_roi;
risc_msgs::Risc_roi IV_roi;

ROS_INFO("1");
try {
           //=================================================#
           //    Convert to  Body Frame Positions  and Rois   #
           //=================================================#

                    listener->waitForTransform( states.Obj[j].name, states.Obj[i].name+"/camera", ros::Time(0), ros::Duration(3.0) );
                    listener->transformPoint(states.Obj[i].name+"/camera", I,I);
                    I_roi = Point2Pix(I);
                    I_roi.name = states.Obj[j].name+"_I";

                    listener->transformPoint(states.Obj[i].name+"/camera", II,II);
                    II_roi = Point2Pix(II);
                    II_roi.name = states.Obj[j].name+"_II";

                    listener->transformPoint(states.Obj[i].name+"/camera", III,III);
                    III_roi = Point2Pix(III);
                    III_roi.name = states.Obj[j].name+"_III";

                    listener->transformPoint(states.Obj[i].name+"/camera", IV,IV);
                    IV_roi = Point2Pix(IV);
                    IV_roi.name = states.Obj[j].name+"_IV";
ROS_INFO("2");

} catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
}

ROS_INFO("3");
           //=====================#
           //    Make roiArray    #
           //=====================#

//        Obsrv.Obj[i].quads[4*j].x   = I_roi.x;
//        Obsrv.Obj[i].quads[4*j].y   = I_roi.y;
//        Obsrv.Obj[i].quads[4*j].name   = I_roi.name;
//        Obsrv.Obj[i].quads[4*j+1] = II_roi;
 //       Obsrv.Obj[i].quads[4*j+2] = III_roi;
  //      Obsrv.Obj[i].quads[4*j+3] = IV_roi;
ROS_INFO("4");
if (i == 0)
{
        cv::circle(cv_ptr->image, cv::Point(  I_roi.x  ,I_roi.y), 4, cv::Scalar(0,0,255) , 2, 8, 0);
        cv::circle(cv_ptr->image, cv::Point( II_roi.x ,II_roi.y), 4, cv::Scalar(0,255,0) , 2, 8, 0);
        cv::circle(cv_ptr->image, cv::Point(III_roi.x,III_roi.y), 4, cv::Scalar(255,0,0) , 2, 8, 0);
        cv::circle(cv_ptr->image, cv::Point( IV_roi.x ,IV_roi.y), 4, cv::Scalar(255,255,255) , 2, 8, 0);
}
  }                                                                                            
}}
ROS_INFO("5");
pub.publish(Obsrv);
cv::imshow(WINDOW1,cv_ptr->image); 
cv::waitKey(3);                                                   
}

int main(int argc, char **argv)                                                             
                                                                                            
{
        ros::init(argc, argv, "quad_finder");
        ros::NodeHandle nh;
        pub = nh.advertise<risc_msgs::Observed_rois>("quad_rois", 1000);

        listener = new (tf::TransformListener);

        image_transport::ImageTransport it(nh);
                                                                                            
        cv::namedWindow(WINDOW1, CV_WINDOW_AUTOSIZE);
                                                                                            
        //image_transport::Subscriber sub = it.subscribe("ardrone/image_rect_color", 1, imageCallback);
        image_transport::Subscriber sub = it.subscribe("ardrone/image_rect_color", 1, imageCallback);
        ros::Subscriber slub = nh.subscribe("/cortex", 1, States);
        cv::destroyAllWindows();                          
        ros::spin();
    
        ROS_INFO("ros_opencv::main.cpp::No error.");
  
 }
