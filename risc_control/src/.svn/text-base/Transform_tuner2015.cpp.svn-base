/*======================================================
	Created by:  	D. Spencer Maughan
	Last updated: 	March 2015
	File name: 	    Transform_tuner2015.cpp
	Organization:	RISC Lab, Utah State University
    Notes:
         This file is intended for use on the RISC MAAP System
      to manually tune the transformation from body frame to 
      camera frame on a rigid camera connection to a quadcopter
      this is in tandem with the launch file entitled 
      "risc_camera_transforms.launch".
 ======================================================*/

#include "ros/ros.h"
#include <risc_msgs/Cortex.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <ros/package.h>
#define PI 3.141592653589793

	/*=============
		Variables
	  =============*/
static const char WINDOW1[] = "transforms";
cv::Mat image;
int azim_bias = 493;
int elev_bias = 1580;
int roll_bias = 1260;
int trans_bias = 1504;
int max = 3000;

void Datahandler(risc_msgs::Cortex p)
{
    cv::createTrackbar("azimuth 1500=0", WINDOW1, &azim_bias, max);
    cv::createTrackbar("elevation 1500=0", WINDOW1, &elev_bias, max);
    cv::createTrackbar("roll 1500=0", WINDOW1, &roll_bias, max);
    cv::createTrackbar("translation 1500=0", WINDOW1, &trans_bias, max);
int bodies = p.Obj.size();
for(int i; i<bodies; i++)
{
	/*======================
        Create Transforms
	  ======================*/

if(p.Obj[i].visible && p.Obj[i].name != "Pendulum_tip")
{

	/*===========================================
        Broadcasters, Transforms and Rotations
     ===========================================*/

     static tf::TransformBroadcaster vbr,v1br,v2br,bodybr;
     tf::Transform body,v,v1,v2;
     tf::Quaternion bodyq,vq,v1q,v2q;

	/*===============
        Set Postion
	  ===============*/

     body.setOrigin( tf::Vector3(p.Obj[i].x, p.Obj[i].y, p.Obj[i].z));
     v.setOrigin( tf::Vector3(p.Obj[i].x, p.Obj[i].y, p.Obj[i].z));
     v1.setOrigin( tf::Vector3(p.Obj[i].x, p.Obj[i].y, p.Obj[i].z));
     v2.setOrigin( tf::Vector3(p.Obj[i].x, p.Obj[i].y, p.Obj[i].z));


	/*=======================
        Set Frame Rotations
	  =======================*/

     // Vehicle
     vq.setRPY(PI,0,0);
     v.setRotation(vq);
     // Vehicle1
     v1q.setRPY(PI, 0,-p.Obj[i].psi*PI/180);
     v1.setRotation(v1q);
     // Vehicle2
     v2q.setRPY(PI, -p.Obj[i].theta*PI/180,-p.Obj[i].psi*PI/180);
     v2.setRotation(v2q);
     // Body
     bodyq.setRPY(PI+p.Obj[i].phi*PI/180, -p.Obj[i].theta*PI/180,-p.Obj[i].psi*PI/180);
     body.setRotation(bodyq);

	/*===================
        Send Transforms
	  ===================*/

     // Send Transform
     vbr.sendTransform(tf::StampedTransform(v, ros::Time::now(), "/cortex", p.Obj[i].name+"_vehicle_frame"));
     v1br.sendTransform(tf::StampedTransform(v1, ros::Time::now(), "/cortex", p.Obj[i].name+"_vehicle1_frame"));
     v2br.sendTransform(tf::StampedTransform(v2, ros::Time::now(), "/cortex", p.Obj[i].name+"_vehicle2_frame"));
     bodybr.sendTransform(tf::StampedTransform(body, ros::Time::now(), "/cortex", p.Obj[i].name+"_body_frame"));

	/*=====================
        Set Camera Offest
	  =====================*/

   // Make camera frame 
     static tf::TransformBroadcaster cr;
    tf::Transform angle; 
   //measured distance from  camera to center of quad 
    angle.setOrigin( tf::Vector3(.21+.001*(trans_bias-1500), 0, 0) );
    tf::Quaternion a;
    a.setRPY(.01*(roll_bias-1500)*PI/180, .01*(elev_bias-1500)*PI/180,((6.09093311+(.01*(azim_bias-1500)))*PI/180));
    angle.setRotation(a); 
    cr.sendTransform(tf::StampedTransform(angle, ros::Time::now(), p.Obj[i].name+"_body_frame", p.Obj[i].name+"/camera")); 
}
}
cv::imshow (WINDOW1, image);
cv::waitKey(3);
}



int main(int argc, char **argv)
{
	ros::init(argc,argv, "transforms");
	ros::NodeHandle n;
    std::string path = ros::package::getPath("risc_control");
    image = cv::imread(path + "/mario.jpg");
    cv::Size size(321,123);
    cv::resize(image,image,size);
    cv::namedWindow(WINDOW1, CV_WINDOW_AUTOSIZE);


	
	/*========================
		Subscribe
	  ========================*/

	ros::Subscriber sub = n.subscribe("/cortex_raw",1000,Datahandler);
	ros::spin();
}

