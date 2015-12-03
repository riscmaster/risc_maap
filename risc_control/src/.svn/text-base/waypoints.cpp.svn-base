/*======================================================
	Created by:  	D. Spencer Maughan
	Last updated: 	May 2014
	File name: 		waypoints.cpp
	Organization:	RISC Lab, Utah State University
 ======================================================*/

#include "ros/ros.h"
#include "cortex_ros/Cortex.h"
#include "risc_msgs/Waypoints.h"
#include "risc_msgs/Controls.h"
#include "geometry_msgs/Point.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <math.h>
#define PI 3.141592653589793

	/*============================
				Globals
	  ============================*/
	 
ros::Publisher p;
risc_msgs::Waypoints wp;
cortex_ros::Cortex states;
static const char WINDOW1[] = "gains";
int Y_kp,Y_ki,Y_kd,A_kp,A_ki,A_kd = 1;
int R_kp = 447;
int R_ki = 250;
int P_kp = 588;
int P_ki = 350;
int Threshold = 1000;
int P_kd = 875;
int R_kd = 625;
cv::Mat image;
float max_pitch = 10*PI/180;
float max_roll = 10*PI/180;

	/*========================
		 Main PID Controller
	  ========================*/
 
float PID(float err, float p)
{
float kp = 3;
float ki = 0.03;
float kd = 0.3;
float u = (kp*err-kd*p+ki*err*.0167)/180;
return u;
}


/*
 The following controller gains are taken from an article titled: 
"Navigation and Localization of a Quad-rotor by Landmarks in a GPS-Denied Known Indoor Environment"
First Author: Anh, Young
*/


	/*========================
		 Yaw PID Controller
	  ========================*/

float PID_Yaw(float err, float p)
{

float kp = 3;
float ki = 0.03;
float kd = 0.3;
float u=(kp*err-kd*p+ki*err*.0167)/180;
return u;
}

	/*=========================
		 Pitch PI Controller
	  =========================*/

float PID_Pitch(float err,float p)
{
float kp = .001*P_kp;//.25;
float ki = .01*P_ki;//0.2;
float kd = .001*P_kd;//0.0
float u=(kp*err-kd*p+ki*err*.0167);
return u;
}

	/*========================
		Roll PI Controller
	  ========================*/

float PID_Roll(float err,float p)
{
float kp = .001*R_kp;//0.4; ZN .447
float ki = .01*R_ki;//0.18; ZN 2.5
float kd = .001*R_kd; //0.0; ZN .625
float u=(kp*err-kd*p+ki*err*.0167);
return u;
}

	/*===========================
		Altitude PID Controller
	  ===========================*/

float PID_Alt(float err, float p)
{
float kp = 1.5;
float kd = 0.025;
float ki = 0.1;
float u=kp*err-kd*p+ki*err*.0167;
return u;
}

	/*=================
		Get Waypoints
	  =================*/
	 
void waypoint(risc_msgs::Waypoints W)
{
wp = W;
}

	/*========================
		  Get Cortex States
	  ========================*/
 
void GetStates(cortex_ros::Cortex S)
{
    states = S;

}

	/*====================================================
		Basic Controller (does not use literature gains)
	  ====================================================*/

void Datahandler()
{
if (wp.Obj.size() == 0)
{
wp.Obj.resize(1);
}
if (states.Obj.size() == 0)
{
states.Obj.resize(1);
}
int bodies = states.Obj.size();
risc_msgs::Controls Ctrl;
Ctrl.Obj.resize(bodies);
ROS_INFO("x = %f",::states.Obj[0].x);
for (int j = 0; j < bodies; ++j)
    {
float Px,Py,Pz,Pxymag;
float psi = states.Obj[j].psi;
Px = wp.Obj[j].x-states.Obj[j].x;
Py = wp.Obj[j].y-states.Obj[j].y;
Pz = wp.Obj[j].z-states.Obj[j].z;

float r = Px*cos(psi*PI/180)-Py*sin(psi*PI/180); 
float p = Px*sin(psi*PI/180)+Py*cos(psi*PI/180);
float u = states.Obj[j].u*cos(psi*PI/180)-states.Obj[j].v*sin(psi*PI/180);
float v = states.Obj[j].u*sin(psi*PI/180)+states.Obj[j].v*cos(psi*PI/180);
float ay = PID_Roll(r,u);
float ax = PID_Pitch(p,v);
if (Vxymag < 4)
	{
        Ctrl.Obj[j].name = states.Obj[j].name;
		Ctrl.Obj[j].theta = ax/9.81; 
		Ctrl.Obj[j].phi = -ay/9.81;
	//	Ctrl.Obj[j].psi = 0;//PID_Yaw(states.Obj[j].psi,states.Obj[j].r);
	//	Ctrl.Obj[j].alt = 0;//PID_Alt(Vz, states.Obj[j].r); //altitude command
	}
if (Vxymag > 4)
	{
		ROS_WARN("Cortex lost object");
		Ctrl.Obj[j].name = states.Obj[j].name;
		Ctrl.Obj[j].phi = 0; // commanded roll angle
		Ctrl.Obj[j].theta = 0;// Commanded  pitch angle 
	//	Ctrl.Obj[j].psi = 0; //commanded yaw
	//	Ctrl.Obj[j].alt = 0; //altitude; //altitude command

	}

}
cv::createTrackbar("R_kp", WINDOW1, &R_kp, Threshold);
cv::createTrackbar("R_ki", WINDOW1, &R_ki, Threshold);
cv::createTrackbar("R_kd", WINDOW1, &R_kd, Threshold);
cv::createTrackbar("P_kp", WINDOW1, &P_kp, Threshold);
cv::createTrackbar("P_ki", WINDOW1, &P_ki, Threshold);
cv::createTrackbar("P_kd", WINDOW1, &P_kd, Threshold);

cv::imshow (WINDOW1, image);
cv::waitKey(3);
p.publish(Ctrl);

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  image = cv::imread("/home/ece/ros_ws/src/quad_command/mario.jpg");
  cv::Size size(321,123);
  cv::resize(image,image,size);
  cv::namedWindow(WINDOW1, CV_WINDOW_AUTOSIZE);
	  
	/*================================
	   Set up Subscriber and Publisher
	  =================================*/
    
    p = n.advertise<risc_msgs::Controls>("controls", 1);
    ros::Subscriber blub = n.subscribe("/waypoints" , 1, waypoint);
    ros::Subscriber sub = n.subscribe("/cortex" , 1000, GetStates);
    Datahandler();
    cv::destroyWindow(WINDOW1);
    ros::spin();
  return 0;
}

