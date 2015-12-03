/*======================================================
	Created by:  	D. Spencer Maughan
	Last updated: 	July 2014
	File name: 	Transform_tuner.cpp
	Organization:	RISC Lab, Utah State University
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

	/*========================
		Variables
	  ========================*/
static const char WINDOW1[] = "transforms";
cv::Mat image;
int azim_bias = 1000;
int elev_bias = 1000;
int roll_bias = 1000;
int trans_bias = 1000;
int max = 2000;


void Datahandler(risc_msgs::Cortex p)
{
cv::createTrackbar("azimuth 1000=0", WINDOW1, &azim_bias, max);
cv::createTrackbar("elevation 1000=0", WINDOW1, &elev_bias, max);
cv::createTrackbar("roll 1000=0", WINDOW1, &roll_bias, max);
cv::createTrackbar("translation 1000=0", WINDOW1, &trans_bias, max);
int bodies = p.Obj.size();
for(int i; i<bodies; i++)
{
	/*======================
            Create Transforms
	  ======================*/

if(p.Obj[i].visible)
{
/*if(p.Obj[i].name == "baby_bird")
{ azim_bias = 1143; elev_bias = 938; roll_bias = 1216;}
if(p.Obj[i].name == "fluffy")
{ azim_bias = 595; elev_bias = 850; roll_bias = 1347;}*/
// Make body frame
     static tf::TransformBroadcaster br;
     tf::Transform body;
     body.setOrigin( tf::Vector3(p.Obj[i].x, p.Obj[i].y, p.Obj[i].z));
     tf::Quaternion u;
     u.setRPY(PI+p.Obj[i].phi*PI/180, p.Obj[i].theta*PI/180,PI/2-p.Obj[i].psi*PI/180);
     body.setRotation(u);
     br.sendTransform(tf::StampedTransform(body, ros::Time::now(), "/cortex", p.Obj[i].name));
   // Make camera frame 
     static tf::TransformBroadcaster cr;
    tf::Transform angle; 
    angle.setOrigin( tf::Vector3(.21+.01*(trans_bias-1000),0, 0) );//measured distance from  camera to center of quad 
     tf::Quaternion a;
    a.setRPY(.01*(roll_bias-1000)*PI/180-PI, .01*(elev_bias-1000)*PI/180,PI/2-((6.09093311+(.01*(azim_bias-1000)))*PI/180));//This angle was found for Mothership using markers and measurig tape 16 july by Spencer Maughan 
    angle.setRotation(a); 
    cr.sendTransform(tf::StampedTransform(angle, ros::Time::now(), p.Obj[i].name, p.Obj[i].name+"/camera")); 
}
}
cv::imshow (WINDOW1, image);
cv::waitKey(3);

}



int main(int argc, char **argv)
{
	ros::init(argc,argv, "transforms");
	ros::NodeHandle n;
	std::string path = ros::package::getPath("quad_command");
        image = cv::imread(path + "/mario.jpg");
        cv::Size size(321,123);
        cv::resize(image,image,size);
        cv::namedWindow(WINDOW1, CV_WINDOW_AUTOSIZE);

	
	/*========================
		Subscribe
	  ========================*/

	ros::Subscriber sub = n.subscribe("/cortex",1000,Datahandler);
	ros::spin();
}

