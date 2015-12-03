/*======================================================
	Created by:  	D. Spencer Maughan
	Last updated: 	Aug 2015
	File name: 	risc_transforms.cpp
	Organization:	RISC Lab, Utah State University
 ======================================================*/

#include "ros/ros.h"
#include <risc_msgs/Cortex.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#define PI 3.141592653589793


void Datahandler(risc_msgs::Cortex p)
{
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

     static tf::TransformBroadcaster vehicle,vehicle1,vehicle2,body_frame;
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
     vehicle.sendTransform(tf::StampedTransform(v, ros::Time::now(), "/cortex", p.Obj[i].name+"_vehicle_frame"));
     vehicle1.sendTransform(tf::StampedTransform(v1, ros::Time::now(), "/cortex", p.Obj[i].name+"_vehicle1_frame"));
     vehicle2.sendTransform(tf::StampedTransform(v2, ros::Time::now(), "/cortex", p.Obj[i].name+"_vehicle2_frame"));
     body_frame.sendTransform(tf::StampedTransform(body, ros::Time::now(), "/cortex", p.Obj[i].name+"_body_frame"));

/*=====================
   Set Camera Offest
  =====================*/

int azim_bias = 1500;
int elev_bias = 1500;
int roll_bias = 1500;
int trans_bias = 1500;
if (p.Obj[i].name == "albatross")
{
azim_bias = 493;
elev_bias = 1580;
roll_bias = 1266;
trans_bias = 1257;
}
if (p.Obj[i].name == "fluffy_II")
{
azim_bias = 375;
elev_bias = 2869;
roll_bias = 1462;
trans_bias = 1277;
}
if (p.Obj[i].name == "Raven")
{
azim_bias = 982;
elev_bias = 2006;
roll_bias = 1779;
trans_bias = 1257;
}
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
}



int main(int argc, char **argv)
{
	ros::init(argc,argv, "transforms");
	ros::NodeHandle n;
	
/*========================
        Subscribe
  ========================*/

	ros::Subscriber sub = n.subscribe("/cortex_raw",1000,Datahandler);
	ros::spin();
}

