/*======================================================
	Created by:  	D. Spencer Maughan
	Last updated: 	May 2014
	File name: 		test.cpp
	Organization:	RISC Lab, Utah State University
 ======================================================*/

#include "ros/ros.h"
#include "risc_msgs/Waypoints.h"

ros::Publisher p;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "publisher");
  ros::NodeHandle n;

	/*====================
		Set up Publisher
	  ====================*/

p = n.advertise<risc_msgs::Waypoints>("waypoints", 1);
risc_msgs::Waypoints wp;
wp.Obj.resize(1);
ros::Rate loop_rate(60);
while (ros::ok())
{
wp.Obj[0].x = 0;
wp.Obj[0].y = 0;
wp.Obj[0].z = 1;

wp.Obj[1].x = .8;
wp.Obj[1].y = .8;
wp.Obj[1].z = 1;

p.publish(wp);
    ros::spinOnce();
loop_rate.sleep();
}
  return 0;
}

