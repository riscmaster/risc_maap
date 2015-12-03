/*======================================================
    Created by:     D. Spencer Maughan
    Last updated:   July 2014
    File name:      draw_angles.cpp
    Organization:   RISC Lab, Utah State University
 ======================================================*/

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <risc_msgs/Observed_angles.h>
#include <risc_msgs/Landmarks.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

float PI = 3.14159265358979323846;
ros::Publisher line_pub;

/*============================================
 *    Create Given Mag,Azim,Elev and color 
 *===========================================*/

visualization_msgs::Marker angle2marker(std::string name, std::string frame, float azim, float elev, float a)
{
int mag = 4;
     visualization_msgs::Marker Marker;
     Marker.header.frame_id = frame;
     Marker.header.stamp = ros::Time::now();
     Marker.ns = name; 
     Marker.id = 0;
 
     // Type
     Marker.type = visualization_msgs::Marker::LINE_STRIP; 
 
if(abs(azim)<40 && abs(elev)<30){
     // Set the marker action.  Options are ADD and DELETE
     Marker.action = visualization_msgs::Marker::ADD;

	geometry_msgs::PointStamped Zero;
        Zero.point.x = 0;
        Zero.point.y = 0;
        Zero.point.z = 0;
	Zero.header.frame_id = frame;
        geometry_msgs::PointStamped E;
if(abs(azim)<90)
{
        E.point.x = mag*cos(azim*PI/180)*cos(-elev*PI/180);
        E.point.y = mag*sin(azim*PI/180);
	E.point.z = mag*sin(-elev*PI/180);
}
if(abs(azim)>=90)
{
        E.point.x = -mag*sin(-azim*PI/180)*cos(-elev*PI/180);
        E.point.y = mag*cos(-azim*PI/180)*cos(-elev*PI/180);
	E.point.z = -mag*sin(-elev*PI/180);
}
	E.header.frame_id = frame;
        Marker.points.resize(2);
        Marker.points[0] = Zero.point;
        Marker.points[1] = E.point;
if(name == "fluffy")
{
     Marker.color.r = 0.0f;
     Marker.color.g = 0.0f;
     Marker.color.b = 0.0f;
}
if(name == "baby_bird")
{
     Marker.color.r = 0.5f;
     Marker.color.g = 0.5f;
     Marker.color.b = 0.5f;
}
if(name == "Blue_green" or name == "blue")
{
     Marker.color.r = 0.0f;
     Marker.color.g = 0.0f;
     Marker.color.b = 1.0f;
}
if(name == "Green_pink" or name == "green")
{
     Marker.color.r = 0.0f;
     Marker.color.g = 1.0f;
     Marker.color.b = 0.0f;
}
if(name == "Pink_blue" or name == "pink")
{
     Marker.color.r = 0.7804f;
     Marker.color.g = 0.082353f;
     Marker.color.b = 0.5216f;
}
if(name == "Orange_blue" or name == "Orange")
{
     Marker.color.r = 1.0f;
     Marker.color.g = 0.274510f;
     Marker.color.b = 0.0f;
}
        Marker.color.a = a;
if(name == "Orange"  or name == "pink" or name == "green" or name == "blue")
{        Marker.lifetime = ros::Duration(.05);
         Marker.scale.x = .04;
}
if(name == "Orange_blue"  or name == "Pink_blue" or name == "Green_pink" or name == "Blue_green")
{        Marker.lifetime = ros::Duration();
         Marker.scale.x = .01;
}

	return Marker;
}
}

/*===============================
 *    Create The MarkerArray 
 *==============================*/

visualization_msgs::MarkerArray drawLines(risc_msgs::Observed_angles ang,float a)
{

     int num = ang.Obj.size();
     visualization_msgs::MarkerArray line;

	// Cycle through objects
     for ( int j = 0; j<num; j++)
     {
        int marks = ang.Obj[j].landmarks.size();

	// Cycle through landmarks
	     for ( int k = 0; k<marks; k++)
		     {
			if (ang.Obj[j].landmarks[k].visible == true && abs(ang.Obj[j].landmarks[k].elev)<40 && abs(ang.Obj[j].landmarks[k].azim)<80  )
			{

visualization_msgs::Marker mark = angle2marker(ang.Obj[j].landmarks[k].name, ang.Obj[j].name+"/camera", ang.Obj[j].landmarks[k].azim,ang.Obj[j].landmarks[k].elev, a);
			line.markers.push_back(mark);
			}
		     }
 int quads = ang.Obj[j].quads.size();

	// Cycle through landmarks
	     for ( int k = 0; k<quads; k++)
		     {
			if (ang.Obj[j].quads[k].visible == true && abs(ang.Obj[j].quads[k].elev)<40 && abs(ang.Obj[j].quads[k].azim)<80  )
			{

visualization_msgs::Marker mark = angle2marker(ang.Obj[j].quads[k].name, ang.Obj[j].name+"/camera", ang.Obj[j].quads[k].azim,ang.Obj[j].quads[k].elev, a);
			line.markers.push_back(mark);
			}
		     }

     }

     return line;
}

/*===============================
 *    Convert Angles to Lines 
 *==============================*/

void Angles(risc_msgs::Observed_angles ang)
{
visualization_msgs::MarkerArray line = drawLines(ang,.4);
line_pub.publish(line);
}

/*================
 *      Main
 *================*/

 int main( int argc, char** argv )
 {
   ros::init(argc, argv, "draw_angle_data");
   ros::NodeHandle n;

/*================
 *   Publishers
 *================*/

   line_pub = n.advertise<visualization_msgs::MarkerArray>("angle_lines", 100);
 
/*================
 *   Subscribers
 *================*/

   ros::Subscriber stub = n.subscribe("/angles", 1000, Angles);
   ros::spin();
 }
