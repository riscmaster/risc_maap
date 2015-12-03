/*======================================================
    Created by:     D. Spencer Maughan
    Last updated:   July 2014
    File name:      draw_waypoints.cpp
    Organization:   RISC Lab, Utah State University
 ======================================================*/

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <risc_msgs/Waypoints.h>
#include <risc_msgs/Cortex.h>
#include <risc_msgs/Observed_angles.h>

float PI = 3.14159265358979323846;
ros::Publisher way_pub; 
ros::Publisher head_pub; 
ros::Publisher txt_pub;
ros::Publisher line_pub;
visualization_msgs::MarkerArray line;
risc_msgs::Cortex states;
risc_msgs::Waypoints Wp;

/*=============================
 *      Draw Trailing line
 *=============================*/

void drawWayTrail(risc_msgs::Waypoints waypoints)
{

     // At 200 Hz 2000 means a ten second trail
     int length = 500;
     line.markers.resize(waypoints.Obj.size());

     //Loop through all objects
     for (int j = 0; j<waypoints.Obj.size(); j++)
     {
         if(waypoints.Obj[j].x < 4)
         {
     // Set the frame ID and timestamp.  
     line.markers[j].header.frame_id = "/cortex";
     line.markers[j].header.stamp = ros::Time::now();
 
     // Set the namespace and id for this marker.  This serves to create a unique ID
     // Any marker sent with the same namespace and id will overwrite the old one
     line.markers[j].ns = "line";
     line.markers[j].id = j+1;

     // Type
     line.markers[j].type = visualization_msgs::Marker::LINE_STRIP; 
 
     // Set the marker action.  Options are ADD and DELETE
     line.markers[j].action = visualization_msgs::Marker::ADD;

     geometry_msgs::Point X;
 
     // Set the pose of the marker. The line needs multiple points. Hence the use of push_back 
    X.x = waypoints.Obj[j].x;
    X.y = waypoints.Obj[j].y;
    X.z = waypoints.Obj[j].z;
if (line.markers[j].points.size()<length)
{
     line.markers[j].points.push_back(X);
}
if (line.markers[j].points.size()==length)
{
    for (int i = 0; i<length-1; i++)
    {
        line.markers[j].points[i] = line.markers[j].points[i+1];
    }
   line.markers[j].points[length-1]=X;
}
     // Set the scale of the line. It only uses the x for line thickness -- 1x1x1 here means 1m on a side
     line.markers[j].scale.x = .01;

     // Set the color. a controls transparency.
     line.markers[j].color.r = 1.0f;
     line.markers[j].color.g = 0.0f;
     line.markers[j].color.b = 0.0f;
     line.markers[j].color.a = 1;
         }
     }
     line_pub.publish(line);
}

/*=============================
 *      Draw Waypoints
 *=============================*/

void drawWaypoints()
{

     drawWayTrail(Wp);
     // Set our initial shape type to be a cube
     visualization_msgs::MarkerArray Way;
     visualization_msgs::MarkerArray Heading;
     visualization_msgs::MarkerArray Labels;
     Way.markers.resize(Wp.Obj.size());
     Heading.markers.resize(Wp.Obj.size());
     Labels.markers.resize(Wp.Obj.size());

     //Loop through all objects
     for (int j = 0; j<states.Obj.size(); j++)
     {
     // Set the frame ID and timestamp.  See the TF tutorials for information on these.
     if (Wp.Obj[j].x<4)
     {
     Way.markers[j].header.frame_id = "/cortex";
     Heading.markers[j].header.frame_id = "/cortex";
     Labels.markers[j].header.frame_id = "/cortex";
     Way.markers[j].header.stamp = ros::Time::now();
     Heading.markers[j].header.stamp = ros::Time::now();
     Labels.markers[j].header.stamp = ros::Time::now();
 
     // Set the namespace and id for this marker.  This serves to create a unique ID
     // Any marker sent with the same namespace and id will overwrite the old one
     Way.markers[j].ns = Wp.Obj[j].name;
     Heading.markers[j].ns = Wp.Obj[j].name;
     Labels.markers[j].ns = Wp.Obj[j].name;
     Way.markers[j].id = j+1;
     Heading.markers[j].id = j+1;
 
     // Set the marker type. Simple shapes are CUBE, SPHERE, ARROW, and CYLINDER
     Way.markers[j].type = visualization_msgs::Marker::SPHERE; 
     Heading.markers[j].type = visualization_msgs::Marker::ARROW; 
     Labels.markers[j].type = visualization_msgs::Marker::TEXT_VIEW_FACING; 
 
     // Set the marker action.  Options are ADD and DELETE
     Way.markers[j].action = visualization_msgs::Marker::ADD;
 
     // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
     Way.markers[j].pose.position.x = Wp.Obj[j].x;
     Way.markers[j].pose.position.y = Wp.Obj[j].y;
     Way.markers[j].pose.position.z = Wp.Obj[j].z;

     Heading.markers[j].pose.position.x = Wp.Obj[j].x;
     Heading.markers[j].pose.position.y = Wp.Obj[j].y;
     Heading.markers[j].pose.position.z = Wp.Obj[j].z;
     Heading.markers[j].pose.orientation =tf::createQuaternionMsgFromRollPitchYaw(0, 0,Wp.Obj[j].heading*PI/180); 
     Labels.markers[j].pose.position.x = Wp.Obj[j].x;
     Labels.markers[j].pose.position.y = Wp.Obj[j].y;
     Labels.markers[j].pose.position.z = Wp.Obj[j].z+.3;
if(states.Obj.size()>0){
     Labels.markers[j].text = states.Obj[j].name+"'s\nWaypoint"; 
}
     // Set the scale of the marker -- 1x1x1 here means 1m on a side
     Way.markers[j].scale.x = .17;
     Way.markers[j].scale.y = .17;
     Way.markers[j].scale.z = .17;

     Heading.markers[j].scale.x = .17*1.5;
     Heading.markers[j].scale.y = .07;
     Heading.markers[j].scale.z = .07;

     Labels.markers[j].scale.z = .17;

     Way.markers[j].color.r = 1.0f;
     Way.markers[j].color.g = 1.0f;
     Way.markers[j].color.b = 1.0f;
     Way.markers[j].color.a = .2;

     Heading.markers[j].color.r = 1.0f;
     Heading.markers[j].color.g = 1.0f;
     Heading.markers[j].color.b = 1.0f;
     Heading.markers[j].color.a = .2;


     Labels.markers[j].color.r = 1.0f;
     Labels.markers[j].color.g = 1.0f;
     Labels.markers[j].color.b = 1.0f;
     Labels.markers[j].color.a = .4;

     Way.markers[j].lifetime = ros::Duration();
     }
     }
     way_pub.publish(Way);
     head_pub.publish(Heading);
     txt_pub.publish(Labels); 

}

/*========================
 *      Get States
 *========================*/
void getStates(risc_msgs::Cortex x)
{states = x;}

/*========================
 *      Get Waypoints
 *========================*/
void getWaypoints(risc_msgs::Waypoints x)
{Wp = x;if(Wp.Obj.size()!=0 && states.Obj.size()!=0){ drawWaypoints();} }


/*================
 *      Main
 *================*/

 int main( int argc, char** argv )
 {
   ros::init(argc, argv, "draw_waypoints");
   ros::NodeHandle n;

/*================
 *   Publishers
 *================*/

   way_pub = n.advertise<visualization_msgs::MarkerArray>("Way_Markers", 1);
   head_pub = n.advertise<visualization_msgs::MarkerArray>("Heading_Markers", 1);
   txt_pub = n.advertise<visualization_msgs::MarkerArray>("Way_Labels", 1);
   line_pub = n.advertise<visualization_msgs::MarkerArray>("Way_trail", 100);
 
/*================
 *   Subscribers
 *================*/

   ros::Subscriber stub = n.subscribe("/cortex", 1000, getStates);
   ros::Subscriber sub = n.subscribe("/waypoints", 1000, getWaypoints);
   ros::spin();
 }
