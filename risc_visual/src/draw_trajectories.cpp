/*======================================================
    Created by:     D. Spencer Maughan
    Last updated:   December 2014
    File name:      draw_trajectories.cpp
    Organization:   RISC Lab, Utah State University
 ======================================================*/

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <risc_msgs/Cortex.h>
#include <risc_msgs/Landmarks.h>
#include <risc_msgs/Observed_angles.h>
#include <risc_msgs/Trajectories.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

float PI = 3.14159265358979323846;
ros::Publisher line_pub;
ros::Publisher traj_pub;
ros::Publisher traj_name_pub;
visualization_msgs::MarkerArray quad_line;
visualization_msgs::MarkerArray traj_line;
std::string name_past = ("name");
tf::TransformListener* listener = NULL; 

/*=============================
 *      Trajectory Trailing line
 *=============================*/
void showName(std::string name)

{    visualization_msgs::Marker traj_name;
     traj_name.header.frame_id = "/cortex";
     traj_name.header.stamp = ros::Time::now();
     traj_name.ns = "traj_name";
     traj_name.id = 0;
     traj_name.text = name;
     traj_name.type = visualization_msgs::Marker::TEXT_VIEW_FACING; 
     traj_name.action = visualization_msgs::Marker::ADD;
     traj_name.scale.z = .3;
     traj_name.pose.position.x = 0;
     traj_name.pose.position.y = 0;
     traj_name.pose.position.z = 3;
     traj_name.color.r = 1.0f;
     traj_name.color.g = 0.4f;
     traj_name.color.b = 0.0f;
     traj_name.color.a = 1;
     traj_name_pub.publish(traj_name);
}
  
void drawTrajectory(risc_msgs::Trajectories states)
{

     // At 200 Hz 2000 means a ten second trail
     int length = 8000;
     traj_line.markers.resize(states.Obj.size());

     //Loop through all objects
     //for (int j = 0; j<states.Obj.size(); j++)
     //{
     int j = 0;
         if(states.Obj[j].x < 4)
         {
     showName(states.Obj[j].name);
             // if the name of the trajectory changes reset line
     if (states.Obj[j].name != name_past)
     {traj_line.markers[j].points.resize(0);}
     // Set the frame ID and timestamp.  
     traj_line.markers[j].header.frame_id = "/cortex";
     traj_line.markers[j].header.stamp = ros::Time::now();
 
     // Set the namespace and id for this marker.  This serves to create a unique ID
     // Any marker sent with the same namespace and id will overwrite the old one
     traj_line.markers[j].ns = "traj_line";
     traj_line.markers[j].id = j+1;

     // Type
     traj_line.markers[j].type = visualization_msgs::Marker::LINE_STRIP; 
     if (states.Obj[j].name == "Origin" or states.Obj[j].name == "Quadrant I" or states.Obj[j].name == "Quadrant II" or states.Obj[j].name == "Quadrant III" or states.Obj[j].name == "Quadrant IV")
     {
  
     traj_line.markers[j].type = visualization_msgs::Marker::SPHERE; 
     // Set the marker action.  Options are ADD and DELETE
     traj_line.markers[j].action = visualization_msgs::Marker::ADD;

     geometry_msgs::Point X;
 
     // Set the pose of the marker. The line needs multiple points. Hence the use of push_back 
    X.x = states.Obj[j].x;
    X.y = states.Obj[j].y;
    X.z = states.Obj[j].z;
    traj_line.markers[j].pose.position = X;

     // Set the scale of the line. It only uses the x for line thickness -- 1x1x1 here means 1m on a side
     traj_line.markers[j].scale.x = .1;
     traj_line.markers[j].scale.y = .1;
     traj_line.markers[j].scale.z = .1;

     // Set the color. a controls transparency.
     traj_line.markers[j].color.r = 1.0f;
     traj_line.markers[j].color.g = 0.4f;
     traj_line.markers[j].color.b = 0.0f;
     traj_line.markers[j].color.a = 1;
         }
    

     else{
     // Set the marker action.  Options are ADD and DELETE
     traj_line.markers[j].action = visualization_msgs::Marker::ADD;

     geometry_msgs::Point X;
 
     // Set the pose of the marker. The line needs multiple points. Hence the use of push_back 
    X.x = states.Obj[j].x;
    X.y = states.Obj[j].y;
    X.z = states.Obj[j].z;
if (traj_line.markers[j].points.size()<length)
{
     traj_line.markers[j].points.push_back(X);
}
if (traj_line.markers[j].points.size()==length)
{
    for (int i = 0; i<length-1; i++)
    {
        traj_line.markers[j].points[i] = traj_line.markers[j].points[i+1];
    }
   traj_line.markers[j].points[length-1]=X;
}
     // Set the scale of the line. It only uses the x for line thickness -- 1x1x1 here means 1m on a side
     traj_line.markers[j].scale.x = .01;

     // Set the color. a controls transparency.
     traj_line.markers[j].color.r = 0.0f;
     traj_line.markers[j].color.g = 1.0f;
     traj_line.markers[j].color.b = 0.0f;
     traj_line.markers[j].color.a = 1;
     }
     traj_pub.publish(traj_line);
     name_past = states.Obj[j].name;
     }
}

/*=============================
 *      Quad Trailing line
 *=============================*/

void drawQuadLines(risc_msgs::Cortex states)
{

     // At 200 Hz 2000 means a ten second trail
     int length = 1000;

     quad_line.markers.resize(states.Obj.size());

     //Loop through all objects
     for (int j = 0; j<states.Obj.size(); j++)
     {
         if(states.Obj[j].x < 4)
         {
     // Set the frame ID and timestamp.  
     quad_line.markers[j].header.frame_id = "/cortex";
     quad_line.markers[j].header.stamp = ros::Time::now();
 
     // Set the namespace and id for this marker.  This serves to create a unique ID
     // Any marker sent with the same namespace and id will overwrite the old one
     quad_line.markers[j].ns = "quad_line";
     quad_line.markers[j].id = j+1;

     // Type
     quad_line.markers[j].type = visualization_msgs::Marker::LINE_STRIP; 
 
     // Set the marker action.  Options are ADD and DELETE
     quad_line.markers[j].action = visualization_msgs::Marker::ADD;

     geometry_msgs::Point X;
 
     // Set the pose of the marker. The line needs multiple points. Hence the use of push_back 
    X.x = states.Obj[j].x;
    X.y = states.Obj[j].y;
    X.z = states.Obj[j].z;
if (quad_line.markers[j].points.size()<length)
{
     quad_line.markers[j].points.push_back(X);
}
if (quad_line.markers[j].points.size()==length)
{
    for (int i = 0; i<length-1; i++)
    {
        quad_line.markers[j].points[i] = quad_line.markers[j].points[i+1];
    }
   quad_line.markers[j].points[length-1]=X;
}
     // Set the scale of the line. It only uses the x for line thickness -- 1x1x1 here means 1m on a side
     quad_line.markers[j].scale.x = .01;

     // Set the color. a controls transparency.
     quad_line.markers[j].color.r = 1.0f;
     quad_line.markers[j].color.g = 0.0f;
     quad_line.markers[j].color.b = 0.0f;
     quad_line.markers[j].color.a = 1;
         }
     }
     line_pub.publish(quad_line);
}

/*================
 *      Main
 *================*/

 int main( int argc, char** argv )
 {
   ros::init(argc, argv, "draw_trajectories");
   ros::NodeHandle n;
   listener = new (tf::TransformListener);

/*================
 *   Publishers
 *================*/

   line_pub = n.advertise<visualization_msgs::MarkerArray>("Quad_trajectory", 100);
   traj_pub = n.advertise<visualization_msgs::MarkerArray>("desired_trajectory", 100);
   traj_name_pub = n.advertise<visualization_msgs::Marker>("desired_trajectory_name", 100);
 
/*================
 *   Subscribers
 *================*/

   ros::Subscriber sub = n.subscribe("/cortex_raw", 1000, drawQuadLines);
   ros::Subscriber traj_sub = n.subscribe("/trajectory", 1000, drawTrajectory);
   ros::spin();
 }
