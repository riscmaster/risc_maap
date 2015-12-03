/*======================================================
    Created by:     D. Spencer Maughan
    Last updated:   July 2014
    File name:      draw_cortex.cpp
    Organization:   RISC Lab, Utah State University
 ======================================================*/

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <risc_msgs/Cortex.h>
#include <risc_msgs/Mocap_data.h>
#include <risc_msgs/Landmarks.h>
#include <risc_msgs/Observed_angles.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <roscopter/Status.h>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <sstream>

#define SSTR( x ) dynamic_cast< std::ostringstream & >( \
                ( std::ostringstream() << std::dec << x ) ).str()

float PI = 3.14159265358979323846;
ros::Publisher quad_pub; 
ros::Publisher status_pub; 
ros::Publisher battery_pub; 
ros::Publisher markers_pub; 
ros::Publisher land_pub;
ros::Publisher quad_txt_pub;
ros::Publisher land_txt_pub;
ros::Publisher ctrl_txt_pub;
ros::Publisher cov_pub;
risc_msgs::Landmarks Lm;
tf::TransformListener* listener = NULL; 
ros::NodeHandle* n = NULL; 

/*============================================
 *      Get and Display Quad Covariance
 *============================================*/

void getCovariance(geometry_msgs::TwistStamped X)
{
    visualization_msgs::Marker batt;
    batt.header.frame_id = "/cortex";
    batt.header.stamp = ros::Time();
    batt.ns = "position_cov";
    batt.id = 0;
    batt.type = visualization_msgs::Marker::SPHERE;
    batt.action = visualization_msgs::Marker::ADD;
    batt.pose.position.x = X.twist.linear.x;
    batt.pose.position.y = X.twist.linear.y;
    batt.pose.position.z = X.twist.linear.z;
    batt.scale.x = 3*sqrt(X.twist.angular.x);
    batt.scale.y = 3*sqrt(X.twist.angular.y);
    batt.scale.z = 3*sqrt(X.twist.angular.z);
    batt.color.a = .2;
    batt.color.r = 1;
    batt.color.g = 0.0;
    batt.color.b = 0.0;
    batt.lifetime = ros::Duration();
    cov_pub.publish(batt);
}

/*============================================
 *      Get and Display Battery Status
 *============================================*/

void getArdroneBatt(ardrone_autonomy::Navdata X)
{
    float battery = X.batteryPercent;
    visualization_msgs::Marker batt;
    batt.header.frame_id = "/cortex";
    batt.header.stamp = ros::Time();
    batt.ns = "battery_status";
    batt.id = 0;
    batt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    batt.action = visualization_msgs::Marker::ADD;
    batt.pose.position.x = 3;
    batt.pose.position.y = -2.5;
    batt.pose.position.z = 0;
    batt.scale.z = 0.3;
    batt.color.a = 1.0;
    float red = (((100-battery)/100)-.5)/.5;
    float green = (battery-50/50);
    if (red>1){red = 1; green = 0;}
    batt.color.r = red;
    batt.color.g = green;
    batt.color.b = 0.0;
    batt.text = "Battery: "+SSTR(battery)+"%";
    batt.lifetime = ros::Duration();
    battery_pub.publish(batt);
}
/*============================================
 *      Get and Display Battery Status
 *============================================*/

void getBatteryStatus(roscopter::Status X)
{
    int battery = X.battery_remaining;
    visualization_msgs::Marker batt;
    batt.header.frame_id = "/cortex";
    batt.header.stamp = ros::Time();
    batt.ns = "battery_status";
    batt.id = 0;
    batt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    batt.action = visualization_msgs::Marker::ADD;
    batt.pose.position.x = 3;
    batt.pose.position.y = -2.5;
    batt.pose.position.z = 0;
    batt.scale.z = 0.3;
    batt.color.a = 1.0;
    batt.color.r = (100-(float)battery)/100;
    batt.color.g = (float)battery/100;
    batt.color.b = 0.0;
    batt.text = "Battery: "+SSTR(battery)+"%";
    batt.lifetime = ros::Duration();
    battery_pub.publish(batt);
}

/*============================================
 *      Get and Display Controller Status
 *============================================*/

void getStatus(std_msgs::Bool X)
{
    visualization_msgs::Marker Status;
    Status.header.frame_id = "/cortex";
    Status.header.stamp = ros::Time();
    Status.ns = "controller_status";
    Status.id = 0;
    Status.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    Status.action = visualization_msgs::Marker::ADD;
    Status.pose.position.x = -3;
    Status.pose.position.y = -2;
    Status.pose.position.z = 0;
    Status.scale.z = 0.3;
    Status.color.a = 1.0;
    if ( X.data == true)
    {
     //Autonomous Control
    Status.color.r = 0.0;
    Status.color.g = 1.0;
    Status.color.b = 0.0;
    Status.text = "Autonomous";
    }
    else
    {
     //Manual Control
    Status.color.r = 1.0;
    Status.color.g = 0.4;
    Status.color.b = 0.0;
    Status.text = "Manual";
    }
    Status.lifetime = ros::Duration();
    ctrl_txt_pub.publish(Status);
}

/*========================
 *      Draw Markers
 *========================*/

void drawMarkers(risc_msgs::Mocap_data states)
{
int number_of_markers = 0;
     // Ensure all markers fit
     for (int i = 0; i<states.Obj.size(); i++)
     {for (int j = 0; j<states.Obj[i].marker.size(); j++){number_of_markers++;}}
     visualization_msgs::MarkerArray Markers;
     Markers.markers.resize(number_of_markers);
     int index = 0;
     //Loop through all templates
     for (int i = 0; i<states.Obj.size(); i++)
     {
     //Loop through all markers
     for (int j = 0; j<states.Obj[i].marker.size(); j++)
     {

     // Set the frame ID and timestamp.  
     Markers.markers[index].header.frame_id = "/cortex";
     Markers.markers[index].header.stamp = ros::Time::now();
 
     // Set the namespace and id for this marker.  This serves to create a unique ID
     // Any marker sent with the same namespace and id will overwrite the old one
     Markers.markers[index].ns = states.Obj[i].name;
     Markers.markers[index].id = index+1;
 
     // Set the marker type. Simple shapes are CUBE, SPHERE, ARROW, and CYLINDER
     Markers.markers[index].type = visualization_msgs::Marker::SPHERE; 

     // Set the marker action.  Options are ADD and DELETE
     Markers.markers[index].action = visualization_msgs::Marker::ADD;

     // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
     Markers.markers[index].pose.position.x = states.Obj[i].marker[j].x;
     Markers.markers[index].pose.position.y = states.Obj[i].marker[j].y;
     Markers.markers[index].pose.position.z = states.Obj[i].marker[j].z;
     
     // Set the scale of the marker -- 1x1x1 here means 1m on a side
     Markers.markers[index].scale.x = .03;
     Markers.markers[index].scale.y = .03;
     Markers.markers[index].scale.z = .03;
     
 
     // Set the color -- be sure to set alpha to something non-zero!
     Markers.markers[index].color.r = 1.0f;
     Markers.markers[index].color.g = 1.0f;
     Markers.markers[index].color.b = 1.0f;
     Markers.markers[index].color.a = 1;
     Markers.markers[index].lifetime = ros::Duration();
     index++;
     }

     for (int i = 0; i<Markers.markers.size(); i++)
     {
     Markers.markers[i].header.frame_id = "/cortex";
     Markers.markers[i].scale.x = .03;
     Markers.markers[i].scale.y = .03;
     Markers.markers[i].scale.z = .03;
     

     }
     markers_pub.publish(Markers);
     }
}

/*=============================
 *      Draw Quadrotors
 *=============================*/

void drawQuads(risc_msgs::Cortex states)
{
 
     // Set our initial shape type to be a cube
     visualization_msgs::MarkerArray Quad;
     visualization_msgs::MarkerArray Labels;
     Quad.markers.resize(states.Obj.size());
     Labels.markers.resize(states.Obj.size());

     //Loop through all objects
     for (int j = 0; j<states.Obj.size(); j++)
     {
         if(states.Obj[j].x < 4)
         {
     // Set the frame ID and timestamp.  
     Quad.markers[j].header.frame_id = states.Obj[j].name+"_body_frame";
     Labels.markers[j].header.frame_id = states.Obj[j].name+"_body_frame";
     Quad.markers[j].header.stamp = ros::Time::now();
     Labels.markers[j].header.stamp = ros::Time::now();
 
     // Set the namespace and id for this marker.  This serves to create a unique ID
     // Any marker sent with the same namespace and id will overwrite the old one
     Quad.markers[j].ns = states.Obj[j].name;
     Labels.markers[j].ns = states.Obj[j].name;
     Quad.markers[j].id = j+1;
 
     // Set the marker type. Simple shapes are CUBE, SPHERE, ARROW, and CYLINDER
     //Quad.markers[j].type = visualization_msgs::Marker::SPHERE; 
     Quad.markers[j].type = visualization_msgs::Marker::MESH_RESOURCE; 
     if(states.Obj[j].name == "Pendulum_tip")
     {
     Quad.markers.resize(states.Obj.size()+1);
     Quad.markers[states.Obj.size()].header.frame_id = "/cortex";
     Quad.markers[j].header.frame_id = "/cortex"; 
     Labels.markers[j].header.frame_id =  "/cortex";
     Quad.markers[states.Obj.size()].type = visualization_msgs::Marker::LINE_STRIP;
     Quad.markers[j].type = visualization_msgs::Marker::SPHERE; 
     Labels.markers[j].type = visualization_msgs::Marker::TEXT_VIEW_FACING; 
 
     // Set the marker action.  Options are ADD and DELETE
     Quad.markers[j].action = visualization_msgs::Marker::ADD;
     Quad.markers[states.Obj.size()].action = visualization_msgs::Marker::ADD;

     // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
     geometry_msgs::Point X1;
     X1.x = states.Obj[0].x;
     X1.y = states.Obj[0].y;
     X1.z = states.Obj[0].z;
     geometry_msgs::Point X2;
     X2.x = states.Obj[1].x;
     X2.y = states.Obj[1].y;
     X2.z = states.Obj[1].z;
     Quad.markers[states.Obj.size()].points.resize(2);
     Quad.markers[states.Obj.size()].points[0] = X1;
     Quad.markers[states.Obj.size()].points[1] = X2;
     Quad.markers[j].pose.position.x = states.Obj[j].x;
     Quad.markers[j].pose.position.y = states.Obj[j].y;
     Quad.markers[j].pose.position.z = states.Obj[j].z;
     Labels.markers[j].pose.position.x = states.Obj[j].x;
     Labels.markers[j].pose.position.y = states.Obj[j].y;
     Labels.markers[j].pose.position.z = states.Obj[j].z+.2;

     Labels.markers[j].text = states.Obj[j].name; 
     
     // Set the scale of the marker -- 1x1x1 here means 1m on a side
     Quad.markers[j].scale.x = .076;
     Quad.markers[j].scale.y = .076;
     Quad.markers[j].scale.z = .076;
     
     Quad.markers[states.Obj.size()].scale.x = .02;
     Labels.markers[j].scale.z = .17;
 
     // Set the color -- be sure to set alpha to something non-zero!
     Quad.markers[states.Obj.size()].color.r = 1.0f;
     Quad.markers[states.Obj.size()].color.g = 0.6f;
     Quad.markers[states.Obj.size()].color.b = 0.0f;
     Quad.markers[states.Obj.size()].color.a = .7;
     Quad.markers[j].color.r = 1.0f;
     Quad.markers[j].color.g = 0.6f;
     Quad.markers[j].color.b = 0.0f;
     Quad.markers[j].color.a = 1;

     Labels.markers[j].color.r = 1.0f;
     Labels.markers[j].color.g = 1.0f;
     Labels.markers[j].color.b = 1.0f;
     Labels.markers[j].color.a = .4;
     Quad.markers[j].lifetime = ros::Duration();
     Quad.markers[states.Obj.size()].lifetime = ros::Duration();

     }
     else
     { 
     Quad.markers[j].mesh_resource = "package://risc_visual/3d_models/Drone.dae";
    // Quad.markers[j].mesh_resource = "package://risc_visual/3d_models/bat_cave/batcave.DAE";
     Labels.markers[j].type = visualization_msgs::Marker::TEXT_VIEW_FACING; 
 
     // Set the marker action.  Options are ADD and DELETE
     Quad.markers[j].action = visualization_msgs::Marker::ADD;

     // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
     Quad.markers[j].pose.position.x = 0;
     Quad.markers[j].pose.position.y = 0;
     Quad.markers[j].pose.position.z = 0;
     //For Mario
     Quad.markers[j].pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,PI,PI/2);
     //For Drone thingy
    // Quad.markers[j].pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
     Labels.markers[j].pose.position.x = 0;
     Labels.markers[j].pose.position.y = 0;
     Labels.markers[j].pose.position.z = -.2;

     Labels.markers[j].text = states.Obj[j].name; 
     
     // Set the scale of the marker -- 1x1x1 here means 1m on a side
     Quad.markers[j].scale.x = .1;
     Quad.markers[j].scale.y = .1;
     Quad.markers[j].scale.z = .1;
     
     Labels.markers[j].scale.z = .17;
 
     // Set the color -- be sure to set alpha to something non-zero!
     Quad.markers[j].color.r = 0.0f;
     Quad.markers[j].color.g = 0.0f;
     Quad.markers[j].color.b = 0.0f;
     Quad.markers[j].color.a = .1;
     Quad.markers[j].mesh_use_embedded_materials = true;

     Labels.markers[j].color.r = 1.0f;
     Labels.markers[j].color.g = 1.0f;
     Labels.markers[j].color.b = 1.0f;
     Labels.markers[j].color.a = .4;
     Quad.markers[j].lifetime = ros::Duration();
     }
         }
     }

     quad_pub.publish(Quad);
     quad_txt_pub.publish(Labels); 
}


/*=============================
 *      Draw Landmarks
 *=============================*/

void drawLandmarks(risc_msgs::Landmarks landmarks)
{
     Lm = landmarks;
 
     // Set our initial shape type to be a cube
     visualization_msgs::MarkerArray Land;
     visualization_msgs::MarkerArray Labels;
     Land.markers.resize(landmarks.Obj.size());
     Labels.markers.resize(landmarks.Obj.size());

     //Loop through all objects
     for (int j = 0; j<landmarks.Obj.size(); j++)
     {
     // Set the frame ID and timestamp.  See the TF tutorials for information on these.
     if (landmarks.Obj[j].x<4)
     {
     Land.markers[j].header.frame_id = "/cortex";
     Labels.markers[j].header.frame_id = "/cortex";
     Land.markers[j].header.stamp = ros::Time::now();
     Labels.markers[j].header.stamp = ros::Time::now();
 
     // Set the namespace and id for this marker.  This serves to create a unique ID
     // Any marker sent with the same namespace and id will overwrite the old one
     Land.markers[j].ns = landmarks.Obj[j].name;
     Labels.markers[j].ns = landmarks.Obj[j].name;
     Land.markers[j].id = j+1;
 
     // Set the marker type. Simple shapes are CUBE, SPHERE, ARROW, and CYLINDER
     Land.markers[j].type = visualization_msgs::Marker::SPHERE; 
     Labels.markers[j].type = visualization_msgs::Marker::TEXT_VIEW_FACING; 
 
     // Set the marker action.  Options are ADD and DELETE
     Land.markers[j].action = visualization_msgs::Marker::ADD;
 
     // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
     Land.markers[j].pose.position.x = landmarks.Obj[j].x;
     Land.markers[j].pose.position.y = landmarks.Obj[j].y;
     Land.markers[j].pose.position.z = landmarks.Obj[j].z;
     Labels.markers[j].pose.position.x = landmarks.Obj[j].x;
     Labels.markers[j].pose.position.y = landmarks.Obj[j].y;
     Labels.markers[j].pose.position.z = landmarks.Obj[j].z+.3;

     Labels.markers[j].text = landmarks.Obj[j].name; 
     
     // Set the scale of the marker -- 1x1x1 here means 1m on a side
     Land.markers[j].scale.x = .17;
     Land.markers[j].scale.y = .17;
     Land.markers[j].scale.z = .17;
     Labels.markers[j].scale.z = .17;

     Land.markers[j].color.r = 1.0f;
     Land.markers[j].color.g = 1.0f;
     Land.markers[j].color.b = 1.0f;

 
     // Set the color -- be sure to set alpha to something non-zero!
     if (landmarks.Obj[j].name == "Orange_blue")
     {
     Land.markers[j].color.r = 1.0f;
     Land.markers[j].color.g = 0.6f;
     Land.markers[j].color.b = 0.0f;
     }
     if (landmarks.Obj[j].name == "Green_pink")
     {
     Land.markers[j].color.r = 0.0f;
     Land.markers[j].color.g = 1.0f;
     Land.markers[j].color.b = 0.0f;
     }

     if (landmarks.Obj[j].name == "Blue_green")
     {
     Land.markers[j].color.r = 0.0f;
     Land.markers[j].color.g = 0.0f;
     Land.markers[j].color.b = 1.0f;
     }

     if (landmarks.Obj[j].name == "Pink_blue")
     {
     Land.markers[j].color.r = 1.0f;
     Land.markers[j].color.g = 0.0f;
     Land.markers[j].color.b = 0.5f;
     }

     Land.markers[j].color.a = .4;

     Labels.markers[j].color.r = 1.0f;
     Labels.markers[j].color.g = 1.0f;
     Labels.markers[j].color.b = 1.0f;
     Labels.markers[j].color.a = .4;

     Land.markers[j].lifetime = ros::Duration();
     }
     }

     land_pub.publish(Land);
     land_txt_pub.publish(Labels); 

}

/*================
 *      Main
 *================*/

 int main( int argc, char** argv )
 {
   ros::init(argc, argv, "draw_cortex_data");
   n = new (ros::NodeHandle);
   listener = new (tf::TransformListener);

/*================
 *   Publishers
 *================*/

   markers_pub = n->advertise<visualization_msgs::MarkerArray>("Cortex_Markers", 100);
   status_pub = n->advertise<visualization_msgs::Marker>("Controller_Status", 100);
   battery_pub = n->advertise<visualization_msgs::Marker>("Battery_Status", 100);
   quad_pub = n->advertise<visualization_msgs::MarkerArray>("Quad_Markers", 1);
   quad_txt_pub = n->advertise<visualization_msgs::MarkerArray>("Quad_Labels", 1);
   land_pub = n->advertise<visualization_msgs::MarkerArray>("Land_Markers", 100);
   land_txt_pub = n->advertise<visualization_msgs::MarkerArray>("Land_Labels", 100);
   ctrl_txt_pub = n->advertise<visualization_msgs::Marker>("Ctrl_Status", 100);
   cov_pub = n->advertise<visualization_msgs::Marker>("Cov_Quad", 100);
 
/*================
 *   Subscribers
 *================*/

   ros::Subscriber sub_quads = n->subscribe("/cortex_raw", 100, drawQuads);
   ros::Subscriber sub_markers = n->subscribe("/mocap_data", 100, drawMarkers);
   ros::Subscriber sub_landmarks = n->subscribe("/states3", 100, drawLandmarks);
   ros::Subscriber sub_ctrl = n->subscribe("/controller_status", 100, getStatus);
   ros::Subscriber sub_battery = n->subscribe("apm/status", 100, getBatteryStatus);
   ros::Subscriber sub_batt = n->subscribe("ardrone/Navdata", 100, getArdroneBatt);
   ros::Subscriber sub_cov = n->subscribe("position_cov", 100, getCovariance);
   ros::spin();
 }
