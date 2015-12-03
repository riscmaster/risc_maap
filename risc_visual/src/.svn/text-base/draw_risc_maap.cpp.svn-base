/*======================================================
    Created by:     D. Spencer Maughan
    Last updated:   December 2014
    File name:      draw_risc_maap.cpp
    Organization:   RISC Lab, Utah State University
 ======================================================*/

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

float PI = 3.14159265358979323846;
ros::Publisher net_pub;
ros::Publisher Cameras_pub;
ros::Publisher grid_pub;
tf::TransformListener* listener = NULL; 

/*====================
 *   Draw Cameras
 *====================*/

void drawCameras()
{
/* Camera Poses retrieved 21 July 2014
1 ,-3435.74023		,160.325989	,2526.87866	,-94.65644	,-42.4772224	,89.41231

2 ,-3452.21387		,-280.361725	,2567.73584	,-90.60017	,-29.60539	,-85.00805

3 ,-3440.533		,-2763.228	,2572.15283	,-42.8116074	,-36.8320541	,-61.50702

4 ,-3277.656		,-2916.27686	,2586.893	,-42.6305237	,-35.7107162	,-53.12766

5 ,-66.28844		,-2881.53027	,2586.858	,9.314909	,-41.7948151	,69.84536

6 ,345.679932		,-2901.50366	,2565.624	,-16.3303013	,-38.5118332	,-81.23038

7 ,3209.59082		,-2947.59253	,2618.13745	,52.1189232	,-40.5058937	,64.256134

8 ,3612.71118		, -2823.55981	,2576.9353	,43.53559	,-29.7827644	,-92.75532

9 ,3832.68213		,-341.30722	,2615.31616	,91.68391	,-31.4917679	,79.08452

10,3841.23438		,180.480026	,2602.325	,90.68552	,-29.4455242	,-90.58732

11,3768.16		,3500.94946	,2640.47461	,144.283	,-33.8016777	,-34.1653252

12,3386.3147		,3746.567	,2661.13	,135.392181	,-33.7677155	,20.13

13,357.820343		,3808.535	,2564.69141	,-175.07605	,-30.091713	,89.79494

14,-40.23637		,3781.61353	,2588.7915	,-176.612228	,-32.1267776	,-73.74045

15,-3350.20532		,3760.1438	,2617.89746	,-143.599243	,-30.2205544	,-3.046481

16,-3532.556		,3774.02783	,2617.578	,-139.8767	,-26.9584541	,0.03185325
*/
 
     visualization_msgs::MarkerArray Cameras;
     Cameras.markers.resize(16);
    float x[16] = {-3435.74023,-3452.21387,-3440.533,-3277.656,-66.28844,345.679932,3209.59082,3612.71118,3832.68213,3841.23438,3768.16	,3386.3147,357.820343,-40.23637	, -3350.20532,-3532.556};	
    float y[16] = {160.325989,-280.361725,-2763.228,-2916.27686,-2881.53027,-2901.50366,-2947.59253, -2823.55981,-341.30722,180.480026,3500.94946,3746.567,3808.535,3781.61353,3760.1438,3774.02783};
    float z[16] = { 2526.87866,2567.73584,2572.15283,2586.893,2586.858,2565.624,2618.13745,2576.9353,2615.31616,2602.325,2640.47461,2661.13,2564.69141,2588.7915,2617.89746,2617.578};	

    //float phi[16],-40.23637	 = {};
    //float theta[1              6] = {};
    float psi[16] = {90,90,45,45,0,0,-45,-45,-90,-90,-135,-135,180,180,135,135};
                                 
     //Loop through,-3532.556	 all objects
     for (int j = 0; j<16; j++)
     {
     Cameras.markers[j].header.frame_id = "/cortex";
     Cameras.markers[j].header.stamp = ros::Time::now();
     Cameras.markers[j].ns = "Camera"; 
     Cameras.markers[j].id = j;
 
     // Set the marker type. Simple shapes are CUBE, SPHERE, ARROW, and CYLINDER
     Cameras.markers[j].type = visualization_msgs::Marker::MESH_RESOURCE; 
     Cameras.markers[j].mesh_resource = "package://risc_visual/3d_models/TV_camera.dae";

     // Set the marker action.  Options are ADD and DELETE
     Cameras.markers[j].action = visualization_msgs::Marker::ADD;
 
     // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
     // We only need Position for the Camerasting
     Cameras.markers[j].pose.position.x = x[j]*.001;
     Cameras.markers[j].pose.position.y = y[j]*.001;
     Cameras.markers[j].pose.position.z = z[j]*.001;
     Cameras.markers[j].pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(30*PI/180, 0*PI/180,PI-psi[j]*PI/180);
     Cameras.markers[j].scale.x = .2;
     Cameras.markers[j].scale.y = .2;
     Cameras.markers[j].scale.z = .2;
 
     // Set the color. a controls the transparency 
     Cameras.markers[j].color.r = 1.0f;
     Cameras.markers[j].color.g = 1.0f;
     Cameras.markers[j].color.b = 1.0f;
     Cameras.markers[j].color.a = .1;
        }
     Cameras_pub.publish(Cameras);
}

/*==========================
 *      Draw Grid Labels
 *==========================*/

void drawGrid()
{
 
     visualization_msgs::MarkerArray grid;
     float step = .5;
     int size = ceil(8/step +4);
     int half = ceil(4/step+1);
     //ROS_INFO("size = %i",size);
     //ROS_INFO("half = %i",half);
     grid.markers.resize(size);

    
     //Loop through all objects
     float value = 2;
     for (int j = 0; j<half; j++)
     {
     grid.markers[j].header.frame_id = "/cortex";
     grid.markers[j+half].header.frame_id = "/cortex";
     grid.markers[j].header.stamp = ros::Time::now();
     grid.markers[j+half].header.stamp = ros::Time::now();
     grid.markers[j].ns = "grid"; 
     grid.markers[j+half].ns = "grid"; 
     grid.markers[j].id = j;
     grid.markers[j+half].id = j+half;
 
     // Set the marker type. Simple shapes are CUBE, SPHERE, ARROW, and CYLINDER
     grid.markers[j].type = visualization_msgs::Marker::TEXT_VIEW_FACING; 
     grid.markers[j+half].type = visualization_msgs::Marker::TEXT_VIEW_FACING; 
 
     // Set the marker action.  Options are half and DELETE
     grid.markers[j].action = visualization_msgs::Marker::ADD;
     grid.markers[j+half].action = visualization_msgs::Marker::ADD;
 
     char number[16]; // string which will contain the number
     sprintf ( number, "%1.1f", value ); // %d makes the result be a decimal integer 
     grid.markers[j].text = number;
     grid.markers[j+half].text = number;
     // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
     // We only need Position for the netting
     grid.markers[j].pose.position.x = value;
     grid.markers[j].pose.position.y = 2.5;
     grid.markers[j].pose.position.z = 0;
     grid.markers[j+half].pose.position.x = 2.5;
     grid.markers[j+half].pose.position.y = value;
     grid.markers[j+half].pose.position.z = 0;
     grid.markers[j].scale.z = .2;
     grid.markers[j+half].scale.z = .2;
 
     // Set the color. a controls the transparency 
     grid.markers[j].color.r = 1.0f;
     grid.markers[j].color.g = 1.0f;
     grid.markers[j].color.b = 1.0f;
     grid.markers[j].color.a = .3;
     grid.markers[j+half].color.r = 1.0f;
     grid.markers[j+half].color.g = 1.0f;
     grid.markers[j+half].color.b = 1.0f;
     grid.markers[j+half].color.a = .3;
     value -=step;
        }

     grid.markers[size-2].header.frame_id = "/cortex";
     grid.markers[size-2].header.stamp = ros::Time::now();
     grid.markers[size-2].ns = "grid"; 
     grid.markers[size-2].id = size-2;
     grid.markers[size-2].type = visualization_msgs::Marker::TEXT_VIEW_FACING; ;
     grid.markers[size-2].text = "X (m)";
     grid.markers[size-2].action = visualization_msgs::Marker::ADD;
     grid.markers[size-2].pose.position.x = 0;
     grid.markers[size-2].pose.position.y = 3;
     grid.markers[size-2].pose.position.z = 0;
     grid.markers[size-2].scale.z = .2;
     grid.markers[size-2].color.r = 1.0f;
     grid.markers[size-2].color.g = 1.0f;
     grid.markers[size-2].color.b = 1.0f;
     grid.markers[size-2].color.a = .3;

     grid.markers[size-1].header.frame_id = "/cortex";
     grid.markers[size-1].header.stamp = ros::Time::now();
     grid.markers[size-1].ns = "grid"; 
     grid.markers[size-1].id = size-1;
     grid.markers[size-1].type = visualization_msgs::Marker::TEXT_VIEW_FACING; ;
     grid.markers[size-1].text = "Y (m)";
     grid.markers[size-1].action = visualization_msgs::Marker::ADD;
     grid.markers[size-1].pose.position.x = 3;
     grid.markers[size-1].pose.position.y = 0;
     grid.markers[size-1].pose.position.z = 0;
     grid.markers[size-1].scale.z = .2;
     grid.markers[size-1].color.r = 1.0f;
     grid.markers[size-1].color.g = 1.0f;
     grid.markers[size-1].color.b = 1.0f;
     grid.markers[size-1].color.a = .3;
     grid_pub.publish(grid);
}

/*====================
 *      Draw Net
 *====================*/

void drawNet()
{
 
     visualization_msgs::MarkerArray net;
     net.markers.resize(4);
    float x[4] = {2.492954,0,-2.492954,0};
    float y[4] = {0,2.05178,0,-2.05178};
    float z[4] = {1.5,1.5,1.5,1.5};
    float sx[4] = {.01, 4.1*10/8,.01 ,4.1*10/8};
    float sy[4] = {4.1,.01,4.1,.01};
    
     //Loop through all objects
     for (int j = 0; j<4; j++)
     {
     net.markers[j].header.frame_id = "/cortex";
     net.markers[j].header.stamp = ros::Time::now();
     net.markers[j].ns = "Net"; 
     net.markers[j].id = j;
 
     // Set the marker type. Simple shapes are CUBE, SPHERE, ARROW, and CYLINDER
     net.markers[j].type = visualization_msgs::Marker::CUBE; 
 
     // Set the marker action.  Options are ADD and DELETE
     net.markers[j].action = visualization_msgs::Marker::ADD;
 
     // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
     // We only need Position for the netting
     net.markers[j].pose.position.x = x[j];
     net.markers[j].pose.position.y = y[j];
     net.markers[j].pose.position.z = z[j];
     net.markers[j].scale.x = sx[j];
     net.markers[j].scale.y = sy[j];
     net.markers[j].scale.z = 3;
 
     // Set the color. a controls the transparency 
     net.markers[j].color.r = 1.0f;
     net.markers[j].color.g = 1.0f;
     net.markers[j].color.b = 1.0f;
     net.markers[j].color.a = .1;
        }
     net_pub.publish(net);
}

/*================
 *      Main
 *================*/

 int main( int argc, char** argv )
 {
   ros::init(argc, argv, "draw_risc_maap");
   ros::NodeHandle n;
   listener = new (tf::TransformListener);

/*================
 *   Publishers
 *================*/

   net_pub = n.advertise<visualization_msgs::MarkerArray>("net", 100);
   Cameras_pub = n.advertise<visualization_msgs::MarkerArray>("Cameras", 100);
   grid_pub = n.advertise<visualization_msgs::MarkerArray>("Grid", 100);

ros::Rate loop_rate(30);
   while (ros::ok())
{
   drawNet();
   drawCameras();
   drawGrid();
   ros::spinOnce();
   loop_rate.sleep();
}
 }
