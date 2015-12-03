/*======================================================
	Created by:     D. Spencer Maughan
	Last updated:   March 2015
	File name:      stream.cpp
	Organization:   RISC Lab, Utah State University
    Notes:
           This file is meant to set up communication 
    over ethernet with Cortex Version 5. Cortex is a
    software product of Motion Analysis and relies on
    feedback from Infrared Cameras to supply relative
    position estimates in a marker array. This file
    publishes this data as a ros message name
    "/mocap_data".
 ======================================================*/

/*==================
   Main ROS Library
  ==================*/
#include "ros/ros.h"
/*================
   Cortex Library
  ================*/
#include "cortex_ros/cortex.h"
/*==========================
   Marker Message Libraries
  ==========================*/
#include <risc_msgs/Mocap_marker.h>
#include <risc_msgs/Mocap_markers.h>
#include <risc_msgs/Mocap_data.h>

/*=====================
    Global Variables
  =====================*/

int bodies; // number of bodies
ros::Publisher pub; // publisher
risc_msgs::Mocap_data p; // Message Variable
float conversion = 0.001; // Converts from millimeters to meters

void MyErrorMsgHandler(int iLevel, const char *szMsg)
{
  const char *szLevel = NULL;

  if (iLevel == VL_Debug) {
    szLevel = "Debug";
  } else if (iLevel == VL_Info) {
    szLevel = "Info";
  } else if (iLevel == VL_Warning) {
    szLevel = "Warning";
  } else if (iLevel == VL_Error) {
    szLevel = "Error";
  }

  ROS_INFO("    %s: %s\n", szLevel, szMsg);
}

/*========================
     Datahandler Function
  ========================*/

void Datahandler(sFrameOfData* frame)
{
bodies = frame->nBodies; // Get Bodies
// Build Header
p.header.seq = frame->iFrame;
p.header.stamp =  ros::Time::now() - ros::Duration(frame->fDelay);
p.header.frame_id = "/cortex";
p.Obj.resize(bodies); // Set array size

/*=====================================
   Variables For marker Objects
  =====================================*/

// loop over all bodies and markers
for (int z = 0; z < bodies; z++)
{
    int markers = frame->BodyData[z].nMarkers;
    p.Obj[z].marker.resize(markers);
    p.Obj[z].name = frame->BodyData[z].szName;
    p.Obj[z].residual = frame->BodyData[z].fAvgMarkerResidual;
    for (int i = 0; i<markers; i++)
{
         p.Obj[z].marker[i].x = conversion*frame->BodyData[z].Markers[i][0];
         p.Obj[z].marker[i].y = conversion*frame->BodyData[z].Markers[i][1];
         p.Obj[z].marker[i].z = conversion*frame->BodyData[z].Markers[i][2];
}
}
pub.publish(p);
}



int main(int argc, char **argv)
{

/*=====================
    Initialize Node
  =====================*/


  ros::init(argc, argv, "MocapStates");
  ros::NodeHandle n;

/*=====================
    Initialize Cortex
  =====================*/

  int retval = RC_Okay;
  unsigned char SDK_Version[4];
  sBodyDefs* pBodyDefs = NULL;
  
  
Cortex_SetVerbosityLevel(VL_Info);
Cortex_SetErrorMsgHandlerFunc(MyErrorMsgHandler);
Cortex_GetSdkVersion(SDK_Version);
   ROS_INFO("Using SDK Version %d.%d.%d\n", SDK_Version[1], SDK_Version[2],
         SDK_Version[3]);
  

ROS_INFO("****** Cortex_Initialize ******\n");
  if (argc == 1) {
    retval = Cortex_Initialize("", NULL);
  } else if (argc == 2) {
    retval = Cortex_Initialize(argv[1], NULL);
  } else if (argc == 3) {
    retval = Cortex_Initialize(argv[1], argv[2]);
  }

if (retval != RC_Okay) {
     ROS_INFO("Error: Unable to initialize ethernet communication\n");
    retval = Cortex_Exit();
    return 1;
  }

ROS_INFO("****** Cortex_GetBodyDefs ******\n");
  pBodyDefs = Cortex_GetBodyDefs();

  if (pBodyDefs == NULL) {
    ROS_INFO("Failed to get body defs\n");
  } else {
    ROS_INFO("Got body defs\n");
    Cortex_FreeBodyDefs(pBodyDefs);
    pBodyDefs = NULL;
  }

void *pResponse;
  int nBytes;
  retval = Cortex_Request("GetContextFrameRate", &pResponse, &nBytes);
  if (retval != RC_Okay)
    ROS_INFO("ERROR, GetContextFrameRate\n");

float *contextFrameRate = (float*) pResponse;

ROS_INFO("ContextFrameRate = %3.1f Hz\n", *contextFrameRate);

float Rate = *contextFrameRate;

ROS_INFO("*** Starting live mode ***\n");
  retval = Cortex_Request("LiveMode", &pResponse, &nBytes);

/*=====================
    Set up Publisher
  =====================*/

pub = n.advertise<risc_msgs::Mocap_data>("/mocap_data", 10);

/*=====================
    Data Request Loop
  =====================*/

ros::Rate loop_rate(Rate);
while (ros::ok())
{
sFrameOfData* frame = Cortex_GetCurrentFrame(); // Request Current Frame
Datahandler(frame); // Pull and Calculate Data you are interested in.
loop_rate.sleep();
}

/*=================
    Close Cortex
  =================*/

retval = Cortex_Request("Pause", &pResponse, &nBytes);
ROS_INFO("*** Paused live mode ***\n");

ROS_INFO("****** Cortex_Exit ******\n");
  retval = Cortex_Exit();

  return 0;
}

