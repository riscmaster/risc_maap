
/*======================================================
	Created by:  	D. Spencer Maughan
	Last updated: 	July 2014
	File name: 	quad_track_modi.cpp
	Organization:	RISC Lab, Utah State University
 ======================================================*/

#include <ros/ros.h>
#include "risc_msgs/Risc_rois.h"
#include "risc_msgs/Risc_roi.h"
#include "ardrone_autonomy/Navdata.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <cv.h>
#include <vector>
#include <string>
#include <algorithm>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>
#define PI 3.14159265

using namespace std;
namespace enc = sensor_msgs::image_encodings;

    /*=================
     *    Globals
     ================*/

int kernal = 1;
int sub_room = 1;
int size_limit = 50;
int threshold = 0;
ros::Publisher pub; 
risc_msgs::Risc_rois roi;
static const char WINDOW1[] = "raw_image";
static const char WINDOW2[] = "Color_segmentation";
static const char WINDOW3[] = "Blobs";
static const char WINDOW4[] = "Quads";
    
    /*=========================================
     *    Color Tuning Values for Paints
     ========================================*/

	int green[] = {80,40,225,45,70,0,1,1};
	int blue[] = {140,100,255,95,95,5,1,2};
	int red[] = {3,175,255,140,90,35,1,1};
	int hsvcolors[] = {0,0,255,0,255,0,1,1};
	int yellow[] = {40,20,255,150,255,80,1,1};
	int pink[] = {178,164,255,125,220,50,1,1};
	int pink_red_color[] = {175,170,208,120,140,75,1,1};
	int orange[] = {19,2,255,180,255,100,1,1};
    
    /*=========================================
     *   Function to Compare Contour Areas
     ========================================*/

bool compareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 ) {
double i = std::fabs( cv::contourArea(cv::Mat(contour1)) );
double j = std::fabs( cv::contourArea(cv::Mat(contour2)) );
return ( i < j );
}

    /*====================================
     *   Get Index of Largest Element
     =====================================*/

int getIndexOfLargestElement(int arr[], int size) {
    int largestIndex = 0;
    for (int index = largestIndex; index < size; index++) {
        if (arr[largestIndex] < arr[index]) {
            largestIndex = index;
        }
    }
    return largestIndex;
}
    /*======================================
     *     Replace Max_Element Function
     =====================================*/

float Max(float Val[3])
{
    float V;
    if ((Val[0] > Val[1]) && (Val[0] > Val[2])){ V = Val[0];} 
    if ((Val[0] > Val[1]) && (Val[0] == Val[2])){ V = Val[0];} 
    if ((Val[0] == Val[1]) && (Val[0] > Val[2])){ V = Val[0];} 
    if ((Val[1] > Val[0]) && (Val[1] > Val[2])){ V = Val[1];} 
    if ((Val[1] > Val[0]) && (Val[1] == Val[2])){ V = Val[1];} 
    if ((Val[1] == Val[0]) && (Val[1] > Val[2])){ V = Val[1];} 
    if ((Val[2] > Val[0]) && (Val[2] > Val[1])){ V = Val[2];} 
    if ((Val[2] > Val[0]) && (Val[2] == Val[1])){ V = Val[2];} 
    if ((Val[2] == Val[0]) && (Val[2] > Val[1])){ V = Val[2];} 
    return V;
}

    /*======================================
     *     Replace Min_Element Function
     =====================================*/
     
float Min(float Val[3])
{
    float V;
    if ((Val[0] < Val[1]) && (Val[0] < Val[2])){ V = Val[0];} 
    if ((Val[0] < Val[1]) && (Val[0] == Val[2])){ V = Val[0];} 
    if ((Val[0] == Val[1]) && (Val[0] < Val[2])){ V = Val[0];} 
    if ((Val[1] < Val[0]) && (Val[1] < Val[2])){ V = Val[1];} 
    if ((Val[1] < Val[0]) && (Val[1] == Val[2])){ V = Val[1];} 
    if ((Val[1] == Val[0]) && (Val[1] < Val[2])){ V = Val[1];} 
    if ((Val[2] < Val[0]) && (Val[2] < Val[1])){ V = Val[2];} 
    if ((Val[2] < Val[0]) && (Val[2] == Val[1])){ V = Val[2];} 
    if ((Val[2] == Val[0]) && (Val[2] < Val[1])){ V = Val[2];} 
    return V;
}

    /*======================================
     *   Replace Opencv cvtColor Function
     =====================================*/
     
cv::Mat BGR2HSV(cv::Mat BGR)
{
    cv::Mat HSV(cv::Size(BGR.cols,BGR.rows),CV_8UC3);

    for(int i=0; i<BGR.rows; i++)
    {
            for(int j=0; j<BGR.cols; j++) 
            {
                float Val[3];
                float H,S,V;

                Val[0]= (BGR.at<cv::Vec3b>(i,j)[0]);
                Val[1]= (BGR.at<cv::Vec3b>(i,j)[1]);
                Val[2]= (BGR.at<cv::Vec3b>(i,j)[2]);

                Val[0]= Val[0]/255;
                Val[1]= Val[1]/255;
                Val[2]= Val[2]/255;


                // Value

                V = Max(Val);
                float delta = V - Min(Val);
          
                // Saturation
                if (V != 0)
                {
                S = delta/V;
                
                //Hue
                if (V == Val[0])
                {
                    H = 240 + (60*(Val[2]-Val[1]))/delta;
                }
                
                if (V == Val[1])
                {
                    H = 120 + (60*(Val[0]-Val[2]))/delta;
                }
                
                if (V == Val[2])
                { 
                    H = (60*(Val[1]-Val[0]))/delta;
                }
                
                if (H<0){H = H + 360;}}
                
                if (V == 0){V = 0;} 
                
                if (V > 1){V = 1;}

                HSV.at<cv::Vec3b>(i,j)[0] = H/2;
                HSV.at<cv::Vec3b>(i,j)[1] = 255*S;
                HSV.at<cv::Vec3b>(i,j)[2] = V*255;
            }
    }
    return HSV;
}


/*=====================================
 *     Allow combination of Masks
 *=====================================*/

cv::Mat combineMasks(cv::Mat mask1, cv::Mat mask2)
{
    cv::Mat new_Mask(cv::Size(mask1.cols,mask1.rows),CV_8U);
    for (int i=0; i<mask1.rows; i++)
    {
        for(int j=0; j<mask1.cols; j++)
        {
            new_Mask.at<uchar>(i,j) = mask1.at<uchar>(i,j)|mask2.at<uchar>(i,j);
        }
    }
    return new_Mask;
}

    /*=================================
     *   Find Boundaries of interest 
     ==================================*/

cv::Mat findBoundaries(cv::Mat frame,std::string name,int kern)
{ 
cv::Vec3b color[6] = {cv::Vec3b( 67,8,246),cv::Vec3b(255,1,61),cv::Vec3b(57,255,0),cv::Vec3b(13,191,255),cv::Vec3b(7,71,255),cv::Vec3b(0,7,245) };
int w = frame.cols; int h = frame.rows;
cv::Mat new_frame(cv::Size(w,h),CV_8UC3);
new_frame.setTo(cv::Scalar(0,0,0));
int baby_bird[4][2] = {{5,0},{0,3},{3,2},{2,5}};
int fluffy[4][2] = {{4,1},{1,3},{3,0},{0,4}};
int quad[4][2];

if (name == "Fluffy"){for (int i = 0; i<4; i++){
for (int j = 0; j<2; j++){quad[i][j] = fluffy[i][j];}}}

if (name == "Baby_bird"){for (int i = 0; i<4; i++){
for (int j = 0; j<2; j++){quad[i][j] = baby_bird[i][j];}}}

for (int border = 0; border<4; border++){
int m = quad[border][0];
int n = quad[border][1];
for (int u=kern; u<h; u++)
{
        for (int v=kern; v<w; v++)
        {
              int C1 = 0;
              int C2 = 0;
              for (int i = 0; i<kern; i++)
               {
                     for (int j = 0; j<kern; j++)
                     {
                        int B = (frame.at<cv::Vec3b>(u-i,v-j)[0]);
//ROS_INFO("Blue = %i",B);
                        int G = (frame.at<cv::Vec3b>(u-i,v-j)[1]);
                        int R = (frame.at<cv::Vec3b>(u-i,v-j)[2]);
                        // First Color
                        if (B == color[m].val[0] && G == color[m].val[1] && R == color[m].val[2]){C1++;} 
                        // Second Color
                        if (B == color[n].val[0] && G == color[n].val[1] && R == color[n].val[2]){C2++;} 
                     }
               }
             if( C2*C1/(kern*kern)>=threshold*.01)
               {new_frame.at<cv::Vec3b>(u,v)[0]=frame.at<cv::Vec3b>(u,v)[0];
                new_frame.at<cv::Vec3b>(u,v)[1]=frame.at<cv::Vec3b>(u,v)[1];
                new_frame.at<cv::Vec3b>(u,v)[2]=frame.at<cv::Vec3b>(u,v)[2];}
        }
}}

return new_frame;
}


    /*=================
     *   Blob it up
     ==================*/

cv::Mat colors2Blobs(cv::Mat frame)
{ 
cv::Vec3b color[6] = {cv::Vec3b( 67,8,246),cv::Vec3b(255,1,61),cv::Vec3b(57,255,0),cv::Vec3b(13,191,255),cv::Vec3b(7,71,255),cv::Vec3b(0,7,245) };
int w = frame.cols; int h = frame.rows;
cv::Mat new_frame(cv::Size(w,h),CV_8UC3);
new_frame.setTo(cv::Scalar(0,0,0));

for(int i = 0; i<6; i++)
{
//Create a mask
	cv::Mat color_thresh(cv::Size(w,h),CV_8U);
        color_thresh.setTo(cv::Scalar(0,0,0));
        cv::Scalar C = cv::Scalar(color[i].val[0],color[i].val[1],color[i].val[2]);
	cv::inRange(frame,C,C,color_thresh);
//erode dilate and blur to reduce noise
	cv::Mat color_element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 3, 3), cv::Point(1, 1));
if(i!=1){
for(int z = 0; z<threshold; z++)
{
        cv::dilate(color_thresh,color_thresh,color_element);
	cv::erode(color_thresh,color_thresh,color_element);
	cv::GaussianBlur(color_thresh,color_thresh,cv::Size(3,3),2,2);
}}
// Find contours
        std::vector< std::vector<cv::Point> > contours,ellipse;
        cv::findContours(color_thresh, contours,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
// Sort by Size
//        std::sort(contours.begin(), contours.end(), compareContourAreas);
// loop through from largest to smallest and get the ones above the set limit
        for  ( int j = contours.size()-1; j> -1; j-- )
             { 
             if (cv::contourArea(contours[j])>size_limit)
                {
cv::drawContours(new_frame, contours, j, C, CV_FILLED);
                }
             }
}
return new_frame;
}



        /*=============================
         *   Find Colors of Interest
         =============================*/
	
cv::Mat HSVcolors(cv::Mat HSV,int kernal,int C[8],bool sub_room)
{
cv::Vec3b color[7] = {cv::Vec3b( 67,8,246),cv::Vec3b(255,1,61),cv::Vec3b(57,255,0),cv::Vec3b(13,191,255),cv::Vec3b(7,71,255),cv::Vec3b(0,7,245),cv::Vec3b(0,0,0) };

int room[] = {25,7,94,43,193,0,1,1};
cv::Mat m(cv::Size(HSV.cols,HSV.rows),CV_8UC3);
m.setTo(cv::Scalar(0,0,0));
for (int u=kernal; u<HSV.rows; u++)
{
	for (int v=kernal; v<HSV.cols; v++)
        {
               int white = 0;
               int P = 0;
               int B = 0;
               int G = 0;
               int Y = 0;
               int O = 0;
               int R = 0;
               for (int i = 0; i<kernal; i++)
               {
                     for (int j = 0; j<kernal; j++)
                     {
	        	int H = (HSV.at<cv::Vec3b>(u-i,v-j)[0]);
	        	int S = (HSV.at<cv::Vec3b>(u-i,v-j)[1]);
	        	int V = (HSV.at<cv::Vec3b>(u-i,v-j)[2]);
// Account for cyclic Hue
// Pink
			if (H<pink[0] && H>pink[1] && S<=pink[2] && S>=pink[3] && V<=pink[4] && V>=pink[5])
                            {P++;}

                        // blue
			if (H<blue[0] && H>blue[1] && S<=blue[2] && S>=blue[3] && V<=blue[4] && V>=blue[5])
                            {B++;}

                        // green
			if (H<green[0] && H>green[1] && S<=green[2] && S>=green[3] && V<=green[4] && V>=green[5])
                            {G++;}
                        // yellow
			if (H<yellow[0] && H>yellow[1] && S<=yellow[2] && S>=yellow[3] && V<=yellow[4] && V>=yellow[5])
                            {Y++;}

                        // orange
			if (H<orange[0] && H>orange[1] && S<=orange[2] && S>=orange[3] && V<=orange[4] && V>=orange[5])
                            {O++;}

                        // red
			if ((H<red[0] || H>red[1]) && S<=red[2] && S>=red[3] && V<=red[4] && V>=red[5])
                            {R++;}


			if (V>=C[4]){white++;}
			if (V<=C[5]){white++;}
                        if (H>=room[1] && H<=room[0] && S<=room[2] && S>=room[3] && V<=room[4] && V>=room[5] && sub_room)
{white = white+2;}
                     }
               }
//Paint colors in
int compare[7] = {P,B,G,Y,O,R,white};
int q = getIndexOfLargestElement(compare, 7);
if(P+B+G+Y+O+R+white!=0){
m.at<cv::Vec3b>(u,v)[0] = color[q].val[0];
m.at<cv::Vec3b>(u,v)[1] = color[q].val[1];
m.at<cv::Vec3b>(u,v)[2] = color[q].val[2];}
       }
}   
return m;
}

    /*=======================
     *  Find Quad in Image
     ======================*/

cv::RotatedRect findQuad(cv::Mat Blobs,std::string name)
{
// Get borders
        cv::Mat borders = findBoundaries(Blobs,name, 5);
        std::vector< std::vector<cv::Point> > contours;
                cv::RotatedRect rect;
                //rect = cv::minAreaRect(contours[j]);

//                cv::Mat mask(cv::Size(w,h),CV_8U);
//                mask.setTo(cv::Scalar(0,0,0));
//                cv::ellipse(mask, rect, cv::Scalar(255,255,255),-1,8);
//                cv::findContours(mask, ellipse,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));
//                float E = cv::matchShapes(contours[i],ellipse[0],1,0);
//if (E<.001*threshold){
//cv::ellipse(new_frame, rect, C,-1,8);
//// save center and area of contour for later
//                int place = j+1-contours.size();
//                center[i][place] = rect.center.x;
//                center[i][place+1] = rect.center.y;
//                center[i][place+2] = cv::contourArea(contours[j]);}




}

    /*=========================================
     *  Function Called for Every New Image
     ========================================*/

void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    /*===================
     *    CV Bridge
     ==================*/

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("ros_opencv::edge.cpp::cv_bridge exception: %s", e.what());
        return;
    }
    

    /*=============================
     *   Set up image variables
     ============================*/
	cv::Mat hsvframe = BGR2HSV(cv_ptr->image);
        cv::createTrackbar("Kernal",WINDOW2, &kernal, 10);
        cv::Mat Colors = HSVcolors(hsvframe,kernal,hsvcolors,sub_room);
        cv::Mat Blobs = colors2Blobs(Colors);
        Blobs = colors2Blobs(Blobs);
 //       cv::rotatedRect fluffy = findQuad(Blobs,"Fluffy");
  //      cv::rotatedRect baby_bird = findQuad(Blobs,"Baby_bird");
        cv::createTrackbar("threshold",WINDOW3,&threshold,1000);
        cv::imshow(WINDOW3,Blobs); 
        //cv::imshow(WINDOW4,borders); 
	cv::waitKey(3);                                                   
        cv::imshow(WINDOW2,Colors); 
	cv::imshow(WINDOW1,cv_ptr->image); 
	cv::waitKey(3);                                                   
  }                                                                                            
int main(int argc, char **argv)                                                             
                                                                                            
{
        ros::init(argc, argv, "quad_finder");
        ros::NodeHandle nh;
        //pub = nh.advertise<risc_msgs::Risc_rois>("ROIs", 1000);

        image_transport::ImageTransport it(nh);
                                                                                            
        cv::namedWindow(WINDOW1, CV_WINDOW_AUTOSIZE);
        cv::namedWindow(WINDOW2, CV_WINDOW_AUTOSIZE);
        cv::namedWindow(WINDOW3, CV_WINDOW_AUTOSIZE);
        cv::namedWindow(WINDOW4, CV_WINDOW_AUTOSIZE);
                                                                                            
        //image_transport::Subscriber sub = it.subscribe("ardrone/image_rect_color", 1, imageCallback);
        image_transport::Subscriber sub = it.subscribe("ardrone/image_raw", 1, imageCallback);
        cv::destroyAllWindows();                          
        ros::spin();
    
        ROS_INFO("ros_opencv::main.cpp::No error.");
  
 }
