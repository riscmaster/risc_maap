
/*======================================================
	Created by:  	D. Spencer Maughan
	Last updated: 	July 2014
	File name: 	Quad_tracker.cpp
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

int kernal = 4;
int sub_room = 1;
int green_radius = 30;
int red_radius = 4;
int yellow_radius = 18;
int pink_radius = 7;
int pink_red_radius = 6;
int size_limit = 117;
ros::Publisher pub; 
risc_msgs::Risc_rois roi;
static const char WINDOW1[] = "Quads";
static const char WINDOW2[] = "Colors";
static const char WINDOW3[] = "red pink border";
static const char WINDOW4[] = "pink yellow border";
static const char WINDOW5[] = "yellow green border";
    
    /*=========================================
     *    Color Tuning Values for Paints
     ========================================*/

        int bgr_green[3] = {27,38,27};
        int bgr_pink[3] = {155,141,237};
        int bgr_red[3] = {21,28,148};
        int bgr_yellow[3] = {73,207,234};

	int green[] = {80,40,225,45,70,0,1,1};
	int blue[] = {140,100,255,95,95,5,1,2};
	int red[] = {3,175,255,140,90,35,1,1};
	int hsvcolors[] = {0,0,255,0,255,0,1,1};
	int yellow[] = {40,20,255,150,255,80,1,1};
	int pink[] = {178,164,255,125,220,50,1,1};
	int pink_red_color[] = {175,170,208,120,140,75,1,1};
	int orange[] = {19,2,255,180,255,100,1,1};
    
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


    /*===========================
     *   Create Color Mask
     ============================*/

cv::Mat createColorMask(cv::Mat hsvframe, int color[8], int w, int h)
{

	cv::Mat color_thresh(cv::Size(w,h),CV_8U);
	cv::Mat color_element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(2*color[6] +1, 2*color[6] + 1), cv::Point(color[6], color[6]));

if (color[0]>color[1])
{
	cv::inRange(hsvframe,cv::Scalar(color[1],color[3],color[5]),cv::Scalar(color[0],color[2],color[4]),color_thresh);
	cv::dilate(color_thresh,color_thresh,color_element);
	cv::erode(color_thresh,color_thresh,color_element);
	cv::GaussianBlur(color_thresh,color_thresh,cv::Size(2*color[7]+1,2*color[7]+1),2,2);
}

if (color[0]<color[1])
{
	cv::Mat color_thresh2(cv::Size(w,h),CV_8U);
	cv::inRange(hsvframe,cv::Scalar(color[1],color[3],color[5]),cv::Scalar(180,color[2],color[4]),color_thresh);
	cv::inRange(hsvframe,cv::Scalar(0,color[3],color[5]),cv::Scalar(color[0],color[2],color[4]),color_thresh2);
    color_thresh = combineMasks(color_thresh,color_thresh2);
	cv::dilate(color_thresh,color_thresh,color_element);
	cv::erode(color_thresh,color_thresh,color_element);
	cv::GaussianBlur(color_thresh,color_thresh,cv::Size(2*color[7]+1,2*color[7]+1),2,2);
}

return color_thresh;
}

        /*========================
         *  Find BGR boundaries
         ========================*/
	
cv::Mat BGRboundary(int C1[3],int C2[3],cv::Mat BGR,int kernal,int radius1, int radius2)
{
cv::Mat m(cv::Size(BGR.cols,BGR.rows),CV_8UC1);
m.setTo(cv::Scalar(0,0,0));
for (int u=kernal; u<BGR.rows; u++)
{
	for (int v=kernal; v<BGR.cols; v++)
        {
               int C1_m = 0;
               int C2_m = 0;
               for (int i = 0; i<kernal; i++)
               {
                     for (int j = 0; j<kernal; j++)
                     {
	        	int B = (BGR.at<cv::Vec3b>(u-i,v-j)[0]);
	        	int G = (BGR.at<cv::Vec3b>(u-i,v-j)[1]);
	        	int R = (BGR.at<cv::Vec3b>(u-i,v-j)[2]);
                        int dist1 = abs((B-C1[0])*(B-C1[1])+(G-C1[1])*(G-C1[1])+(R-C1[2])*(R-C1[2]));
                        if (dist1<radius1*radius1){C1_m++;}
                        int dist2 = abs((B-C2[0])*(B-C2[1])+(G-C2[1])*(G-C2[1])+(R-C2[2])*(R-C2[2]));
                        if (dist2<radius2*radius2){C2_m++;}
                     }
               }

	       m.at<uchar>(u,v) = floor(C2_m*C1_m*255/(kernal*kernal));
        }
}   


return m;
}

        /*========================
         *  Find HSV Colors
         ========================*/
	
cv::Mat HSVcolors(cv::Mat HSV,int kernal,int C[8],bool sub_room)
{
cv::Vec3b color[7] = {cv::Vec3b( 67,8,246),cv::Vec3b(255,1,61),cv::Vec3b(57,255,0),cv::Vec3b(13,191,255),cv::Vec3b(7,71,255),cv::Vec3b(0,7,245),cv::Vec3b(255,255,255) };

int room[] = {25,7,94,43,193,0,1,1};
cv::Mat m(cv::Size(HSV.cols,HSV.rows),CV_8UC3);
m.setTo(cv::Scalar(255,255,255));
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

        /*========================
         *  Find HSV boundaries
         ========================*/
	
cv::Mat HSVboundary(int C1[8],int C2[8],cv::Mat HSV,int kernal,int radius1, int radius2)
{
cv::Mat m(cv::Size(HSV.cols,HSV.rows),CV_8UC1);
m.setTo(cv::Scalar(0,0,0));
for (int u=kernal; u<HSV.rows; u++)
{
float C1_c = (C1[0]+C1[0])/2;
if (C1_c>180){C1_c = C1_c-180;}
float C2_c = (C2[0]+C2[0])/2;
if (C2_c>180){C2_c = C2_c-180;}
	for (int v=kernal; v<HSV.cols; v++)
        {
               int C1_m = 0;
               int C2_m = 0;
               for (int i = 0; i<kernal; i++)
               {
                     for (int j = 0; j<kernal; j++)
                     {
	        	int H = (HSV.at<cv::Vec3b>(u-i,v-j)[0]);
	        	int S = (HSV.at<cv::Vec3b>(u-i,v-j)[1]);
	        	int V = (HSV.at<cv::Vec3b>(u-i,v-j)[2]);
// Account for cyclic Hue
			if (S<C1[2] && S>C1[3] && V<C1[4] && V>C1[5])
			{
                        int dist1a = abs(H-C1_c);
                        int dist1b = abs(H+180-C1_c);
                        int dist1 = radius1;
			if(dist1a<dist1b){dist1=dist1a;}
			if(dist1a>dist1b){dist1=dist1b;}
                        if (dist1<radius1){C1_m++;}
			}

			if (S<C2[2] && S>C2[3] && V<C2[4] && V>C2[5])
			{
                        int dist2a = abs(H-C2_c);
                        int dist2b = abs(H+180-C2_c);
                        int dist2 = radius2;
			if(dist2a<dist2b){dist2=dist2a;}
			if(dist2a>dist2b){dist2=dist2b;}
                        if (dist2<radius2){C2_m++;}
			}

                     }
               }

	       m.at<uchar>(u,v) = floor(C2_m*C1_m*255/(kernal*kernal));
        }
}   
cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(2*C1[6] +1, 2*C1[6] + 1), cv::Point(C1[6], C1[6]));
cv::dilate(m,m,element);
cv::erode(m,m,element);
cv::GaussianBlur(m,m,cv::Size(2*C1[7]+1,2*C1[7]+1),2,2);

element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(2*C2[6] +1, 2*C2[6] + 1), cv::Point(C2[6], C2[6]));
cv::dilate(m,m,element);
cv::erode(m,m,element);
cv::GaussianBlur(m,m,cv::Size(2*C2[7]+1,2*C2[7]+1),2,2);

return m;
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
 //       cv::createTrackbar("Green Radius",WINDOW2, &green_radius, 180);
  //      cv::createTrackbar("Yellow Radius",WINDOW2, &yellow_radius, 180);
   //     cv::createTrackbar("Pink Radius",WINDOW2, &pink_radius, 180);
        //cv::createTrackbar("Red Radius",WINDOW2, &red_radius, 180);
        //cv::createTrackbar("High Sat",WINDOW2, &yellow[2], 255);
        //cv::createTrackbar("Low Sat",WINDOW2, &yellow[3], 255);
        //cv::createTrackbar("High Val",WINDOW2, &yellow[4], 255);
        //cv::createTrackbar("Low Val",WINDOW2, &yellow[5], 255);
        //cv::createTrackbar("subtract some room noise",WINDOW2, &sub_room, 1);
//        cv::createTrackbar("Green High Sat",WINDOW2, &green[2], 255);
 //       cv::createTrackbar("Green Low Sat",WINDOW2,  &green[3], 255);
  //      cv::createTrackbar("Green High Val",WINDOW2, &green[4], 255);
   //     cv::createTrackbar("Green Low Val",WINDOW2,  &green[5], 255);

/*

        cv::Mat green_red = BGRboundary(bgr_green,bgr_red,cv_ptr->image,kernal,green_radius,red_radius);
        cv::imshow(WINDOW2,green_red); 
        cv::Mat pink_red = BGRboundary(bgr_pink,bgr_red,cv_ptr->image,kernal,pink_radius,red_radius);
        cv::imshow(WINDOW3,pink_red); 
        cv::Mat pink_yellow = BGRboundary(bgr_pink,bgr_yellow,cv_ptr->image,kernal,pink_radius,yellow_radius);
        cv::imshow(WINDOW4,pink_yellow); 
        cv::Mat green_yellow = BGRboundary(bgr_green,bgr_yellow,cv_ptr->image,kernal,green_radius,yellow_radius);
        cv::imshow(WINDOW5,green_yellow); 
*/
//        cv::Mat green_red = HSVboundary(green,red,hsvframe,kernal,green_radius,red_radius);
//        cv::imshow(WINDOW2,green_red); 
//        cv::Mat pink_red = HSVboundary(pink_red_color,red,hsvframe,kernal,pink_red_radius,red_radius);
//        cv::imshow(WINDOW3,pink_red); 
//        cv::Mat pink_yellow = HSVboundary(pink,yellow,hsvframe,kernal,pink_radius,yellow_radius);
//        cv::imshow(WINDOW4,pink_yellow); 
//        cv::Mat green_yellow = HSVboundary(green,yellow,hsvframe,kernal,green_radius,yellow_radius);
//        cv::imshow(WINDOW5,green_yellow); 
    //    cv::Mat combine = combineMasks(green_red,pink_red);
     //   combine = combineMasks(combine,pink_yellow);
      //  combine = combineMasks(combine,green_yellow);
       // cv::imshow(WINDOW2,combine); 
	//cv::waitKey(3);                                                   
        cv::Mat Colors = HSVcolors(hsvframe,kernal,hsvcolors,sub_room);
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
        cv::namedWindow(WINDOW5, CV_WINDOW_AUTOSIZE);
                                                                                            
        //image_transport::Subscriber sub = it.subscribe("ardrone/image_rect_color", 1, imageCallback);
        image_transport::Subscriber sub = it.subscribe("ardrone/image_raw", 1, imageCallback);
        cv::destroyAllWindows();                          
        ros::spin();
    
        ROS_INFO("ros_opencv::main.cpp::No error.");
  
 }
