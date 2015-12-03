
/*======================================================
	Created by:  	D. Spencer Maughan
	Last updated: 	July 2014
	File name: 		Quad_tracker.cpp
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
int sub_room = 0;
int size_limit = 117;
ros::Publisher pub; 
risc_msgs::Risc_rois roi;
static const char WINDOW1[] = "Quads";
static const char WINDOW2[] = "green red border";
static const char WINDOW3[] = "red pink border";
//static const char WINDOW4[] = "pink yellow border";
//static const char WINDOW5[] = "yellow green border";
    
    /*=========================================
     *    Color Tuning Values for Paints
     ========================================*/

	int hsvcolors[] = {0,0,255,0,255,0,1,1};
	int yellow[] = {40,20,255,80,255,140,1,1};
	int pink[] = {178,164,230,50,250,80,1,1};
	int pink_red_color[] = {175,170,208,120,140,75,1,1};
	int orange[] = {19,2,255,130,255,185,1,1};

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

cv::Mat createbinMask(cv::Mat hsvframe, int bin1, int bin2 )
{
// Blue bin 14-19
// Green bin 8-12
// Pink bin 24
// Yellow bin 5
// Orange bin 2-3
// Red bin 1

cv::Vec3b color[27] = {cv::Vec3b( 0, 7, 245),cv::Vec3b(2,39,255),cv::Vec3b( 7,71,255),cv::Vec3b( 0,128,241),cv::Vec3b( 13,191,255),cv::Vec3b( 60,254,254),cv::Vec3b( 50,253,202),cv::Vec3b( 0,255,129),cv::Vec3b( 0,241,58),cv::Vec3b( 60,254,53),cv::Vec3b( 57,255,0),cv::Vec3b( 129,254,1),cv::Vec3b( 201,254,57),cv::Vec3b( 245,246,0),cv::Vec3b( 251,186,4),cv::Vec3b( 252,136,0),cv::Vec3b( 249,65,0),cv::Vec3b( 252,1,0),cv::Vec3b( 255,1,61),cv::Vec3b( 251,0,130),cv::Vec3b( 255,3,190),cv::Vec3b( 249,0,255),cv::Vec3b( 196,0,255),cv::Vec3b( 121,0,252),cv::Vec3b( 67,8,246),cv::Vec3b(0,0,0),cv::Vec3b(255,255,255) };
int w = hsvframe.cols;int  h = hsvframe.rows;
	cv::Mat color_thresh(cv::Size(w,h),CV_8U);

if (bin1==bin2){

cv::Scalar C = cv::Scalar(color[bin1-1].val[0],color[bin1-1].val[1],color[bin1-1].val[2]);
	cv::inRange(hsvframe,C,C,color_thresh);
}

if (bin1!=bin2){
	cv::Mat color_thresh1(cv::Size(w,h),CV_8U);
for(int i = bin1; i<bin2+1; i++)
{
cv::Scalar C = cv::Scalar(color[i-1].val[0],color[i-1].val[1],color[i-1].val[2]);
	cv::inRange(hsvframe,C,C,color_thresh1);
        color_thresh = combineMasks(color_thresh1,color_thresh);
}}
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
cv::Vec3b color[27] = {cv::Vec3b( 0, 7, 245),cv::Vec3b(2,39,255),cv::Vec3b( 7,71,255),cv::Vec3b( 0,128,241),cv::Vec3b( 13,191,255),cv::Vec3b( 60,254,254),cv::Vec3b( 50,253,202),cv::Vec3b( 0,255,129),cv::Vec3b( 0,241,58),cv::Vec3b( 60,254,53),cv::Vec3b( 57,255,0),cv::Vec3b( 129,254,1),cv::Vec3b( 201,254,57),cv::Vec3b( 245,246,0),cv::Vec3b( 251,186,4),cv::Vec3b( 252,136,0),cv::Vec3b( 249,65,0),cv::Vec3b( 252,1,0),cv::Vec3b( 255,1,61),cv::Vec3b( 251,0,130),cv::Vec3b( 255,3,190),cv::Vec3b( 249,0,255),cv::Vec3b( 196,0,255),cv::Vec3b( 121,0,252),cv::Vec3b( 67,8,246),cv::Vec3b(0,0,0),cv::Vec3b(255,255,255) };

int room[] = {25,7,94,43,193,0,1,1};
cv::Mat m(cv::Size(HSV.cols,HSV.rows),CV_8UC3);
m.setTo(cv::Scalar(0,0,0));
for (int u=kernal; u<HSV.rows; u++)
{
	for (int v=kernal; v<HSV.cols; v++)
        {
               float C1_m = 0;
               int k = 0;
               int black = 0;
               int white = 0;
               for (int i = 0; i<kernal; i++)
               {
                     for (int j = 0; j<kernal; j++)
                     {
	        	int H = (HSV.at<cv::Vec3b>(u-i,v-j)[0]);
	        	int S = (HSV.at<cv::Vec3b>(u-i,v-j)[1]);
	        	int V = (HSV.at<cv::Vec3b>(u-i,v-j)[2]);
// Account for cyclic Hue
			if (S<=C[2] && S>=C[3] && V<=C[4] && V>=C[5])
			{ C1_m = C1_m + (H*24/180);
                          k++;		}

			if (V>=C[4]){white++;}
			if (V<=C[5]){black++;}
                        if (H>=room[1] && H<=room[0] && S<=room[2] && S>=room[3] && V<=room[4] && V>=room[5] && sub_room)
{white = white+2; black--;}
                     }
               }
//Paint in colors 
if(k < black){ m.at<cv::Vec3b>(u,v)[0] =  0;
            m.at<cv::Vec3b>(u,v)[1] =  0;
            m.at<cv::Vec3b>(u,v)[2] =  0;}
if(k < white){ m.at<cv::Vec3b>(u,v)[0] =  255;
            m.at<cv::Vec3b>(u,v)[1] =  255;
            m.at<cv::Vec3b>(u,v)[2] =  255;}

if(k != 0 && k>white && k>black){ if(cvRound(C1_m/k)>23){k=1;C1_m=23;}
            m.at<cv::Vec3b>(u,v)[0] = color[cvRound(C1_m/k)].val[0];
            m.at<cv::Vec3b>(u,v)[1] = color[cvRound(C1_m/k)].val[1];
            m.at<cv::Vec3b>(u,v)[2] = color[cvRound(C1_m/k)].val[2];}
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
        cv::createTrackbar("High Sat",WINDOW2, &hsvcolors[2], 255);
        cv::createTrackbar("Low Sat",WINDOW2, &hsvcolors[3], 255);
        cv::createTrackbar("High Val",WINDOW2, &hsvcolors[4], 255);
        cv::createTrackbar("Low Val",WINDOW2, &hsvcolors[5], 255);
        cv::createTrackbar("subtract some room noise",WINDOW2, &sub_room, 1);
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


// Blue bin 14-19
// Green bin 8-12
// Pink bin 24
// Yellow bin 5
// Orange bin 2-3
// Red bin 1

        cv::Mat green_red = createbinMask(Colors, 24, 24 );
        cv::imshow(WINDOW3,green_red); 
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
      //  cv::namedWindow(WINDOW4, CV_WINDOW_AUTOSIZE);
      //  cv::namedWindow(WINDOW5, CV_WINDOW_AUTOSIZE);
                                                                                            
        //image_transport::Subscriber sub = it.subscribe("ardrone/image_rect_color", 1, imageCallback);
        image_transport::Subscriber sub = it.subscribe("ardrone/image_raw", 1, imageCallback);
        cv::destroyAllWindows();                          
        ros::spin();
    
        ROS_INFO("ros_opencv::main.cpp::No error.");
  
 }
