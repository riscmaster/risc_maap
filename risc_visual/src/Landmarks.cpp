
/*======================================================
	Created by:  	D. Spencer Maughan
	Last updated: 	July 2014
	File name: 	Landmarks.cpp
	Organization:	RISC Lab, Utah State University
 ======================================================*/

#include <ros/ros.h>
#include "risc_msgs/Observed_rois.h"
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

int size_int = 117;
int error_threshold_int = 50;
int ring_int = 20;
ros::Publisher pub; 
image_transport::Publisher im;
risc_msgs::Risc_rois roi;
//static const char WINDOW1[] = "Landmarks";
//static const char WINDOW2[] = "HSV";
//static const char WINDOW3[] = "pink_thresh";

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
     *   Replace Opecv cvtColor Function
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

    /*===============================
     *   Show a Rainbow Histogram
     ==============================*/

int showHistogram(cv::Mat image, cv::Mat Mask)
{
int hbins = 24;             // number of bins
int histSize[] = {hbins};
float range[] = {0,180};
const float* ranges[] = {range};
int channels[] = {0};
std::vector< cv::Mat > hsvchannels;
cv::split(image, hsvchannels);
cv::Mat histogram;
int hist_w = 512; int hist_h = 400;
int bin_w = cvRound( hist_w/hbins );

cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
cv::calcHist(&hsvchannels[0], 1, channels, Mask, histogram, 1, histSize, ranges, true, false);

//rainbow assortment of colors to match the bin associated

cv::Scalar color[24] = {cv::Scalar( 0, 7, 245),cv::Scalar( 7,71,255),cv::Scalar( 0,128,241),cv::Scalar( 13,191,255),cv::Scalar( 60,254,254),cv::Scalar( 50,253,202),cv::Scalar( 0,255,129),cv::Scalar( 0,241,58),cv::Scalar( 60,254,53),cv::Scalar( 57,255,0),cv::Scalar( 129,254,1),cv::Scalar( 201,254,57),cv::Scalar( 245,246,0),cv::Scalar( 251,186,4),cv::Scalar( 252,136,0),cv::Scalar( 249,65,0),cv::Scalar( 252,1,0),cv::Scalar( 255,1,61),cv::Scalar( 251,0,130),cv::Scalar( 255,3,190),cv::Scalar( 249,0,255),cv::Scalar( 196,0,255),cv::Scalar( 121,0,252),cv::Scalar( 67,8,246) };
int max = 0;
int Integer = 30;
for( int i = 1; i < hbins; i++ )
{
cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(histogram.at<float>(i-1)) ) , cv::Point( bin_w*(i), hist_h - cvRound(histogram.at<float>(i)) ), color[i], 2, 8, 0  );
if((cvRound(histogram.at<float>(i)))>max)
        {
        max = (cvRound(histogram.at<float>(i)));
        Integer = i;
        }
}   
//cv::imshow(WINDOW3,histImage);
//cv::waitKey(3);

return Integer;
}

    /*=========================================
     *   Function to Compare Contour Areas
     ========================================*/
     
bool compareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 ) {
double i = std::fabs( cv::contourArea(cv::Mat(contour1)) );
double j = std::fabs( cv::contourArea(cv::Mat(contour2)) );
return ( i < j );
}

	/*====================================================
     *  Draw Crosshairs to designate center and angles
     ===================================================*/

cv::Mat drawCross(cv::Mat image)
{
	int h = image.rows;
	int w = image.cols;
    cv::line(image,cv::Point(0,h/2),cv::Point(floor(.45*w),h/2),cv::Scalar(0,0,0),3,8); 
    cv::line(image,cv::Point(floor(.55*w),h/2),cv::Point(w,h/2),cv::Scalar(0,0,0),3,8); 
    cv::line(image,cv::Point(w/2,0),cv::Point(w/2,floor(.4*h)),cv::Scalar(0,0,0),3,8); 
    cv::line(image,cv::Point(w/2,floor(.6*h)),cv::Point(w/2,h),cv::Scalar(0,0,0),3,8); 
    int thick[12] = {1,1,2,1,1,1,2,1,1,1,1,2};
    float line[12] = {16/2,16/3,16/4,16/5,16/6,16/7,16/8,16/9,16/10,16/11,16/12,16/14};
    int angle1[4] = {-10,80,170,260};
    int angle2[4] = {10,100,190,280};
    for (int i=0; i<4; i++)
    {
        for ( int j=0; j<12; j++)
        {

    cv::ellipse(image,cv::Point(w/2,h/2),cv::Size(floor(h/line[j]),floor(h/line[j])),0,angle1[i],angle2[i],cv::Scalar(0,0,0),thick[j],8,0);
        }}
    cv::ellipse(image,cv::Point(w/2,h/2),cv::Size(2,2),0,0,360,cv::Scalar(0,0,255),-1,8,0);
    return image;
}

	/*=========================================
     *  Function Called for Every New Image
     ========================================*/

void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
     ros::Time time = ros::Time::now();

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
    
    //cv::createTrackbar( "size limit",WINDOW1, &size_int, 1000);
    //cv::createTrackbar( "ring size",WINDOW1, &ring_int, 100);
    //cv::createTrackbar( "Error Threshold",WINDOW1, &error_threshold_int, 500);
    float size_limit = size_int*1;
    float error_threshold = error_threshold_int*.01;

    /*=============================
     *   Set up image variables
     ============================*/

	int h = cv_ptr->image.rows;
	int w = cv_ptr->image.cols;
	//cv::Mat hsvframe = cvCreateMat(h,w,0);
	//cv::cvtColor(cv_ptr->image,hsvframe,CV_BGR2HSV);
	cv::Mat hsvframe = BGR2HSV(cv_ptr->image);
    
    /*============================
     *    Color Tuning Values
     ===========================*/

	// Tuning done on 27 June 2014 around 3pm
	// Use blob tuner and then input values in order below/
	int green[] = {78,54,196,26,255,84,1,1};
	int blue[] = {107,90,210,20,255,80,2,2};
	int pink[] = {177,157,220,50,255,129,2,2};
	int orange[] = {20,10,191,40,255,190,2,2};
	cv::Scalar pink_color = cv::Scalar( 0, 0, 100) ;
	cv::Scalar orange_color = cv::Scalar( 0, 153, 255) ;
	cv::Scalar blue_color = cv::Scalar( 200, 0, 100) ;
	cv::Scalar green_color = cv::Scalar( 0, 204, 204) ;
	cv::Mat green_thresh(cv::Size(w,h),CV_8U);
	cv::Mat blue_thresh(cv::Size(w,h),CV_8U);
	cv::Mat pink_thresh(cv::Size(w,h),CV_8U);
	cv::Mat orange_thresh(cv::Size(w,h),CV_8U);

   
    /*============================
     *    Create Coloring Masks
     ===========================*/

	// Green Mask
	cv::Mat green_element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(2*green[6] +1, 2*green[6] + 1), cv::Point(green[6], green[6]));
	cv::inRange(hsvframe,cv::Scalar(green[1],green[3],green[5]),cv::Scalar(green[0],green[2],green[4]),green_thresh);
	cv::dilate(green_thresh,green_thresh,green_element);
	cv::erode(green_thresh,green_thresh,green_element);
	cv::GaussianBlur(green_thresh,green_thresh,cv::Size(2*green[7]+1,2*green[7]+1),2,2);

	// blue Mask
	cv::Mat blue_element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(2*blue[6] +1, 2*blue[6] + 1), cv::Point(blue[6], blue[6]));
	cv::inRange(hsvframe,cv::Scalar(blue[1],blue[3],blue[5]),cv::Scalar(blue[0],blue[2],blue[4]),blue_thresh);
	cv::dilate(blue_thresh,blue_thresh,blue_element);
	cv::erode(blue_thresh,blue_thresh,blue_element);
	cv::GaussianBlur(blue_thresh,blue_thresh,cv::Size(2*blue[7]+1,2*blue[7]+1),2,2);

	// pink Mask
	cv::Mat pink_element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(2*pink[6] +1, 2*pink[6] + 1), cv::Point(pink[6], pink[6]));
	cv::inRange(hsvframe,cv::Scalar(pink[1],pink[3],pink[5]),cv::Scalar(pink[0],pink[2],pink[4]),pink_thresh);
	cv::dilate(pink_thresh,pink_thresh,pink_element);
	cv::erode(pink_thresh,pink_thresh,pink_element);
	cv::GaussianBlur(pink_thresh,pink_thresh,cv::Size(2*pink[7]+1,2*pink[7]+1),2,2);

	// orange Mask
	cv::Mat orange_element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(2*orange[6] +1, 2*orange[6] + 1), cv::Point(orange[6], orange[6]));
	cv::inRange(hsvframe,cv::Scalar(orange[1],orange[3],orange[5]),cv::Scalar(orange[0],orange[2],orange[4]),orange_thresh);
	cv::dilate(orange_thresh,orange_thresh,orange_element);
	cv::erode(orange_thresh,orange_thresh,orange_element);
	cv::GaussianBlur(orange_thresh,orange_thresh,cv::Size(2*orange[7]+1,2*orange[7]+1),2,2);


	/*========================================================
	 *    Find Largest Contour that resemples an ellipse 
	 ========================================================*/

	roi.landmarks.resize(4);
        /*====================================
         *    Green Centered Pink Outline
         ===================================*/
    cv::Mat green_largestmask(cv::Size(w,h),CV_8U);
    
    risc_msgs::Risc_roi green_roi;
    std::vector< std::vector<cv::Point> > green_contours,green_ellipse;
    cv::findContours(green_thresh, green_contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    std::sort(green_contours.begin(), green_contours.end(), compareContourAreas);
    int green_Index = 0;
    cv::RotatedRect green_boundRect;
    cv::Mat green_temp(cv::Size(w,h),CV_8U); 
    cv::Mat green_mask(cv::Size(w,h),CV_8U);
    green_mask.setTo(cv::Scalar(0,0,0));

		/*================================================
         *    Loop Through all Contours if they exist
         ===============================================*/
    
	if ( green_contours.size() != 0 )
	{
		for( int i = 0; i< green_contours.size(); i++ )
		{
			if (cv::contourArea(green_contours[i])>117)
			{ 
				green_boundRect = cv::minAreaRect(green_contours[i]);

			   /*================================================
				*    Store the index and the corresponding Mask
				===============================================*/
			   
				green_Index = i;
				green_largestmask.setTo(cv::Scalar(0,0,0));
				green_boundRect.size.height = green_boundRect.size.height*(1+.01*ring_int);
				green_boundRect.size.width = green_boundRect.size.width*(1+.01*ring_int);
				cv::ellipse( green_largestmask, green_boundRect, cv::Scalar(255,255,255),-1, 8);
				cv::drawContours(green_largestmask, green_contours, i , cv::Scalar(0,0,0), CV_FILLED, 8);
				int Color = showHistogram(hsvframe, green_largestmask); 
				if ((cv::contourArea(green_contours[green_Index])>size_limit) && (floor(pink[1]/7.5) <= Color) && (Color <= ceil(pink[0]/7.5)))
				{

					 /*===================
					  *    Create ROI
					  ==================*/
					 cv::RotatedRect green_Rect;
					 green_Rect = cv::minAreaRect(green_contours[green_Index]);

					 green_roi.name = "green";
					 green_roi.x = green_Rect.center.x;
					 green_roi.y = green_Rect.center.y;
					 green_roi.width = green_Rect.size.width;
					 green_roi.height = green_Rect.size.height;
					 green_roi.angle = green_Rect.angle;
		  
		 
					 /*=============
					  *    Draw 
					  ============*/
					 cv::ellipse( cv_ptr->image, green_Rect, green_color,-1, 8);
	}}}}

			/*====================================
			 *    pink Centered blue Outline
			 ===================================*/
		cv::Mat pink_largestmask(cv::Size(w,h),CV_8U);
		
		risc_msgs::Risc_roi pink_roi;
		std::vector< std::vector<cv::Point> > pink_contours,pink_ellipse;
		cv::findContours(pink_thresh, pink_contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
		std::sort(pink_contours.begin(), pink_contours.end(), compareContourAreas);
		int pink_Index = 0;
		cv::RotatedRect pink_boundRect;

		/*================================================
         *    Loop Through all Contours if they exist
         ===============================================*/
		
	if ( pink_contours.size() != 0 )
	{
		for( int i = 0; i< pink_contours.size(); i++ )
		{
			if (cv::contourArea(pink_contours[i])>110)
			{ 
				pink_boundRect = cv::minAreaRect(pink_contours[i]);
				cv::Mat pink_mask(cv::Size(w,h),CV_8U);
				pink_mask.setTo(cv::Scalar(0,0,0));
				cv::ellipse(pink_mask, pink_boundRect, cv::Scalar(255,255,255),-1,8);
				cv::findContours(pink_mask, pink_ellipse,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));
				float E = cv::matchShapes(pink_contours[i],pink_ellipse[0],1,0);
				if (E<.6)
				{

			   /*================================================
				*    Store the index and the corresponding Mask
				===============================================*/
			   
				pink_Index = i;
				pink_largestmask.setTo(cv::Scalar(0,0,0));
				pink_boundRect.size.height = pink_boundRect.size.height*(1+.01*ring_int);
				pink_boundRect.size.width = pink_boundRect.size.width*(1+.01*ring_int);
				cv::ellipse( pink_largestmask, pink_boundRect, cv::Scalar(255,255,255),-1, 8);
				cv::drawContours(pink_largestmask, pink_contours, i , cv::Scalar(0,0,0), CV_FILLED, 8);
				int Color = showHistogram(hsvframe, pink_largestmask); 
				if ((cv::contourArea(pink_contours[pink_Index])>size_limit) && (floor(blue[1]/7.5) <= Color) && (Color <= ceil(blue[0]/7.5)))
				{

					 /*===================
					  *    Create ROI
					  ==================*/
					 cv::RotatedRect pink_Rect;
					 pink_Rect = cv::minAreaRect(pink_contours[pink_Index]);

					 pink_roi.name = "pink";
					 pink_roi.x = pink_Rect.center.x;
					 pink_roi.y = pink_Rect.center.y;
					 pink_roi.width = pink_Rect.size.width;
					 pink_roi.height = pink_Rect.size.height;
					 pink_roi.angle = pink_Rect.angle;
		  
		 
					 /*=============
					  *    Draw 
					  ============*/
					 cv::ellipse( cv_ptr->image, pink_Rect, pink_color,-1, 8);
	}}}}}

			/*====================================
			 *    orange Centered blue Outline
			 ===================================*/
		cv::Mat orange_largestmask(cv::Size(w,h),CV_8U);
		
		risc_msgs::Risc_roi orange_roi;
		std::vector< std::vector<cv::Point> > orange_contours,orange_ellipse;
		cv::findContours(orange_thresh, orange_contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
		std::sort(orange_contours.begin(), orange_contours.end(), compareContourAreas);
		int orange_Index = 0;
		cv::RotatedRect orange_boundRect;

		/*================================================
         *    Loop Through all Contours if they exist
         ===============================================*/
		
	if ( orange_contours.size() != 0 )
	{
		for( int i = 0; i< orange_contours.size(); i++ )
		{
			if (cv::contourArea(orange_contours[i])>110)
			{ 
				orange_boundRect = cv::minAreaRect(orange_contours[i]);
				cv::Mat orange_mask(cv::Size(w,h),CV_8U);
				orange_mask.setTo(cv::Scalar(0,0,0));
				cv::ellipse(orange_mask, orange_boundRect, cv::Scalar(255,255,255),-1,8);
				cv::findContours(orange_mask, orange_ellipse,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));
				float E = cv::matchShapes(orange_contours[i],orange_ellipse[0],1,0);
				if (E<.1)
				{

			   /*================================================
				*    Store the index and the corresponding Mask
				===============================================*/
			   
				orange_Index = i;
				orange_largestmask.setTo(cv::Scalar(0,0,0));
				orange_boundRect.size.height = orange_boundRect.size.height*(1+.01*45);
				orange_boundRect.size.width = orange_boundRect.size.width*(1+.01*45);
				cv::ellipse( orange_largestmask, orange_boundRect, cv::Scalar(255,255,255),-1, 8);
				cv::drawContours(orange_largestmask, orange_contours, i , cv::Scalar(0,0,0), CV_FILLED, 8);
				int Color = showHistogram(hsvframe, orange_largestmask); 
				if ((cv::contourArea(orange_contours[orange_Index])>size_limit) && (floor(blue[1]/7.5) <= Color) && (Color <= ceil(blue[0]/7.5)))
				{

					 /*===================
					  *    Create ROI
					  ==================*/
					 cv::RotatedRect orange_Rect;
					 orange_Rect = cv::minAreaRect(orange_contours[orange_Index]);

					 orange_roi.name = "Orange";
					 orange_roi.x = orange_Rect.center.x;
					 orange_roi.y = orange_Rect.center.y;
					 orange_roi.width = orange_Rect.size.width;
					 orange_roi.height = orange_Rect.size.height;
					 orange_roi.angle = orange_Rect.angle;
		  
					 /*=============
					  *    Draw 
					  ============*/
					 cv::ellipse( cv_ptr->image, orange_Rect, orange_color,-1, 8);
	}}}}}

			/*====================================
			 *    blue Centered green Outline
			 ===================================*/
		cv::Mat blue_largestmask(cv::Size(w,h),CV_8U);
		
		risc_msgs::Risc_roi blue_roi;
		std::vector< std::vector<cv::Point> > blue_contours,blue_ellipse;
		cv::findContours(blue_thresh, blue_contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
		std::sort(blue_contours.begin(), blue_contours.end(), compareContourAreas);
		int blue_Index = 0;
		cv::RotatedRect blue_boundRect;

		/*================================================
         *    Loop Through all Contours if they exist
         ===============================================*/
		
	if ( blue_contours.size() != 0 )
	{
		for( int i = 0; i< blue_contours.size(); i++ )
		{
			if (cv::contourArea(blue_contours[i])>110)
			{ 
				blue_boundRect = cv::minAreaRect(blue_contours[i]);
				cv::Mat blue_mask(cv::Size(w,h),CV_8U);
				blue_mask.setTo(cv::Scalar(0,0,0));
				cv::ellipse(blue_mask, blue_boundRect, cv::Scalar(255,255,255),-1,8);
				cv::findContours(blue_mask, blue_ellipse,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));
				float E = cv::matchShapes(blue_contours[i],blue_ellipse[0],1,0);
				if (E<.6)
				{

			   /*================================================
				*    Store the index and the corresponding Mask
				===============================================*/
			   
				blue_Index = i;
				blue_largestmask.setTo(cv::Scalar(0,0,0));
				blue_boundRect.size.height = blue_boundRect.size.height*(1+.01*ring_int);
				blue_boundRect.size.width = blue_boundRect.size.width*(1+.01*ring_int);
				cv::ellipse( blue_largestmask, blue_boundRect, cv::Scalar(255,255,255),-1, 8);
				cv::drawContours(blue_largestmask, blue_contours, i , cv::Scalar(0,0,0), CV_FILLED, 8);
				int Color = showHistogram(hsvframe, blue_largestmask); 
				if ((cv::contourArea(blue_contours[blue_Index])>size_limit) && (floor(green[1]/7.5) <= Color) && (Color <= ceil(green[0]/7.5)))
				{

					 /*===================
					  *    Create ROI
					  ==================*/
					 cv::RotatedRect blue_Rect;
					 blue_Rect = cv::minAreaRect(blue_contours[blue_Index]);

					 blue_roi.name = "blue";
					 blue_roi.x = blue_Rect.center.x;
					 blue_roi.y = blue_Rect.center.y;
					 blue_roi.width = blue_Rect.size.width;
					 blue_roi.height = blue_Rect.size.height;
					 blue_roi.angle = blue_Rect.angle;
		  
		 
					 /*=============
					  *    Draw 
					  ============*/
					 cv::ellipse( cv_ptr->image, blue_Rect, blue_color,-1, 8);
	}}}}}

//	cv::imshow(WINDOW1,cv_ptr->image); 
//	cv::waitKey(3);                                                   




	 
		/*===================
		 *    Publish
		 ==================*/

	roi.landmarks[0] = green_roi;
	roi.landmarks[1] = pink_roi;
	roi.landmarks[2] = orange_roi;
	roi.landmarks[3] = blue_roi;
    risc_msgs::Risc_rois seen;
    int length = 0;
	for(int i; i<4; i++)
	{
		roi.landmarks[i].visible = false;
		if(roi.landmarks[i].width != 0 && roi.landmarks[i].height != 0 ) 
		{roi.landmarks[i].visible = true;
         length++;
         seen.landmarks.resize(length);
         seen.landmarks[length-1] = roi.landmarks[i];}
	}

	risc_msgs::Observed_rois R;
    R.Obj.resize(1);
	R.Obj[0] = seen; //roi;
	R.Obj[0].name = "fluffy_II";
	R.header.stamp = time;
	pub.publish(R);                      

    cv_ptr->image = drawCross(cv_ptr->image);
    im.publish(cv_ptr->toImageMsg());
                                                                                           
}                                                                                            
int main(int argc, char **argv)                                                             
                                                                                            
{
        ros::init(argc, argv, "blob_finder");
        ros::NodeHandle nh;
        pub = nh.advertise<risc_msgs::Observed_rois>("land_rois", 1000);

        image_transport::ImageTransport it(nh);
                                                                                            
 //       cv::namedWindow(WINDOW1, CV_WINDOW_AUTOSIZE);
 //       cv::namedWindow(WINDOW2, CV_WINDOW_AUTOSIZE);
 //       cv::namedWindow(WINDOW3, CV_WINDOW_AUTOSIZE);
                                                                                            
        image_transport::Subscriber sub = it.subscribe("/ardrone/image_raw", 1, imageCallback);
        
        im = it.advertise("/ardrone/image_modi",1);

        cv::destroyAllWindows();                          
     
        ros::spin();
    
        ROS_INFO("ros_opencv::main.cpp::No error.");
  
 }
