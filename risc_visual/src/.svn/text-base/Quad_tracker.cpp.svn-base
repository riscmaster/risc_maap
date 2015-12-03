
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

int size_limit = 117;
int error_threshold_int = 50;
int ring_int = 20;
ros::Publisher pub; 
risc_msgs::Risc_rois roi;
static const char WINDOW1[] = "Quads";
static const char WINDOW2[] = "Combined";
static const char WINDOW3[] = "Green";
static const char WINDOW4[] = "Pink";
static const char WINDOW5[] = "Red";
static const char WINDOW6[] = "Yellow";

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

//cv::imshow(WINDOW2,histImage);
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

    /*=========================
     *   Get Color Blob Mask
     ==========================*/


cv::Mat addColorBlob(cv::Mat Mask, cv::Mat Color)
{

    /*=============================
     *   Set up image variables
     ============================*/

	int h = Mask.rows;
	int w = Mask.cols;
    cv::Mat Zeromat(cv::Size(w,h),CV_8U);
    Zeromat.setTo(cv::Scalar(0,0,0));

	/*==============================
	 *    Find Largest Contour 
	 ==============================*/

// Initiate with this because that color is unique
                     /*=======
                      *  One
                       ========*/

		cv::Mat One_largestmask(cv::Size(w,h),CV_8U);
		One_largestmask.setTo(cv::Scalar(0,0,0));
		std::vector< std::vector<cv::Point> > One_contours,One_ellipse;
		cv::findContours(Color, One_contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
		std::sort(One_contours.begin(), One_contours.end(), compareContourAreas);

        /*================================================
         *    Loop Through all Contours if they exist
         ===============================================*/
		
	if ( One_contours.size() != 0 )
	{
		int k = 0;
		int center[2];

		for( int i = One_contours.size()-1; i> -1; i-- )
		{
			if (cv::contourArea(One_contours[i])>size_limit)
			{ 
				cv::RotatedRect One_boundRect;
				One_boundRect = cv::minAreaRect(One_contours[i]);
if (i == One_contours.size()-1)
{
center[0]=One_boundRect.center.x;
center[1]=One_boundRect.center.y;
if (One_boundRect.size.height<One_boundRect.size.width){center[2] = 1.1*One_boundRect.size.height;}
if (One_boundRect.size.height>One_boundRect.size.width){center[2] = 1.1*One_boundRect.size.width;}
}
				int x = abs(One_boundRect.center.x-center[0]);
				int y = abs(One_boundRect.center.y-center[1]);
				float dist = sqrt(x*x+y*y);
if (dist < center[2])
{
				cv::ellipse(One_largestmask, One_boundRect, cv::Scalar(255,255,255),-1,8);
//				cv::line(One_largestmask, One_boundRect.center,cv::Point(center[0],center[1]), cv::Scalar(255,255,255),2,8);
}
					}}

            if ( cv::countNonZero(One_largestmask) > 0)
            {
                                                 
            cv::findContours(One_largestmask, One_ellipse,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));
            std::sort(One_ellipse.begin(), One_ellipse.end(), compareContourAreas);

	    cv::Mat One_mask(cv::Size(w,h),CV_8U);
	    One_mask.setTo(cv::Scalar(0,0,0));
            cv::RotatedRect One_e = cv::minAreaRect(One_ellipse[One_ellipse.size()-1]);
	    cv::ellipse(One_mask, One_e, cv::Scalar(255,255,255),-1,8);
            Mask = combineMasks(One_mask,Mask);
}
}
return Mask;
}
cv::RotatedRect findQuad(cv::Mat One, cv::Mat Two, cv::Mat Three, cv::Mat Four)
{
cv::Mat Mask(cv::Size(One.cols,One.rows),CV_8U);
Mask = addColorBlob(Mask,One);
Mask = addColorBlob(Mask,Two);
//Mask = addColorBlobrestricted(Mask,Three,rect);
//Mask = addColorBlobrectricted(Mask,Four,rect);
        cv::imshow(WINDOW2,Mask); 
        cv::waitKey(3);                  
cv::RotatedRect Quad;
return Quad;
 

}


        /*========================
         *     Find boundaries
         ========================*/
	
cv::Mat boundary(int C1[8],int C2[8],cv::Mat HSV,int kernal)
{
cv::Mat m(cv::Size(HSV.cols,HSV.rows),CV_8UC1);
m.setTo(cv::Scalar(0,0,0));

for (int u=kernal; u<HSV.rows; u++)
{
	for (int v=kernal; v<HSV.cols; v++)
        {
               int C1_m = 0;
               int C2_m = 0;
               for (int i = 0; i<kernal; i++)
               {
                     for (int j = 0; j<kernal; i++)
                     {
			int H = (HSV.at<cv::Vec3b>(u-i,v-j)[0]);
			int S = (HSV.at<cv::Vec3b>(u-i,v-j)[1]);
			int V = (HSV.at<cv::Vec3b>(u-i,v-j)[2]);
			if (S<C1[2] && S>C1[3] && V<C1[4] && V>C1[5] && H<C1[0] && H>C1[1])
			{     C1_m = C1_m+1;}

			if (S<C2[2] && S>C2[3] && V<C2[4] && V>C2[5] && H<C2[0] && H>C2[1])
			{     C2_m = C2_m+1;}
                     }
               }

	       m.at<cv::Vec3b>(u,v)[0] = floor(C2_m*C1_m*255/(kernal*kernal));
        }
}   
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
    
    //cv::createTrackbar( "Error Threshold",WINDOW1, &error_threshold_int, 500);

    /*=============================
     *   Set up image variables
     ============================*/
	int h = cv_ptr->image.rows;
	int w = cv_ptr->image.cols;
    cv::Mat Zeromat(cv::Size(w,h),CV_8U);
    Zeromat.setTo(cv::Scalar(0,0,0));
	cv::Mat hsvframe = BGR2HSV(cv_ptr->image);
    
    /*=========================================
     *    Color Tuning Values for Paints
     ========================================*/

	// Tuning done on 27 June 2014 around 3pm
	// update on 29 July 2014
	// Use blob tuner and then input values in order below/
	int green[] = {95,35,230,80,255,5,2,1};
	int blue[] = {140,100,255,15,180,5,1,2};
	int red[] = {2,177,255,130,255,100,2,1};
	int yellow[] = {30,20,255,110,255,90,2,1};
	int pink[] = {175,160,184,61,255,160,1,1};
	int orange[] = {19,2,255,130,255,185,1,1};
	cv::Scalar green_color = cv::Scalar( 0, 0, 100) ;
	cv::Scalar blue_color = cv::Scalar( 0, 153, 255) ;
	cv::Scalar red_color = cv::Scalar( 0, 100, 0) ;
	cv::Scalar yellow_color = cv::Scalar( 204, 0, 204) ;
   
    /*============================
     *    Create Coloring Masks
     ===========================*/
	cv::Mat green_thresh = createColorMask(hsvframe,green,w,h);
	cv::Mat blue_thresh = createColorMask(hsvframe,blue,w,h);
	cv::Mat red_thresh = createColorMask(hsvframe,red,w,h);
	cv::Mat yellow_thresh = createColorMask(hsvframe,yellow,w,h);
	cv::Mat pink_thresh = createColorMask(hsvframe,pink,w,h);
	cv::Mat orange_thresh = createColorMask(hsvframe,orange,w,h);
                                 



        cv::Mat green_red = boundary(green,red,hsvframe,5);
        cv::imshow(WINDOW3,green_red); 
        cv::waitKey(3);                                                   
        cv::imshow(WINDOW4,pink_thresh); 
        cv::waitKey(3);                                                   
        cv::imshow(WINDOW6,yellow_thresh); 
        cv::waitKey(3);                                                   
        cv::imshow(WINDOW5,red_thresh); 
        cv::waitKey(3);                                                   
//	cv::Mat combined(cv::Size(w,h),CV_8U);
//	combined = combineMasks(green_thresh,blue_thresh);
//	combined = combineMasks(combined,red_thresh);
//	combined = combineMasks(combined,yellow_thresh);

       /*===================
        *    Find Fluffy
         ===================*/

        cv::RotatedRect fluffy = findQuad(orange_thresh, yellow_thresh, pink_thresh, blue_thresh);
                       
       /*==================
        *    Draw Fluffy 
         ==================*/

        if (fluffy.size.width!=0 && fluffy.size.height!=0)
            {
		cv::ellipse( cv_ptr->image, fluffy, red_color,2, 8);
            } 
    
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
        cv::namedWindow(WINDOW6, CV_WINDOW_AUTOSIZE);
                                                                                            
        //image_transport::Subscriber sub = it.subscribe("ardrone/image_rect_color", 1, imageCallback);
        image_transport::Subscriber sub = it.subscribe("ardrone/image_raw", 1, imageCallback);
        cv::destroyAllWindows();                          
        ros::spin();
    
        ROS_INFO("ros_opencv::main.cpp::No error.");
  
 }
