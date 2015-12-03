
/*======================================================
	Created by:  	D. Spencer Maughan
	Last updated: 	July 2014
	File name: 	Landmarks.cpp
	Organization:	RISC Lab, Utah State University
 ======================================================*/

                                                                                           
int main(int argc, char **argv)                                                             
                                                                                            
{
        ros::init(argc, argv, "landmark_finder");
        ros::NodeHandle nh;
        pub = nh.advertise<risc_msgs::Observed_rois>("land_rois", 200);

        image_transport::ImageTransport it(nh);

        im1 = it.advertise("risc1/ardrone/image_modi",1);
        im2 = it.advertise("risc2/ardrone/image_modi",1);
        im3= it.advertise("risc3/ardrone/image_modi",1);
        im4= it.advertise("risc4/ardrone/image_modi",1);

        image_transport::Subscriber sub1= it.subscribe("risc1/ardrone/image_rect_color", 1, imageCallback1);
        image_transport::Subscriber sub2= it.subscribe("risc2/ardrone/image_rect_color", 1, imageCallback2);
        image_transport::Subscriber sub3= it.subscribe("risc3/ardrone/image_rect_color", 1, imageCallback3);
        image_transport::Subscriber sub4= it.subscribe("risc4/ardrone/image_rect_color", 1, imageCallback4);

        cv::destroyAllWindows();                          
     
        ros::spin();
    
        ROS_INFO("ros_opencv::main.cpp::No error.");
  
 }
