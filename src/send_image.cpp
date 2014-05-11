#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include "proto/img.pb.h"
#include "proto/command.pb.h"


//mono8
int main(int argc, char **argv){
    unsigned char* raw_data = (unsigned char*)malloc(40*40);
    for(int i = 0; i < 40*40; i++)
        raw_data[i] = i % 2 == 0 ? 0: 255; 

    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
   // image_transport::ImageTransport it(nh);
   // image_transport::Publisher pub = it.advertise("camera/image", 1);
    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/cube_feed_topic", 1);

    //cv_image.image = cv::imread("/home/philippe/Pictures/ineffective_sorts.png", CV_LOAD_IMAGE_COLOR);
    cv_bridge::CvImage cv_image;
    int channels = 1; 
    IplImage* pimg = cvCreateImageHeader(cvSize(40, 40), IPL_DEPTH_8U, channels);

    cvSetData(pimg, raw_data, pimg->widthStep);

    cv_image.image = pimg;

    cv_image.encoding = "mono8";//"bgr8";
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);

    ros::Rate loop_rate(5);
    while (nh.ok()) {
        pub.publish(ros_image);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}