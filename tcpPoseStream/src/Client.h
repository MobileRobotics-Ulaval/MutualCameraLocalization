#ifndef _LEDSFINDER_H_
#define _LEDSFINDER_H_

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <time.h>
#include <stdlib.h> //calloc
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <netdb.h> 
#include <string>
#include <iostream>
#include <cstdlib> // exit()
#include <fstream>
#include <sstream>
#include <iostream> 
#include <pthread.h>
#include <time.h>

//ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>
class Client
{
private:
    // TCP
    struct sockaddr_in server;
    int comSocket;
   
    ros::NodeHandle nh;
    ros::Subscriber pose_;

public:
	Client();
	void callback(const geometry_msgs::PoseStamped::ConstPtr& pos);
	~Client();
};

#endif

