#ifndef _LEDSFINDER_H_
#define _LEDSFINDER_H_

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include "proto/img.pb.h"
#include "proto/command.pb.h"
#include "lodepng.h"
#include "lz4.h"
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
#include "proto/img.pb.h"
#include "proto/command.pb.h"


#include "Socket.h"

class Client
{
private:

    int WIDTH; //640
    int HEIGHT; //480

    unsigned char bigBuffer[307200];

    SocketTCP clientTCP;
    SocketUDP clientUDP;

    dotCapture::Img recvDelimProtobuf(unsigned char *buffer);

    // Threading
    pthread_mutex_t recordingMux; 
    volatile bool recording;

    //ROS
    ros::NodeHandle nh;
    image_transport::CameraPublisher pub;
public:
	Client();
	int startListeningLoop();
	int sendCommand(dotCapture::Command &com);
    void* receivingImgLoop();
    static void* callReceivingImgLoopFunction(void *arg) { return ((Client*)arg)->receivingImgLoop(); } // Threading hack

    void compressToPNG(unsigned char**& outBuf, size_t* outsize, const unsigned char *imgBuf, const long int size);
    void saveFilePNG(unsigned char* pngBuf, size_t pngSize, std::string filename);
    bool isRecording();
    void setRecording(bool r);
    void foo();
	~Client();
};

#endif

