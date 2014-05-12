#ifndef _LEDSFINDER_H_
#define _LEDSFINDER_H_

#include "FlyCapture2.h"
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include "proto/img.pb.h"
#include "proto/command.pb.h"
#include "lodepng.h"
#include "lz4.h"
#include <time.h>
#include <stdlib.h> //calloc
#include <stdio.h>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <netdb.h> 
#include <string>
#include <iostream>
#include <sstream>
#include <cstdlib> // exit()
#include <pthread.h>
//#include <mutex>
//#include <thread>
//#include <chrono>

using namespace FlyCapture2;

class LedsFinder
{
private:
    // cam
    Camera cam;
    long time;
    float shutter;
    int brightness;
    int exposure;
    float gain;
    int threshold;

    // TCP
    const int port;
    static const int BUFFER = 1024;
    static const int WIDTH = 640;
    static const int HEIGHT = 480;
    char comBuffer[BUFFER];
    struct sockaddr_in ClientAddress;
    int comSocket;   

    // Threading
    pthread_t imgGathering;
    pthread_mutex_t recordingMux; 
    pthread_mutex_t proprietyMux; 
    bool recording;
    
    void printErrorCam(Error error);
    void checkForErrorTCP(int errorCode, const char* errorMessage);
    long getTimeInMilliseconds();
    dotCapture::Command*  getCommand();

    void compressToPNG(unsigned char**& outBuf, size_t* outsize, const unsigned char *imgBuf, const long int size);
    void saveFilePNG(unsigned char* pngBuf, size_t pngSize, std::string filename);
    dotCapture::Img* dataToProto(long timestamp, int timestamp_microsec, unsigned char* data, long unsigned int size);


    int startRecording();
    void stopRecording();

    static void* callLoopRecordingFunction(void *arg) { return ((LedsFinder*)arg)->loopRecording(); }

    void* loopRecording();
    void configureProperties();
    void doThreshold(Image& img, int t);

   
    bool isRecording();
    void setRecording(bool r);


public:
    void takeRawPicture(int nbrPic, int threshold);
    LedsFinder(int port, int threshold, float shutter, int brightness, int exposure, float gain);
    ~LedsFinder();
    void startProcessingLoop();
    void sendProto(dotCapture::Img* message);
};

#endif

