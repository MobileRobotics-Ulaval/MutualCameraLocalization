#ifndef _CAMERA_MOD_H_
#define _CAMERA_MOD_H_

#include "FlyCapture2.h"
#include "lodepng.h"
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <cstdlib> // exit()
#ifdef __arm__
    #include <arm_neon.h>
#endif

namespace img_server
{
class Camera{
private:
    FlyCapture2::Camera cam;
    unsigned char* img;
    int imgSize;

    static const int WIDTH = 640;
    static const int HEIGHT = 480;

    float shutter;
    int brightness;
    int exposure;
    float gain;
    int threshold;

    int zig;
    void takePicturesAndSaveIt(int nbrPic, int threshold);
    void printErrorCam(FlyCapture2::Error error);
public:
	//FlyCapture2::Image img;

	Camera(int threshold, float shutterTime, int brightness, int exposure, float gain);
	int initCamera();
    void takeRawPicture();
    void takeFakePicture();
	void stopRecording();
    void compressToPNG(unsigned char**& outBuf, size_t* outsize, const unsigned char *imgBuf, const int w, const int h);
    void saveFilePNG(unsigned char* pngBuf, size_t pngSize, std::string filename);
    void configureProperties();

   	void doThreshold();
    void divByTwoRes(unsigned char* c, int w, int h);

    void setShutter(float shutter);
    void setBrightness(int brightness);
    void setExposure(int exposure);
    void setGain(float gain);
    void setThreshold(int threshold);
};

}
#endif