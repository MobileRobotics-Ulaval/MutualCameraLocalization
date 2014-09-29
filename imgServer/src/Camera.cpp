#include "Camera.h"

using namespace std;

namespace img_server
{
Camera::Camera(int threshold, float shutterTime, int brightness, int exposure, float gain):
    threshold(threshold), shutter(shutterTime), brightness(brightness), exposure(exposure), gain(gain){
        
    img = (unsigned char*)malloc(WIDTH * HEIGHT);
    imgSize = (WIDTH * HEIGHT);
    zig = 0;
}


/*
   Initiate the camera's parameter
*/
int Camera::initCamera(){
    FlyCapture2::BusManager busMgr;
    FlyCapture2::PGRGuid guid;
    if(busMgr.GetCameraFromIndex(0, &guid) != FlyCapture2::PGRERROR_OK){
      printf("No camera connected.\n");
      //exit(0);
      //return -1;
    }
    this->printErrorCam(cam.Connect(&guid));
    this->configureProperties();
    this->printErrorCam(cam.StartCapture());
    
    // Sometimes, the first image is invalid. This way, we get rid of it.
    FlyCapture2::Image rawImage;
    this->printErrorCam(cam.RetrieveBuffer( &rawImage ));
    return 0;
}

void Camera::takeRawPicture(){
    FlyCapture2::Image image;
    //FlyCapture2::Error err = 
    cam.RetrieveBuffer(&image);
    img = image.GetData();
    imgSize = image.GetDataSize();

}

void Camera::takeFakePicture(){
    if(zig > 255) zig = 0;
    printf("color: %i\n", zig);
    for(int i = 0; i < imgSize; i++){
        img[i] = i < WIDTH * HEIGHT * 0.5 ? zig : i%255;
    }
    zig++;
}
/**
    Pass a raw image throught a threshold
*/
void Camera::doThreshold(){
    int t = this->threshold;
    unsigned char* p = this->img;
    unsigned int size = this->imgSize;
    for(int i = 0; i < size; i++){
        if(p[i] < t)
            p[i] = 0;
    }
}

/*
   Update the camera's parameters
*/
void Camera::configureProperties(){
    FlyCapture2::Property property;
    
    property.type = FlyCapture2::SHUTTER;
    this->printErrorCam(cam.GetProperty( &property ));
    property.onOff = true;
    property.absControl = true;
    property.autoManualMode = false;
    property.absValue = shutter;
    this->printErrorCam(cam.SetProperty( &property ));
    
    property.type = FlyCapture2::BRIGHTNESS;
    this->printErrorCam(cam.GetProperty( &property ));
    property.onOff = true;
    property.autoManualMode = false;
    property.absValue = brightness;
    this->printErrorCam(cam.SetProperty( &property ));
    
    property.type = FlyCapture2::AUTO_EXPOSURE;
    this->printErrorCam(cam.GetProperty( &property ));
    property.onOff = true;
    property.autoManualMode = false;
    property.absValue = exposure;
    this->printErrorCam(cam.SetProperty( &property ));
    
    property.type = FlyCapture2::GAIN;
    this->printErrorCam(cam.GetProperty( &property ));
   property.onOff = true;
    property.absControl = true;
    property.autoManualMode = false;
    property.absValue = gain;
    this->printErrorCam(cam.SetProperty( &property ));
}

/*
   Disconnect the camera
*/
void Camera::stopRecording()
{
    this->printErrorCam(cam.StopCapture());
    this->printErrorCam(cam.Disconnect());
}


/*
   Convert a char array to PNG
*/
void Camera::compressToPNG(unsigned char**& pngBuf, size_t* pngSize, const unsigned char *imgBuf, const int w, const int h){
    int error = 1;//lodepng_encode_memory(pngBuf, pngSize, imgBuf, w, h, LCT_GREY,  8);

    if(error){
        //printf("Error %u: %s\n", error, lodepng_error_text(error));
        exit(0);
    }
    printf("Compression success!!\npng=%.3fko raw=%.3fko\n", (float)*pngSize/1000.f, (float)(w*h)/1000.f);
}

/*
   Save a PNG in memory to a file
*/
void Camera::saveFilePNG(unsigned char* pngBuf, size_t pngSize, string filename){
    //lodepng_save_file(pngBuf, pngSize, filename.c_str());
}

/*
   Error output functions
*/
void Camera::printErrorCam(FlyCapture2::Error error ){
    if (error != FlyCapture2::PGRERROR_OK)
        error.PrintErrorTrace();
}

/**
    Reduce the resolution of an image by a factor 2
*/
void Camera::divByTwoRes(unsigned char* c, int width, int height){
    unsigned char* p = this->img;
    #ifdef __arm__
       
    uint8_t * __restrict src1;
    uint8_t * __restrict src2;
    uint8_t * __restrict dest;

    for (int h = 0; h < height/2 - 1; h++){
        src1 = p+width*(h*2);
        src2 = p+width*(h*2+1);
        dest = c+width/2*h;
        for (int i = 0; i < width; i += 16){
            // load upper line and add neighbor pixels:
            uint16x8_t a = vpaddlq_u8 (vld1q_u8 (src1));

            // load lower line and add neighbor pixels:
            uint16x8_t b = vpaddlq_u8 (vld1q_u8 (src2));

            // sum of upper and lower line: 
            uint16x8_t c = vaddq_u16 (a,b);

            // divide by 4, convert to char and store:
            vst1_u8 (dest, vshrn_n_u16 (c, 2));

            // move pointers to next chunk of data
            src1+=16;
            src2+=16;
            dest+=8;
        }
    }

    #endif
    #ifndef __arm__
        int nw = width/2;
        int nh = height/2;
        //unsigned char* c = (unsigned char*)malloc(w * h/4);
        /*
        //USE MAXIMUN 
            for(int i = 0; i < nw; i++){
                for(int j = 0; j < nh; j++){// Way too slow
                //printf("%i\n", (2 * i * w + 2 * j));
                c[j * nw + i]  = max(
                                    max(
                                        max(p[2 * j * width + 2 * i], p[2 * j * width + 2 * i + 1]),
                                        p[(2 * j + 1) * width + 2 * i]),
                                    p[(2 * j + 1) * width + 2 * i + 1]);
                }
            }
        }*/
        // USE AVERAGE
        for(int i = 0; i < nw; i++){
            for(int j = 0; j < nh; j++){// Way too slow
            //printf("%i\n", (2 * i * w + 2 * j));
            c[j * nw + i]  =(p[2 * j * width + 2 * i] +
                             p[2 * j * width + 2 * i + 1] +
                             p[(2 * j + 1) * width + 2 * i] +
                             p[(2 * j + 1) * width + 2 * i + 1])/4;
            }
        }
    #endif
   // p = c;
}

/**
    Debug function for camera testing
*/
void Camera::takePicturesAndSaveIt(int nbrPic, int threshold){
  /*FlyCapture2::Image rawImage;
  FlyCapture2::PNGOption pngOption;
  pngOption.interlaced = false;
  pngOption.compressionLevel = 0;

  this->startRecording();
    unsigned char** pngBuf = new unsigned char*();
    size_t pngSize;
    unsigned char* izBuff = (unsigned char*)malloc(WIDTH * HEIGHT);
    unsigned char* decod = (unsigned char*)malloc(WIDTH * HEIGHT);
    unsigned char* res = (unsigned char*)malloc(WIDTH * HEIGHT/4);

  for (int i=0; i < nbrPic; i++){
    FlyCapture2::Error err = cam.RetrieveBuffer( &rawImage );
    doThreshold(rawImage, threshold);
        divByTwoRes(rawImage.GetData(), res, WIDTH, HEIGHT);

        
        int e = LZ4_compress((char*)(void*)(res), (char*)(void*)(izBuff), rawImage.GetDataSize()/4);

        printf("Size =%i\n", e);
        int sizeDecom = LZ4_decompress_safe((char*)(void*)(izBuff), (char*)(void*)(decod), e, WIDTH * HEIGHT/4);
        printf("Sizedecod =%i\n", sizeDecom);    
        compressToPNG(pngBuf, &pngSize, decod, WIDTH/2, HEIGHT/2);    
        saveFilePNG(*pngBuf, pngSize, "camarche2.png");
  } 
  this->stopRecording();*/

}

void Camera::setShutter(float shutter){
    this->shutter = shutter;
}
void Camera::setBrightness(int n){
    this->brightness = n;
}
void Camera::setExposure(int n){
    this->exposure = n;
}
void Camera::setGain(float f){
    this->gain = f;
}
void Camera::setThreshold(int n){
    this->threshold = n;
}

} // namespace img_server