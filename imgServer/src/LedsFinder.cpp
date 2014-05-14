#include "LedsFinder.h"

using namespace std;

/**
    Initiation of the server
*/
LedsFinder::LedsFinder(int port, int t, float s, int b, int e, float g) : port(port), threshold(t), shutter(s), brightness(b), exposure(e), gain(g),recording(false){
	bzero((char *) &ClientAddress, sizeof(ClientAddress));

    ClientAddress.sin_family = AF_INET;
    ClientAddress.sin_addr.s_addr = INADDR_ANY;
    ClientAddress.sin_port = htons(port);

    //pthread_cancel(imgGathering);
    //pthread_mutex_unlock(&recordingMux);
}
/**
    Reduce the resolution of an image by a factor 2
*/
inline void LedsFinder::divByTwoRes(unsigned char* p, unsigned char* c, int w, int h){
    int nw = w/2;
    int nh = h/2;
    //unsigned char* c = (unsigned char*)malloc(w * h/4);

    printf("%i\n", 2 * 200 * h + 2 * 100);
    printf("%i\n", 2 * 200 * h + 2 * 100 + 1);
    printf("%i\n", (2 * 200 + 1) * h + 2 * 100);
    printf("%i\n", (2 * 200 + 1) * h + 2 * 100 + 1);
    for(int i = 0; i < nw; i++){
        for(int j = 0; j < nh; j++){// Way too slow
            //printf("%i\n", (2 * i * w + 2 * j));
            c[j * nw + i]  =(p[2 * j * w + 2 * i] +
                             p[2 * j * w + 2 * i + 1] +
                             p[(2 * j + 1) * w + 2 * i] +
                             p[(2 * j + 1) * w + 2 * i + 1])/4;
        }
    }
   // p = c;
}

/**
    Pass a raw image throught a threshold
*/
inline void LedsFinder::doThreshold(Image& Img, int t){
	unsigned char* p = Img.GetData();
	unsigned int size = Img.GetDataSize();
	for(int i = 0; i < size; i++){
		if(p[i] < t)
			p[i] = 0;
	}
}

/**
    Debug function for camera testing
*/
void LedsFinder::takeRawPicture(int nbrPic, int threshold){
	Image rawImage;
	PNGOption pngOption;
	pngOption.interlaced = false;
	pngOption.compressionLevel = 0;

	this->startRecording();
    unsigned char** pngBuf = new unsigned char*();
    size_t pngSize;
    unsigned char* izBuff = (unsigned char*)malloc(WIDTH * HEIGHT);
    unsigned char* decod = (unsigned char*)malloc(WIDTH * HEIGHT);
    unsigned char* res = (unsigned char*)malloc(WIDTH * HEIGHT/4);

	for (int i=0; i < nbrPic; i++){
		Error err = cam.RetrieveBuffer( &rawImage );
        printf("fuck");
		doThreshold(rawImage, threshold);
        divByTwoRes(rawImage.GetData(), res, WIDTH, HEIGHT);

        
        int e = LZ4_compress((char*)(void*)(res), (char*)(void*)(izBuff), rawImage.GetDataSize()/4);

        printf("Size =%i\n", e);
        int sizeDecom = LZ4_decompress_safe((char*)(void*)(izBuff), (char*)(void*)(decod), e, WIDTH * HEIGHT/4);
        printf("Sizedecod =%i\n", sizeDecom);    
        compressToPNG(pngBuf, &pngSize, decod, WIDTH/2, HEIGHT/2);    
        saveFilePNG(*pngBuf, pngSize, "camarche2.png");
	} 
	this->stopRecording();

}

/**
    Main Server loop
*/
void LedsFinder::startProcessingLoop(){
    while(true){
        //Creation of the socket
        int socketFileDescriptor;
        struct sockaddr_in clientAddress;
        socklen_t clientAdressLength;
        
        socketFileDescriptor = socket(AF_INET, SOCK_STREAM, 0);
        this->checkForErrorTCP(socketFileDescriptor, "ERROR opening socket");
        
        int comErrorCode = bind(socketFileDescriptor, (struct sockaddr *) &ClientAddress, sizeof(ClientAddress));
        if(comErrorCode != 0){
            this->checkForErrorTCP(comErrorCode, "ERROR on binding");
            close(comSocket);
            close(socketFileDescriptor);
            return;
        }
        
        listen(socketFileDescriptor, 5);
        
        clientAdressLength = (socklen_t) sizeof(clientAddress);
        comSocket = accept(socketFileDescriptor, (struct sockaddr *) &clientAddress, &clientAdressLength);
        this->checkForErrorTCP(comSocket, "ERROR on accept");


        printf("[SERVER] Client connected\n");
        bool flag = true;
        do{
            dotCapture::Command* c = getCommand();
            printf("[SERVER] Command receive:\n");
            printf("%s", c->option(0).DebugString().c_str());
            for(int i = 0; i < c->option_size(); i++){
                dotCapture::Command_Option o = c->option(i);

                if(o.type() == dotCapture::Command::START_RECORDING){
                    //printf("[SERVER] Start Recording\n");
                    setRecording(true);

                    pthread_create(&imgGathering, 0, LedsFinder::callLoopRecordingFunction, this);
                }
                else if(o.type() == dotCapture::Command::STOP_RECORDING){
                    //printf("[SERVER] Stop Recording\n");
                    setRecording(false);
                    pthread_join(imgGathering, NULL);

                }
                else if(o.type() == dotCapture::Command::DISCONNECT){
                    //printf("[SERVER] Disconnecting client\n");
                    flag = false;
                }
                else if(o.type() == dotCapture::Command::SHUTTER){
                    //printf("[SERVER] Change shutter\n");
                    pthread_mutex_lock(&proprietyMux);
                    shutter = o.value();
                    if(isRecording())
                        configureProperties();
                    pthread_mutex_unlock(&proprietyMux);
                }
                else if(o.type() == dotCapture::Command::BRIGHTNESS){
                    //printf("[SERVER] Change brightness\n");
                    pthread_mutex_lock(&proprietyMux);
                    brightness = o.value();
                    if(isRecording())
                        configureProperties();
                    pthread_mutex_unlock(&proprietyMux);
                }
                else if(o.type() == dotCapture::Command::EXPOSURE){
                    //printf("[SERVER] Change exposure\n");
                    pthread_mutex_lock(&proprietyMux);
                    exposure = o.value();
                    if(isRecording())
                        configureProperties();
                    pthread_mutex_unlock(&proprietyMux);
                }
                else if(o.type() == dotCapture::Command::GAIN){
                    //printf("[SERVER] Change gain\n");
                    pthread_mutex_lock(&proprietyMux);
                    gain = o.value();
                    if(isRecording())
                        configureProperties();
                    pthread_mutex_unlock(&proprietyMux);
                }
                else if(o.type() == dotCapture::Command::THRESHOLD){
                    //printf("[SERVER] Change threshold\n");
                    pthread_mutex_lock(&proprietyMux);
                    threshold = o.value();
                    if(isRecording())
                        configureProperties();
                    pthread_mutex_unlock(&proprietyMux);
                }

            }

        } while(flag);

        close(comSocket);
        close(socketFileDescriptor);
    } 
}

/*
   Setter/getter of the recording flag
*/
bool LedsFinder::isRecording(){
    bool r;
    printf("[isrecord");
    pthread_mutex_lock(&recordingMux);
    r = recording;
    printf("u");
    pthread_mutex_unlock(&recordingMux);
    printf("]\n");
    return r;
}

void LedsFinder::setRecording(bool r){
    printf("[setrecord");
    pthread_mutex_lock(&recordingMux);
    recording = r;
    printf("u");
    pthread_mutex_unlock(&recordingMux);
    printf("]\n");
}

/*
   Starting function of the image recording Thread.
   It takes, compress and sends picture
*/
void* LedsFinder::loopRecording(){
    Image rawImage;
    int sizeLz4;
    int i = 0;
    int d1, d2, d3, d4;
    struct timeval t1, t2, t3, t4, t5;
    long elapsed1, elapsed2, elapsed3, elapsed4;
    //times_t t1, t2, t3, t4;

    unsigned char* izBuff = (unsigned char*)malloc(WIDTH * HEIGHT/4);
    unsigned char* res = (unsigned char*)malloc(WIDTH * HEIGHT/4);
    //unsigned char* decod = (unsigned char*)malloc(WIDTH * HEIGHT);
    
    // If the camera is not connected, the thread is stop
    if(this->startRecording() == 0){
        while(isRecording()){
            //t1 = std::chrono::steady_clock::now();
            gettimeofday(&t1, 0);

            pthread_mutex_lock(&proprietyMux);
            Error err = cam.RetrieveBuffer( &rawImage );
            //t2 = std::chrono::steady_clock::now();
            gettimeofday(&t2, 0);

            this->doThreshold(rawImage, threshold);
            pthread_mutex_unlock(&proprietyMux);
           // t3 = std::chrono::steady_clock::now();
            gettimeofday(&t3, 0);

            divByTwoRes(rawImage.GetData(), res, WIDTH, HEIGHT);
            gettimeofday(&t4, 0);

            //compressToPNG(pngBuf, &pngSize, rawImage.GetData(), rawImage.GetDataSize());
            sizeLz4 = LZ4_compress((char*)(void*)(res), (char*)(void*)(izBuff), WIDTH * HEIGHT/4);
           // t4 = std::chrono::steady_clock::now();
            gettimeofday(&t5, 0);
            
           //d1 = (new millisecs_t( std::chrono::duration_cast<millisecs_t>(t2-t1) ))->count();
            //d2 = (new millisecs_t( std::chrono::duration_cast<millisecs_t>(t3-t2) ))->count();
            ///d3 = (new millisecs_t( std::chrono::duration_cast<millisecs_t>(t4-t3) ))->count();

            //d4 = d1 + d2 + d3;
           // printf("#%i - Retrieve cam: %ims \nThreshold: %ims \nCompresse: %ims\nTotal: %ims\n size: %i Octets\n", i, d1, d2, d3, d4, sizeIz4);
            i++;
            FlyCapture2::TimeStamp times = rawImage.GetTimeStamp();
            sendProto(dataToProto(times.seconds, times.microSeconds, izBuff, sizeLz4));
            long elapsed1 = (t2.tv_sec-t1.tv_sec)*1000000 + t2.tv_usec-t1.tv_usec;
            long elapsed2 = (t3.tv_sec-t2.tv_sec)*1000000 + t3.tv_usec-t2.tv_usec;
            long elapsed3 = (t4.tv_sec-t3.tv_sec)*1000000 + t4.tv_usec-t3.tv_usec;
            long elapsed4 = (t5.tv_sec-t4.tv_sec)*1000000 + t5.tv_usec-t4.tv_usec;
            
            printf("Time pass: \n%i\n%i\n%i\n%i\n", elapsed1 ,elapsed2, elapsed3, elapsed4);
            //sendProto(dataToProto(rawImage.GetTimeStamp().seconds, rawImage.GetTimeStamp().microSeconds, *pngBuf, pngSize));
            

    //      INCASE OF DEBUGING
            /*compressToPNG(pngBuf, &pngSize, rawImage.GetData(), rawImage.GetDataSize());
            saveFilePNG(*pngBuf, pngSize, "beforeBehindSend.png");*/
        }
        printf("[THREAD] Images total =%i\n", i);
        this->stopRecording();
    }
}


/*
   Create a new Protobuf to encapsulate the data
*/
dotCapture::Img* LedsFinder::dataToProto(long timestamp, int timestamp_microsec, unsigned char* data, unsigned long int size){
	dotCapture::Img* message = new dotCapture::Img();
	
	message->set_timestamp(timestamp);
	message->set_timestamp_microsec(timestamp_microsec);
	message->set_image(data, size);
	return message;
}


/*
   reads a varint delimited protocol buffers message from a TCP socket
   returns message in buffer, and returns number of bytes read (not including delimiter)
*/
int recvDelimProtobuf(int sock, unsigned char **buffer){
    //read the delimiting varint byte by byte
    unsigned int length=0;
    int recv_bytes=0;
    char bite;
    int received=recv(sock, &bite, 1, 0);
    if(received<0)
        return received;
    else
        recv_bytes += received;
    length = (bite & 0x7f);
    while(bite & 0x80){
        memset(&bite, 0, 1);
        received=recv(sock, &bite, 1, 0);
        if(received<0)
            return received;
        else
            recv_bytes += received;
        length|= (bite & 0x7F) << (7*(recv_bytes-1));
    }
 
    //receive remainder of message
    recv_bytes=0;
    *buffer=(unsigned char *)malloc(sizeof(unsigned char) * length);
    while(recv_bytes < length){
        received=recv(sock, *buffer + (sizeof(unsigned char) * recv_bytes), length-recv_bytes, 0);
        if(received<0)
            return received;
        else
            recv_bytes+=received;
    }
    return recv_bytes;
}

/*
   reads a varint delimited protocol buffers message from a TCP socket
   returns message in buffer, and returns number of bytes read (not including delimiter)
*/
dotCapture::Command*  LedsFinder::getCommand(){
    
    printf("\n\n[SERVER] Waiting for command\n");

	dotCapture::Command* com = new dotCapture::Command();
    unsigned char *buffer;

    int received = recvDelimProtobuf(comSocket, &buffer);
     
    //read varint delimited protobuf object in to buffer
    google::protobuf::io::ArrayInputStream arrayIn(buffer, received);
    google::protobuf::io::CodedInputStream codedIn(&arrayIn);
    google::protobuf::io::CodedInputStream::Limit msgLimit = codedIn.PushLimit(received);
    com->ParseFromCodedStream(&codedIn);
    codedIn.PopLimit(msgLimit);

    free(buffer);
    
    //close(comSocket);
   // close(socketFileDescriptor);
    return com; 
}


/*
   Send Image protobuf to the client
*/
void LedsFinder::sendProto(dotCapture::Img* message){
	int varintsize = ::google::protobuf::io::CodedOutputStream::VarintSize32(message->ByteSize());
	int ackSize = message->ByteSize() + varintsize;
	char* ackBuf = new char[ackSize];
	 
	//write varint delimiter to buffer
	::google::protobuf::io::ArrayOutputStream arrayOut(ackBuf, ackSize);
	::google::protobuf::io::CodedOutputStream codedOut(&arrayOut);
	codedOut.WriteVarint32(message->ByteSize());
	 
	//write protobuf ack to buffer
	message->SerializeToCodedStream(&codedOut);
	int c = send(comSocket, ackBuf, ackSize, 0);
	if(c < 0)
		printf("Error: Impossible d'envoyer");
	delete(ackBuf);
}

/*
   Initiate the camera's parameter
*/
int LedsFinder::startRecording(){
    BusManager busMgr;
    PGRGuid guid;
    if(busMgr.GetCameraFromIndex(0, &guid) != PGRERROR_OK){
    	printf("Aucune caméra connecté.\n");
    	//exit(0);
        return -1;
    }
    this->printErrorCam(cam.Connect(&guid));
    this->configureProperties();
    this->printErrorCam(cam.StartCapture());
    
    // Sometimes, the first image is invalid. This way, we get rid of it.
    Image rawImage;
    this->printErrorCam(cam.RetrieveBuffer( &rawImage ));
    return 0;
}

/*
   Update the camera's parameters
*/
void LedsFinder::configureProperties()
{
    Property property;
    
    property.type = SHUTTER;
    this->printErrorCam(cam.GetProperty( &property ));
    property.onOff = true;
    property.absControl = true;
    property.autoManualMode = false;
    property.absValue = shutter;
    this->printErrorCam(cam.SetProperty( &property ));
    
    property.type = BRIGHTNESS;
    this->printErrorCam(cam.GetProperty( &property ));
    property.onOff = true;
    property.autoManualMode = false;
    property.absValue = brightness;
    this->printErrorCam(cam.SetProperty( &property ));
    
    property.type = AUTO_EXPOSURE;
    this->printErrorCam(cam.GetProperty( &property ));
    property.onOff = true;
    property.autoManualMode = false;
    property.absValue = exposure;
    this->printErrorCam(cam.SetProperty( &property ));
    
    property.type = GAIN;
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
void LedsFinder::stopRecording()
{
    this->printErrorCam(cam.StopCapture());
    this->printErrorCam(cam.Disconnect());
}


long LedsFinder::getTimeInMilliseconds()
{
    struct timespec t;
    //clock_gettime(CLOCK_REALTIME, &t);
    // The + 0.5 rounds the milliseconds value
    return (t.tv_sec*1000 + t.tv_nsec/1.0e6) + 0.5;
}

/*
   Convert a char array to PNG
*/
void LedsFinder::compressToPNG(unsigned char**& pngBuf, size_t* pngSize, const unsigned char *imgBuf, const int w, const int h){
    int error = lodepng_encode_memory(pngBuf, pngSize, imgBuf, w, h, LCT_GREY,  8);

    if(error){
        printf("Error %u: %s\n", error, lodepng_error_text(error));
        exit(0);
    }
    printf("Compression success!!\npng=%.3fko raw=%.3fko\n", (float)*pngSize/1000.f, (float)(w*h)/1000.f);
}

/*
   Save a PNG in memory to a file
*/
void LedsFinder::saveFilePNG(unsigned char* pngBuf, size_t pngSize, string filename){
    lodepng_save_file(pngBuf, pngSize, filename.c_str());
}

/*
   Error output functions
*/
void LedsFinder::printErrorCam( Error error ){
    if (error != PGRERROR_OK)
        error.PrintErrorTrace();
}

void LedsFinder::checkForErrorTCP(int errorCode, const char* errorMessage){
    if (errorCode < 0) 
    	perror(errorMessage);
}


LedsFinder::~LedsFinder(){
    //close(sClient);
}



