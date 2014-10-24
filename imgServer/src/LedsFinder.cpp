#include "LedsFinder.h"

using namespace std;

namespace img_server
{
/**
    Initiation of the server
*/
LedsFinder::LedsFinder(int portTCP, int portUDP, int threshold, float shutterTime, int brightness, int exposure, float gain, bool pIsTcp):
                    camera(threshold, shutterTime, brightness, exposure, gain),
                    recording(false), 
                    isTcp(pIsTcp){

    this->serverTCP.init(portTCP);
    if(isTcp){
        printf("Mode TCP\n");
        this->serverUDP = new SocketTCP;
    }
    else
        printf("Mode UDP\n");
        this->serverUDP = new SocketUDP;
    }
    this->serverUDP->init(portUDP);
    //serverUDP.connect();
    printf("[SERVER] Start image server on port TCP: %i port UDP: %i...\n", portTCP, portUDP);
}


/**
    Main Server loop
*/
void LedsFinder::startServerLoop(){
    while(true){
        this->waitingForClient();
        this->waitingForCommandFromClient();
    }
}

void LedsFinder::waitingForClient(){
    while(true){
        printf("\n\n[SERVER] Waiting for a client to connect\n");
        this->serverTCP.connect();

        if(!this->serverTCP.bindAndAccept()){
            printf("Retry binding in 5s... \n");
            sleep(5);
            continue;
        }
        else{
            printf("[SERVER] Client connected\n");
            return;;
        }
    }
}

void LedsFinder::waitingForCommandFromClient(){
    bool flag = true;
    while(flag){
        dotCapture::Command* c = this->getCommand();
        printf("[SERVER] Command receive:\n");
        printf("%s", c->option(0).DebugString().c_str());
        for(int i = 0; i < c->option_size(); i++){
            dotCapture::Command_Option o = c->option(i);

            if(o.type() == dotCapture::Command::START_RECORDING){
                //printf("[SERVER] Start Recording\n");
                recording = true;

                pthread_create(&imgGathering, 0, LedsFinder::callLoopRecordingFunction, this);
            }
            else if(o.type() == dotCapture::Command::STOP_RECORDING){
                //printf("[SERVER] Stop Recording\n");
                recording = false;
                pthread_join(imgGathering, NULL);

            }
            else if(o.type() == dotCapture::Command::DISCONNECT){
                //printf("[SERVER] Disconnecting client\n");
                flag = false;
            }
            else if(o.type() == dotCapture::Command::SHUTTER){
                //printf("[SERVER] Change shutter\n");
                pthread_mutex_lock(&proprietyMux);
                this->camera.setShutter(o.value());
                if(recording){
                    pthread_mutex_lock(&proprietyMux);
                    this->camera.configureProperties();
                    pthread_mutex_unlock(&proprietyMux);
                }
            }
            else if(o.type() == dotCapture::Command::BRIGHTNESS){
                //printf("[SERVER] Change brightness\n");
                this->camera.setBrightness(o.value());
                if(recording){
                    pthread_mutex_lock(&proprietyMux);
                    this->camera.configureProperties();
                    pthread_mutex_unlock(&proprietyMux);
                }
            }
            else if(o.type() == dotCapture::Command::EXPOSURE){
                //printf("[SERVER] Change exposure\n");
                this->camera.setExposure(o.value());
                if(recording){
                    pthread_mutex_lock(&proprietyMux);
                    this->camera.configureProperties();
                    pthread_mutex_unlock(&proprietyMux);
                }
            }
            else if(o.type() == dotCapture::Command::GAIN){
                //printf("[SERVER] Change gain\n");
                this->camera.setGain(o.value());
                if(recording){
                    pthread_mutex_lock(&proprietyMux);
                    this->camera.configureProperties();
                    pthread_mutex_unlock(&proprietyMux);
                }
            }
            else if(o.type() == dotCapture::Command::THRESHOLD){
                printf("[SERVER] Change threshold\n");
                this->camera.setThreshold(o.value());
                if(recording){
                    pthread_mutex_lock(&proprietyMux);
                    this->camera.configureProperties();
                    pthread_mutex_unlock(&proprietyMux);
                }
            }

        }

    }

    this->serverTCP.disconnect();
    serverUDP.disconnect();
}



/*
   Starting function of the image recording Thread.
   It takes, compress and sends picture
*/
void* LedsFinder::loopRecording(){
    int sizeLz4;
    int i = 0;
    int d1, d2, d3, d4;
    struct timeval t1, t2, t3, t4, t5, t6;
    long elapsed1, elapsed2, elapsed3, elapsed4;
    float fps;
    //times_t t1, t2, t3, t4;

    unsigned char* izBuff = (unsigned char*)malloc(WIDTH * HEIGHT/4);
    unsigned char* res = (unsigned char*)malloc(WIDTH * HEIGHT/4);
    //unsigned char* decod = (unsigned char*)malloc(WIDTH * HEIGHT);
    
    // If the camera is not connected, the thread is stop

   while(this->camera.initCamera() != 0){
       printf("Waiting 5s and trying again...\n");
       sleep(5);
   }

    //Init the UDP connection by waiting on the client:
   if(isTCP){
        serverUDP.connect();
        serverUDP.bindAndAccept();
   }
   else{
        serverUDP.bindAndAccept();
        serverUDP.toReceive((char * )izBuff, 100, 0);
   }


    gettimeofday(&t6, 0);
    while(recording){
        gettimeofday(&t1, 0);

        pthread_mutex_lock(&proprietyMux);

        this->camera.takeRawPicture();
        //IF YOU DONT HAVE A CAMERA
        //this->camera.takeFakePicture(); 

        gettimeofday(&t2, 0);

        this->camera.doThreshold();

        pthread_mutex_unlock(&proprietyMux);
        gettimeofday(&t3, 0);

        this->camera.divByTwoRes(res, WIDTH, HEIGHT);
        gettimeofday(&t4, 0);

        //compressToPNG(pngBuf, &pngSize, rawImage.GetData(), rawImage.GetDataSize());
        sizeLz4 = LZ4_compress((char*)(void*)(res), (char*)(void*)(izBuff), WIDTH * HEIGHT/4);

        i++;
        gettimeofday(&t5, 0);
        sendProto(dataToProto(t5.tv_sec, t6.tv_usec, izBuff, sizeLz4));

        gettimeofday(&t6, 0);

//      INCASE OF DEBUGING
        /*compressToPNG(pngBuf, &pngSize, rawImage.GetData(), rawImage.GetDataSize());
        saveFilePNG(*pngBuf, pngSize, "beforeBehindSend.png");*/
    }
    printf("[THREAD] Images total =%i\n", i);
    this->camera.stopRecording();

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
int LedsFinder::recvDelimProtobuf(unsigned char **buffer){
    //read the delimiting varint byte by byte
    unsigned int length=0;
    int recv_bytes=0;
    char bite;
    int received=this->serverTCP.toReceive(&bite, 1, 0);
    if(received<0)
        return received;
    else
        recv_bytes += received;
    length = (bite & 0x7f);
    while(bite & 0x80){
        memset(&bite, 0, 1);
        received=serverTCP.toReceive(&bite, 1, 0);
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
        received=serverTCP.toReceive((char *)(*buffer + (sizeof(unsigned char) * recv_bytes)), length-recv_bytes, 0);
        if(received<0)
            return received;
        else
            recv_bytes+=received;
    }
    return recv_bytes;
}

/* 
    Wait for a command from the client
*/
dotCapture::Command*  LedsFinder::getCommand(){
    
    printf("\n\n[SERVER] Waiting for command\n");

	dotCapture::Command* com = new dotCapture::Command();
    unsigned char *buffer;

    int received = recvDelimProtobuf(&buffer);
     
    //read varint delimited protobuf object in to buffer
    google::protobuf::io::ArrayInputStream arrayIn(buffer, received);
    google::protobuf::io::CodedInputStream codedIn(&arrayIn);
    google::protobuf::io::CodedInputStream::Limit msgLimit = codedIn.PushLimit(received);
    com->ParseFromCodedStream(&codedIn);
    codedIn.PopLimit(msgLimit);

    free(buffer);
    
    //close(comSocketTCP);
   // close(socketFileDescriptorTCP);
    return com;
}


/*
   Send Image protobuf to the client
*/
void LedsFinder::sendProto(dotCapture::Img* message){
    google::protobuf::io::ArrayOutputStream arr(bigBuffer, sizeof(bigBuffer));
    google::protobuf::io::CodedOutputStream output(&arr);
                 
    output.WriteVarint32(message->ByteSize());
    message->SerializeToCodedStream(&output);
     
    // repeat if more messages can fit into the buffer
     
    int nSent = this->serverUDP.toSend( (char*)bigBuffer, output.ByteCount(), 0);
	// int varintsize = ::google::protobuf::io::CodedOutputStream::VarintSize32(message->ByteSize());
	// int ackSize = message->ByteSize() + varintsize;
	// char* ackBuf = new char[ackSize];
	 
	// //write varint delimiter to buffer
	// ::google::protobuf::io::ArrayOutputStream arrayOut(ackBuf, ackSize);
	// ::google::protobuf::io::CodedOutputStream codedOut(&arrayOut);
	// codedOut.WriteVarint32(message->ByteSize());
	 
	// //write protobuf ack to buffer
	// message->SerializeToCodedStream(&codedOut);
	// serverUDP.toSend(ackBuf, ackSize, 0);
	// delete(ackBuf);
}


long LedsFinder::getTimeInMilliseconds()
{
    struct timespec t;
    //clock_gettime(CLOCK_REALTIME, &t);
    // The + 0.5 rounds the milliseconds value
    return (t.tv_sec*1000 + t.tv_nsec/1.0e6) + 0.5;
}

LedsFinder::~LedsFinder(){
    //close(sClient);
}



} // img_server