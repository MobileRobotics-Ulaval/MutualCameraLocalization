// Les threads utilisent std:thread et donc il est nécessaire d'utlisé c++11 rajouter le flags  "-std=c++11" à la compilation

#include "Client.h"

using namespace std;

/**
    Initiation of the client socket
*/
Client::Client(): recording(false){
    image_transport::ImageTransport it(nh);
    
    string address, cubeid;
    int port;
    ros::param::get("~address", address);
    ros::param::get("~port", port);

    ROS_INFO("Addresse: %s", address.c_str());
    ROS_INFO("port: %i", port);

    pub = it.advertiseCamera("/camera/image_raw", 1);
    //pub = it.advertiseCamera("/camera/image_raw", 1);
    //pub = nh.advertise<sensor_msgs::Image>("/cube_feed_topic", 1);
    //if(!nh.ok()){
    //    printf("Not ok");
    //}

	comSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (comSocket < 0) 
        ROS_ERROR("ERROR opening socket");
        
   struct hostent *host = gethostbyname(address.c_str());
    if (host == NULL) 
        ROS_ERROR("ERROR, no such host");
        
    bzero((char *) &server, sizeof(server));
    server.sin_family = AF_INET;
    bcopy((char *)host->h_addr, (char *)&server.sin_addr.s_addr, host->h_length);
    server.sin_port = htons(port);
    int errorCode = connect(comSocket,(struct sockaddr *) &server, sizeof(server));
    if(errorCode < 0){
        //TODO add try again
        ROS_ERROR("ERROR connecting to socket, %i", errorCode);
        exit(0);
    }
}


/**
   Main loop
*/
int Client::startListeningLoop(){
    pthread_t imgGathering;
    bool flag = true;
    do{
        printf("\n==== MENU ====\n1. Start recording\n2. Stop recording\n3. Change Shutter\n4. Change Threshold\n5. Quit\nTo do? ");
        string s = "";
        int i;
        getline(cin, s);
        istringstream myStream(s);
        if(!(myStream >> i)){
            printf("Invalid answer!(%s)\n", s.c_str());
            continue;
        }
        if(i == 1){ 
            if(recording){
                printf("Stop recording before exiting\n");
                continue;
            }
            printf("Send start_recording command\n");
            dotCapture::Command d;
            d.add_option()->set_type(dotCapture::Command::START_RECORDING);
            sendCommand(d);
            recording = true;

            printf("Start imgLoop thread\n");
            pthread_create(&imgGathering, 0, Client::callReceivingImgLoopFunction, this);

        }
        else if(i == 2){ 
            if(!recording){
                printf("Not currently recording!!\n");
                continue;
            } 

            printf("Send stop_recording command\n");
            dotCapture::Command d;
            d.add_option()->set_type(dotCapture::Command::STOP_RECORDING);
            sendCommand(d);
            recording = false;

            printf("Join imgGathering thread...\n");
            pthread_join(imgGathering, NULL);
        }
        else if(i == 3){ 
            dotCapture::Command d;
            dotCapture::Command_Option* o = d.add_option();
            o->set_type(dotCapture::Command::SHUTTER);

            printf("What Value?");
            float v;
            getline(cin, s);
            istringstream myStream(s);
            myStream >> v;
            o->set_value(v);
            printf("Sending command SHUTTER with v=%.3f\n", v);
            sendCommand(d);
        }
        else if(i == 4){ 
            dotCapture::Command d;
            dotCapture::Command_Option* o = d.add_option();
            o->set_type(dotCapture::Command::THRESHOLD);

            printf("What Value?");
            float v;
            getline(cin, s);
            istringstream myStream(s);
            myStream >> v;
            o->set_value(v);
            printf("Sending command THRESHOLD with v=%.3f\n", v);
            sendCommand(d);
        }
        else if(i == 5){ 
            dotCapture::Command d;
            // In case we are still recording.
            if(recording){
                d.add_option()->set_type(dotCapture::Command::STOP_RECORDING);
                sendCommand(d);
                dotCapture::Command d2;
                d2.add_option()->set_type(dotCapture::Command::DISCONNECT);
                sendCommand(d2);
                recording = false;

                printf("Join imgGathering thread...\n");
                pthread_join(imgGathering, NULL);
            }
            else{
                d.add_option()->set_type(dotCapture::Command::DISCONNECT);
                sendCommand(d);
            }
            flag = false;

        }

    }while(flag);
    ROS_INFO("End of program");

    close(comSocket);

    return 0; 
}



/**
    Send command to the server
*/
int Client::sendCommand(dotCapture::Command &com){
    int varintsize = ::google::protobuf::io::CodedOutputStream::VarintSize32(com.ByteSize());
    int ackSize = com.ByteSize()+varintsize;
    char* ackBuf = new char[ackSize];
     
    //write varint delimiter to buffer
    ::google::protobuf::io::ArrayOutputStream arrayOut(ackBuf, ackSize);
    ::google::protobuf::io::CodedOutputStream codedOut(&arrayOut);
    codedOut.WriteVarint32(com.ByteSize());
     
    //write protobuf ack to buffer
    com.SerializeToCodedStream(&codedOut);
    int c = send(comSocket, ackBuf, ackSize, 0);
    if(c < 0){
        ROS_ERROR("Error: Impossible to send packet");
    }
    delete(ackBuf);
}

/*
   Reads a varint delimited protocol buffers message from a TCP socket
   returns message in buffer, and returns number of bytes read (not including delimiter)
*/
int recvDelimProtobuf(int sock, unsigned char **buffer){
    //read the delimiting varint byte by byte
    unsigned int length=0;
    int recv_bytes=0;
    char bite;
    int received = recv(sock, &bite, 1, 0);
    if(received < 0)
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

  //  if(recv_bytes >= length)
  //      printf("Bytes receive excess (%i) = (%x)\n length(%i)\n", recv_bytes, recv_bytes, length);
    return recv_bytes;
}

/*
   Starting function of the image gathering Thread.
   It receives each packet and extract the image.
   The raw image is convert to openCV than to a ROS message
*/
void* Client::receivingImgLoop(){
    unsigned char *buffer;
    unsigned char* data;
    dotCapture::Img message;
    int i = 0;
    int sizeLz4;


    //unsigned char** pngBuf = new unsigned char*();
   // size_t pngSize;

    camera_info_manager::CameraInfoManager m(nh, "narrow_stereo", "package://client_cube/calibration.ini");
    sensor_msgs::CameraInfo ros_camInfo = m.getCameraInfo();
    WIDTH = ros_camInfo.width;
    HEIGHT = ros_camInfo.height;

    unsigned char* iz4BuffDecod = (unsigned char*)malloc(WIDTH * HEIGHT);

    // Initiation of the Opencv-ROS bridge
    cv_bridge::CvImage cv_image;
    cv_image.encoding = "mono8";
    IplImage* pImg;
    pImg = cvCreateImageHeader(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
    sensor_msgs::Image ros_image;

    printf("\n");

    ros::Rate loop_rate(30);
    printf("Start recording!!!\n");
    while(recording){
        //printf("Receiving image...\n");
        int received = recvDelimProtobuf(comSocket, &buffer);
        // printf("-");
         
        //read varint delimited protobuf object in to buffer
        google::protobuf::io::ArrayInputStream arrayIn(buffer, received);
        google::protobuf::io::CodedInputStream codedIn(&arrayIn);
        google::protobuf::io::CodedInputStream::Limit msgLimit = codedIn.PushLimit(received);
        if(!message.ParseFromCodedStream(&codedIn)){
            ROS_ERROR("Can't parse img");
            exit(0);
        }
        codedIn.PopLimit(msgLimit);


        //printf("Timestamp: %i\n Micro: %i\n", message.timestamp(), message.timestamp_microsec());
        data = (unsigned char*)(const void *)message.mutable_image()->c_str();

        sizeLz4 = LZ4_decompress_safe((char*)(void*)(data), (char*)(void*)(iz4BuffDecod), message.mutable_image()->size(), WIDTH * HEIGHT);
        
        ROS_INFO("#%i img  %do", i, (int)message.mutable_image()->size());
        
        
        cvSetData(pImg, iz4BuffDecod, pImg->widthStep);

        cv_image.image = pImg;
        
        // ---------- TO CHANGE FOR THE ACTUAL TIME ON THE DEVICE!!! ---------
        ros::Time time_on_device(message.timestamp(), message.timestamp_microsec());
        //cv_image.header.stamp = ros::Time::now();
        cv_image.header.stamp = time_on_device;

        cv_image.toImageMsg(ros_image);
        pub.publish(ros_image, ros_camInfo);

        //compressToPNG(pngBuf, &pngSize, iz4BuffDecod, sizeLz4);
        //saveFilePNG(*pngBuf, pngSize, "fromGoogleWithLove.png");
        i++;
        //ros::spinOnce();
        //loop_rate.sleep();
        free(buffer);
    }
    ROS_INFO("Img total =%i", i);
}


/**
    Debugging fonction, send a image from a PNG file
*/
void Client::foo(){
    unsigned char *buffer;
    unsigned char* data;
    dotCapture::Img message;
    int i = 0;
    int sizeLz4;


    unsigned char* pngBuf;
    size_t pngSize;

    camera_info_manager::CameraInfoManager m(nh, "narrow_stereo", "package://client_cube/calibration.ini");
    sensor_msgs::CameraInfo ros_camInfo = m.getCameraInfo();
    WIDTH = ros_camInfo.width;
    HEIGHT = ros_camInfo.height;

    unsigned char* iz4BuffDecod = (unsigned char*)malloc(WIDTH * HEIGHT);

    // Initiation of the Opencv-ROS bridge
    cv_bridge::CvImage cv_image;
    cv_image.encoding = "mono8";
    IplImage* pImg;
    pImg = cvCreateImageHeader(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
    sensor_msgs::Image ros_image;



    printf("\n");
    ros::Rate loop_rate(30);

    unsigned width, height;

    string filename;
    ros::param::get("~filename", filename);
    unsigned error = lodepng_decode_file(&pngBuf, &width, &height, filename.c_str(), LCT_GREY,  8);
    if(error) ROS_ERROR("error %u: %s", error, lodepng_error_text(error));

    cvSetData(pImg, pngBuf, pImg->widthStep);

    cv_image.image = pImg;
    while(ros::ok()){

        cv_image.header.stamp = ros::Time::now();

        cv_image.toImageMsg(ros_image);
        pub.publish(ros_image, ros_camInfo);

        //compressToPNG(pngBuf, &pngSize, iz4BuffDecod, sizeLz4);
        //saveFilePNG(*pngBuf, pngSize, "fromGoogleWithLove.png");
        i++;

        ros::spinOnce();
        loop_rate.sleep();
    }
    free(pngBuf);
    ROS_INFO("Images total =%i", i);
}



/*
   Convert a char array to PNG
*/
void Client::compressToPNG(unsigned char**& pngBuf, size_t* pngSize, const unsigned char *imgBuf, const long int size){
    int error = lodepng_encode_memory(pngBuf, pngSize, imgBuf, WIDTH, HEIGHT, LCT_GREY,  8);

    if(error){
        ROS_ERROR("Error %u: %s", error, lodepng_error_text(error));
        exit(0);
    }
    //printf("Compression success!!\npng=%.3fko raw=%.3fko\n", (float)*pngSize/1000.f, (float)size/1000.f);
}

/*
   Save a PNG in memory to a file
*/
void Client::saveFilePNG(unsigned char* pngBuf, size_t pngSize, string filename){
    lodepng_save_file(pngBuf, pngSize, filename.c_str());
}


Client::~Client(){}


