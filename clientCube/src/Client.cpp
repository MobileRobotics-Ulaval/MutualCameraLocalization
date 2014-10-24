// Les threads utilisent std:thread et donc il est nécessaire d'utlisé c++11 rajouter le flags  "-std=c++11" à la compilation

#include "Client.h"

using namespace std;

/**
    Initiation of the client socket
*/
Client::Client(): recording(false){
    image_transport::ImageTransport it(nh);
    
    string address;
    int portTCP, portUDP;
    ros::param::get("~address", address);
    ros::param::get("~portTCP", portTCP);
    ros::param::get("~portUDP", portUDP);

    ROS_INFO("Addresse: %s", address.c_str());
    ROS_INFO("portTCP: %i", portTCP);
    ROS_INFO("portUDP: %i", portUDP);

    pub = it.advertiseCamera("camera/image_raw", 1);
    //pub = it.advertiseCamera("/camera/image_raw", 1);
    //pub = nh.advertise<sensor_msgs::Image>("/cube_feed_topic", 1);
    //if(!nh.ok()){
    //    printf("Not ok");
    //}

    clientTCP.init(address, portTCP);
    clientUDP.init(address, portUDP);
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

    clientTCP.disconnect();
    clientUDP.disconnect();

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
    int c = clientTCP.toSend(ackBuf, ackSize, 0);
    if(c < 0){
        ROS_ERROR("Error: Impossible to send packet");
    }
    delete(ackBuf);
}

/*
   Reads a varint delimited protocol buffers message from a TCP socket
   returns message in buffer, and returns number of bytes read (not including delimiter)
*/
dotCapture::Img  Client::recvDelimProtobuf(unsigned char *buffer){
    dotCapture::Img msg;
    //read the delimiting varint byte by byte
    int size = this->clientUDP.toReceive((char*)buffer, 307200, 0);
    ROS_INFO("length %i", size);

    google::protobuf::io::ArrayInputStream arr(buffer, size);
    google::protobuf::io::CodedInputStream input(&arr);

    uint32_t message1_size = 0;
    input.ReadVarint32(&message1_size);
    google::protobuf::io::CodedInputStream::Limit limit = input.PushLimit(message1_size);
    if(!msg.ParseFromCodedStream(&input)){
        ROS_ERROR("Can't parse img");
        exit(0);
    }
    input.PopLimit(limit);
    return msg;
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
    struct timeval current_sys_time;
    struct timeval t1, t2;
    double elapsedTime;


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

    printf("Waiting 3s for the server\n");
    sleep(3);

    ros::Rate loop_rate(30);
    printf("Start recording!!!\n");


    //Init UDP by sending a bit
    printf("please/n");
    clientUDP.toSend((char *)iz4BuffDecod, 100, 0);
    printf("yeah!/n");

    while(recording){
        //printf("Receiving image...\n");bigBuffer
        //int received = recvDelimProtobuf(&buffer); good old buffer
        message = recvDelimProtobuf((unsigned char *)bigBuffer);
        gettimeofday(&current_sys_time, 0);
         
        //read varint delimited protobuf object in to buffer
//        google::protobuf::io::ArrayInputStream arrayIn(buffer, received);
//        google::protobuf::io::CodedInputStream codedIn(&arrayIn);
//        google::protobuf::io::CodedInputStream::Limit msgLimit = codedIn.PushLimit(received);
//        if(!message.ParseFromCodedStream(&codedIn)){
//            ROS_ERROR("Can't parse img");
//            exit(0);
//        }
//        codedIn.PopLimit(msgLimit);


        //printf("Timestamp: %i\n Micro: %i\n", message.timestamp(), message.timestamp_microsec());
        data = (unsigned char*)(const void *)message.mutable_image()->c_str();

        sizeLz4 = LZ4_decompress_safe((char*)(void*)(data), (char*)(void*)(iz4BuffDecod), message.mutable_image()->size(), WIDTH * HEIGHT);


        gettimeofday(&t2, NULL);
        double delay = current_sys_time.tv_sec - message.timestamp() + (current_sys_time.tv_usec - message.timestamp_microsec())/1000000.f;
        ROS_INFO("#%i img  %do Image: %dsec %dmicro delay: %fmicro",
                 i,
                 (int)message.mutable_image()->size(),
                 message.timestamp(),
                 message.timestamp_microsec(),
                 delay);
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
        cout << elapsedTime << " ms.\n";
        gettimeofday(&t1, NULL);

        
        
        cvSetData(pImg, iz4BuffDecod, pImg->widthStep);

        cv_image.image = pImg;
        
        // ---------- TO CHANGE FOR THE ACTUAL TIME ON THE DEVICE!!! ---------
        //ros::Time time_on_device(message.timestamp(), message.timestamp_microsec());
        ros::Time time_on_device(delay, 0);
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


