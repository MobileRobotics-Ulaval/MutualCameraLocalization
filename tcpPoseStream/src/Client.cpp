// Les threads utilisent std:thread et donc il est nécessaire d'utlisé c++11 rajouter le flags  "-std=c++11" à la compilation

#include "Client.h"

using namespace std;

/**
    Initiation of the client socket
*/
Client::Client(){

    string address, topic;
    int port;
    ros::param::get("~address", address);
    ros::param::get("~topic", topic);
    ros::param::get("~port", port);

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
    if (connect(comSocket,(struct sockaddr *) &server,sizeof(server)) < 0){
        //TODO add try again
        ROS_ERROR("ERROR connecting to socket");
        exit(0);
    }
    ROS_INFO("A connect");
    ROS_INFO("Listening to topic%s", topic.c_str());


    pose_ = nh.subscribe(topic.c_str(), 1, &Client::callback, this);
//    pose_B = nh.subscribe("/cubeB_pose", 1, &Client::callbackB, this);
}
//sscanf(p->buffer_in, "l-x:%ld/y:%ld/z:%ld/yaw:%ld/pitch:%ld/roll:%ld\n", &lposx, &lposy, &lposz, &lthetax, &lthetay, &lthetaz);
void Client::callback(const geometry_msgs::PoseStamped::ConstPtr& pos){
    std::stringstream request;
    //Eigen::Quaterniond orientation = Eigen::Quaterniond(Eigen::AngleAxisd(cubesAngles[0], Eigen::Vector3d::UnitZ()));

    Eigen::Quaterniond quad(pos->pose.orientation.x,pos->pose.orientation.y,pos->pose.orientation.z,pos->pose.orientation.w);
    double yaw = atan2(quad.toRotationMatrix()(1, 2), quad.toRotationMatrix()(2, 2));
    //cout<<"M:"<<quad.toRotationMatrix() <<endl;
    //cout<<"P:"<<quad.toRotationMatrix()(2, 2) <<endl;
    request<<"l-x:"<<(long int )(pos->pose.position.x*100.0)<<"y:"<<(long int )(pos->pose.position.y*100.0)<<"z:0yaw:"<<(long int)(yaw *180.0 / M_PI) <<"pitch:0roll:0\n";
    string r = request.str();
    ROS_INFO(r.c_str());
     if(send(comSocket, r.data(), r.size(), 0) < 0){
        ROS_ERROR("Error: Impossible d'envoyer");
        //close(comSocket);
        //exit(0);
    }
}


Client::~Client(){

    close(comSocket);
}


