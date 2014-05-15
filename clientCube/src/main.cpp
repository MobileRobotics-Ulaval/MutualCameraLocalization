#include "Client.h"

int main(int argc, char* argv[]){

    
	ROS_INFO("Main start...\n");
	//ros::init(argc, argv,  std::string("image_publisher")+std::string(argv[1]));
	ros::init(argc, argv,  "image_publisher");
	Client client;
	client.startListeningLoop();
	return 0;
}
