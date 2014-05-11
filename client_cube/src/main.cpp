#include "Client.h"

int main(int argc, char* argv[]){

	ros::init(argc, argv, "image_publisher");
    
	printf("Main commence...\n");
	if(argc == 1){
		Client client("127.0.0.1", 5005);//192.168.10.245
		client.startListeningLoop();
	}
	else{
		printf("Host: %s\n", argv[1]);
		Client client(std::string(argv[1]), 5005);
		client.startListeningLoop();
	}
	return 0;
}
