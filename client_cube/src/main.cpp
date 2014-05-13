#include "Client.h"

int main(int argc, char* argv[]){

    
	printf("Main start...\n");
	if(argc >= 1){
		//std::string idCube(std::string(argv[1])); 
		//if (argv[1][0] == "A" or idCube == "B"){
			if(argc == 2){
				printf("Cube id: %s\n",  argv[1]);
				ros::init(argc, argv, std::string("image_publisher")+std::string(argv[1]));
				Client client(argv[1], "127.0.0.1", 5005);//192.168.10.245
				client.startListeningLoop();
			}
			else{
				printf("Cube id: %s\n", argv[1]);
				//ros::init(argc, argv,  std::string("image_publisher")+std::string(argv[1]));
				ros::init(argc, argv,  std::string("image_publisher"));
				printf("Host: %s\n", argv[2]);
				Client client(argv[1], std::string(argv[2]), 5005);
				client.startListeningLoop();
			}
		//}
		//else
			//printf("You need to specify what cube...\n");

		
	}
	else{

		printf("You need to specify what cube...\n");
	}
	return 0;
}
