#include "Socket.h"

using namespace std;

void Socket::init(string address,int port){
}

void Socket::checkForError(int errorCode, const char* errorMessage){
    if (errorCode < 0)
        perror(errorMessage);
}

void Socket::disconnect(){
    close(this->comSocket);
    close(this->socketFileDescriptor);
}

/**
    TCP SOCKET
*/

void SocketTCP::init(string address,int port){
    comSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (comSocket < 0)
        ROS_ERROR("ERROR opening socket");

   struct hostent *host = gethostbyname(address.c_str());
    if (host == NULL)
        ROS_ERROR("ERROR, no such host");

    bzero((char *) &sockClient, sizeof(sockClient));
    sockClient.sin_family = AF_INET;
    bcopy((char *)host->h_addr, (char *)&sockClient.sin_addr.s_addr, host->h_length);
    sockClient.sin_port = htons(port);
    int errorCode = connect(comSocket,(struct sockaddr *) &sockClient, sizeof(sockClient));
    if(errorCode < 0){
        //TODO add try again
        ROS_ERROR("ERROR connecting to socket, %i", errorCode);
        exit(0);
    }
}


ssize_t SocketTCP::toSend(const char * byte, size_t size, int flag){
    int comErrorCode = send(comSocket, byte, size, flag);
    this->checkForError(comErrorCode, "ERROR on send");
    return comErrorCode;
}

ssize_t SocketTCP::toReceive(char * byte, size_t size, int flag){
    int comErrorCode = recv(comSocket, byte, size, flag);
    this->checkForError(comErrorCode, "ERROR on receive");
    return comErrorCode;
}

/**
    UDP SOCKET
*/
void SocketUDP::init(string address,int port){
    comSocket = socket(AF_INET,SOCK_DGRAM,0);

    bzero(&sockClient,sizeof(sockClient));
    sockClient.sin_family = AF_INET;
    sockClient.sin_addr.s_addr = inet_addr(address.c_str());
    sockClient.sin_port = htons(port);

    /*comSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (comSocket < 0)
        ROS_ERROR("ERROR opening socket");

   struct hostent *host = gethostbyname(address.c_str());
    if (host == NULL)
        ROS_ERROR("ERROR, no such host");

    bzero((char *) &server, sizeof(server));
    server.sin_family = AF_INET;
    bcopy((char *)host->h_addr, (char *)&server.sin_addr.s_addr, host->h_length);
    server.sin_port = htons(port);
    /*int errorCode = connect(comSocket,(struct sockaddr *) &server, sizeof(server));
    if(errorCode < 0){
        //TODO add try again
        ROS_ERROR("ERROR connecting to socket, %i", errorCode);
        exit(0);
    }*/
}

ssize_t SocketUDP::toSend(const char * byte, size_t size, int flag){
	int comErrorCode = sendto(comSocket, byte, size, flag,
							   (struct sockaddr *)&sockClient, sizeof(sockClient));
	this->checkForError(comErrorCode, "ERROR on send");
	return comErrorCode;
}

ssize_t SocketUDP::toReceive(char * byte, size_t size, int flag){
	//socklen_t addrlen = sizeof(sockServer);
	int comErrorCode = recvfrom(comSocket, byte, size, flag,
							   NULL, NULL);
	this->checkForError(comErrorCode, "ERROR on receive");
	return comErrorCode;
}
