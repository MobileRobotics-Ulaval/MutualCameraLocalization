#include "Socket.h"

namespace img_server
{
void Socket::init(int port){
    bzero((char *) &clientAddress, sizeof(clientAddress));
    clientAddress.sin_family = AF_INET;
    clientAddress.sin_addr.s_addr = INADDR_ANY;
    clientAddress.sin_port = htons(port);
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

void SocketTCP::connect(){
	this->socketFileDescriptor = socket(AF_INET, SOCK_STREAM, 0);

    this->checkForError(this->socketFileDescriptor, "ERROR opening socket");
 }

bool SocketTCP::bindAndAccept(){
    int comErrorCode = bind(this->socketFileDescriptor, (struct sockaddr *) &clientAddress, sizeof(clientAddress));
    if(comErrorCode != 0){
        this->checkForError(comErrorCode, "ERROR on binding");
        this->disconnect();
        return false;
    }
    
    listen(socketFileDescriptor, 5);
    
    socklen_t clientAdressLength = (socklen_t) sizeof(clientAddress);
    this->comSocket = accept(this->socketFileDescriptor, (struct sockaddr *) &clientAddress, &clientAdressLength);
    this->checkForError(this->comSocket, "ERROR on accept");
    return true;
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

void SocketUDP::connect(){
	this->socketFileDescriptor = socket(AF_INET, SOCK_DGRAM, 0);

    this->checkForError(this->socketFileDescriptor, "ERROR opening socket");

    int comErrorCode = bind(this->socketFileDescriptor, (struct sockaddr *) &clientAddress, sizeof(clientAddress));
    if(comErrorCode != 0){
        this->checkForError(comErrorCode, "ERROR on binding");
        this->disconnect();
        return false;
    }
}


ssize_t SocketUDP::toSend(const char * byte, size_t size, int flag){
	int comErrorCode = sendto(comSocket, byte, size, flag,
							  (struct sockaddr *)&clientAddress, sizeof(clientAddress));
    this->checkForError(comErrorCode, "ERROR on send");
    return comErrorCode;
}	

ssize_t SocketUDP::toReceive(char * byte, size_t size, int flag){
	struct sockaddr_in remaddr;
	socklen_t addrlen = sizeof(remaddr);
	int comErrorCode = recvfrom(comSocket, byte, size, flag,
							   (struct sockaddr *)&remaddr, &addrlen);
    this->checkForError(comErrorCode, "ERROR on receive");
    return comErrorCode;
}

}

