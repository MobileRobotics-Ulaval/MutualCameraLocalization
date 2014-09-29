#include "Socket.h"

namespace img_server
{
void Socket::init(int port){
    bzero((char *) &serverSock, sizeof(serverSock));
    serverSock.sin_family = AF_INET;
    serverSock.sin_addr.s_addr = INADDR_ANY;
    serverSock.sin_port = htons(port);
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
    int comErrorCode = bind(this->socketFileDescriptor, (struct sockaddr *) &serverSock, sizeof(serverSock));
    if(comErrorCode != 0){
        this->checkForError(comErrorCode, "ERROR on binding");
        this->disconnect();
        return false;
    }
    
    listen(socketFileDescriptor, 5);
    
    socklen_t clientAdressLength = (socklen_t) sizeof(serverSock);
    this->comSocket = accept(this->socketFileDescriptor, (struct sockaddr *) &serverSock, &clientAdressLength);
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
void SocketUDP::init(int port){
    this->socketFileDescriptor = socket(AF_INET, SOCK_DGRAM, 0);
    this->checkForError(this->socketFileDescriptor, "ERROR opening socket");

    bzero((char *) &serverSock, sizeof(serverSock));
    serverSock.sin_family = AF_INET;
    serverSock.sin_addr.s_addr = htonl(INADDR_ANY);
    serverSock.sin_port = htons(port);
}

/*
void SocketUDP::connect(){
    this->socketFileDescriptor = socket(AF_INET, SOCK_DGRAM, 0);
    this->checkForError(this->socketFileDescriptor, "ERROR opening socket");
}*/

bool SocketUDP::bindAndAccept(){
    int comErrorCode = bind(this->socketFileDescriptor, (struct sockaddr *) &serverSock, sizeof(serverSock));
    if(comErrorCode != 0){
        this->checkForError(comErrorCode, "ERROR on binding");
        this->disconnect();
        //return false;
        exit(-1);
    }
}


ssize_t SocketUDP::toSend(const char * byte, size_t size, int flag){
    //socklen_t addrlen = sizeof(clientSock);
	int comErrorCode = sendto(socketFileDescriptor, byte, size, flag,
							 (struct sockaddr *)&clientSock, sizeof(clientSock));
    this->checkForError(comErrorCode, "ERROR on send");
    return comErrorCode;
}	

ssize_t SocketUDP::toReceive(char * byte, size_t size, int flag){
    clientSockLen = sizeof(clientSock);
	int comErrorCode = recvfrom(socketFileDescriptor, byte, size, flag,
							   (struct sockaddr *)&clientSock, &clientSockLen);

    printf("from port %d and address %s.\n", ntohs(clientSock.sin_port),
        inet_ntoa (clientSock.sin_addr));
    this->checkForError(comErrorCode, "ERROR on receive");
    return comErrorCode;
}

}
