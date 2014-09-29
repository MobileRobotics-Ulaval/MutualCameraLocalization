
#ifndef _SOCKETS_H_
#define _SOCKETS_H_

#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <netdb.h> 
#include <stdlib.h> //calloc
#include <stdio.h>
#include <cstdlib> // exit()
#include <string.h> 

namespace img_server
{
class Socket
{
public:
	void init(int port);
	void connect();
	ssize_t toSend(const char * byte, size_t size, int flag);
	ssize_t toReceive(char * byte, size_t size, int flag);
	void disconnect();
	void checkForError(int errorCode, const char* errorMessage);
protected:
    struct sockaddr_in serverSock;
    int comSocket;  
    int socketFileDescriptor; 
};


class SocketTCP: public Socket
{
public:
	void connect();
	bool bindAndAccept();
	ssize_t toSend(const char * byte, size_t size, int flag);
	ssize_t toReceive(char * byte, size_t size, int flag);
};

class SocketUDP: public Socket
{
public:
	void init(int port);
	void connect();
	bool bindAndAccept();
	ssize_t toSend(const char * byte, size_t size, int flag);
	ssize_t toReceive(char * byte, size_t size, int flag);
private:
	struct sockaddr_in clientSock;
	socklen_t clientSockLen;
};

}

#endif