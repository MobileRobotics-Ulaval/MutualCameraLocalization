#ifndef SOCKET_H
#define SOCKET_H

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <netdb.h>
#include <stdlib.h> //calloc
#include <stdio.h>
#include <cstdlib> // exit()
#include <string.h>
#include <string>

#include <ros/ros.h>

class Socket
{
protected:
    struct sockaddr_in sockClient;
    int comSocket;
    int socketFileDescriptor;
public:
	void init(std::string address, int port);
	ssize_t toSend(const char * byte, size_t size, int flag);
	ssize_t toReceive(char * byte, size_t size, int flag);
	void disconnect();
	void checkForError(int errorCode, const char* errorMessage);
};


class SocketTCP: public Socket
{
public:
	void init(std::string address,int port);
	ssize_t toSend(const char * byte, size_t size, int flag);
	ssize_t toReceive(char * byte, size_t size, int flag);
};

class SocketUDP: public Socket
{
public:
	void init(std::string address,int port);
	ssize_t toSend(const char * byte, size_t size, int flag);
	ssize_t toReceive(char * byte, size_t size, int flag);
private:
	struct sockaddr_in remoteSoc;
};


#endif // SOCKET_H
