#ifndef _RECV_NODE_
#define _RECV_NODE_

#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <arpa/inet.h> 
#include <string.h>
#include <errno.h>
#include <opencv2/core/core.hpp>
#include <thread>
#include <mutex>

#include "utils.hpp"
#include "struct.hpp"

#include "const.hpp"

using namespace std;
using namespace chrono;
using namespace cv;

extern int fd;
extern mutex fd_mutex;

extern mutex mission_command_mtx;
extern Topic<MissionCommand> mission_command_topic;

void recvLoop( FileNode recv_config );

bool recvSocketInit();

void recvMsg( char* msg, int msg_length, sockaddr_in address );

#endif