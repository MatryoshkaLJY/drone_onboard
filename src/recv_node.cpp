#include "recv_node.hpp"

struct sockaddr_in recv_from_addr;

void recvLoop( FileNode recv_config )
{
    int enable;
    string host;
    int port;
    recv_config["ENABLE"] >> enable;
    recv_config["HOST"] >> host;
    recv_config["PORT"] >> port;
    if( enable == 0 )
    {
        cout << "[WARNING]: recv node disabled" << endl;
        return;
    }
    if( ! recvSocketInit() )
    {
        cout << "[WARNING]: recv node shutdown" << endl;
        return;
    }

    bzero(&recv_from_addr, sizeof(recv_from_addr));
    if( inet_pton( AF_INET, host.c_str(), &recv_from_addr.sin_addr ) <= 0 )
    {
        cout << "[WARNING]: " + string(strerror(errno)) << endl;
        return;
    }
    recv_from_addr.sin_family = AF_INET; 
    recv_from_addr.sin_port = htons( port );

    char msg[MAX_MSG_LENGTH];
    int msg_length;
    struct sockaddr_in addr;
    socklen_t addr_len  = sizeof recv_from_addr;
    while (true)
    {
        msg_length = (int)recvfrom( fd, msg, sizeof msg, 0, (struct sockaddr *)&addr, &addr_len );
        if( msg_length < 0 && errno == EAGAIN)
        {
            this_thread::sleep_for( milliseconds( 30 ) );
            continue;
        }
        else if( msg_length < 0 )
        {
            cout << "[WARNING]: " + string(strerror(errno)) << endl;
            continue;
        }
        recvMsg( msg, msg_length, addr );
    }
    cout << "[WARNING]: recv node shutdown" << endl;
    return;
}

bool recvSocketInit()
{
    fd_mutex.lock();
    if( fd < 0 )
    {
        if ( ( fd = socket(AF_INET, SOCK_DGRAM, 0 ) ) < 0 )
        {
            cout << "[WARNING]: " + string(strerror(errno)) << endl;
            fd_mutex.unlock();
            return false;
        }
        if ( fcntl( fd, F_SETFL, O_NONBLOCK ) < 0 )
        {
            cout << "[WARNING]: " + string(strerror(errno)) << endl;
            fd_mutex.unlock();
            return false;
        }
    }
    fd_mutex.unlock();
    return true;
}

void recvMsg( char* msg, int msg_length, sockaddr_in addr )
{
    uint8_t msg_type;
    uint16_t head, tail, len;
    if ( addr.sin_addr.s_addr != recv_from_addr.sin_addr.s_addr || addr.sin_port != recv_from_addr.sin_port )
        return;
    if( msg_length < 8 || msg_length > MAX_MSG_LENGTH )
        return;
    char *p = msg;
    memcpy( &head, p, sizeof head );
    p = p + sizeof head;
    if( head != HEAD )
        return;
    memcpy( &msg_type, p, sizeof msg_type );
    p = p + sizeof msg_type;
    memcpy( &len, p, sizeof len );
    p = p + sizeof len;
    if( msg_length != 7 + (int)len )
        return;
    char buffer[len];
    memcpy( buffer, p, len );
    p = p + len;
    memcpy( &tail, p, sizeof tail );
    if( tail != TAIL )
        return;
    MissionCommand mission_command;
    //int index;
    //double param_1;
    switch(msg_type)
    {
        case MISSION_COMMAND_MSG:
            memcpy( &mission_command, buffer, sizeof mission_command );
            update<MissionCommand>(mission_command_topic, mission_command, mission_command_mtx);
            break;
        default:
            break;
    }
}