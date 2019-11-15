#include "send_node.hpp"

struct sockaddr_in send_to_addr;

void sendLoop( FileNode send_config )
{
    int enable;
    string host;
    int port;
    double img_msg_resize;
    int img_msg_quality;
    bool gray;
    send_config["ENABLE"] >> enable;
    send_config["HOST"] >> host;
    send_config["PORT"] >> port;
    send_config["GRAY_IMG"] >> gray;
    send_config["IMG_MSG_RESIZE"] >> img_msg_resize;
    send_config["IMG_MSG_QUALITY"] >> img_msg_quality;
    if( enable == 0 )
    {
        cout << "[WARNING]: send node disabled" << endl;
        return;
    }
    if( ! sendSocketInit() )
    {
        cout << "[WARNING]: send node shutdown" << endl;
        return;
    }

    bzero(&send_to_addr, sizeof(send_to_addr));
    if( inet_pton( AF_INET, host.c_str(), &send_to_addr.sin_addr ) <= 0 )
    {
        cout << "[WARNING]: " + string(strerror(errno)) << endl;
        return;
    }
    send_to_addr.sin_family = AF_INET; 
    send_to_addr.sin_port = htons( port );
    
    int64_t sent_err_ms = 0,sent_position_body_ms = 0, sent_velocity_body_ms = 0, sent_attitude_ms = 0, 
    sent_input_attitude_ms=0, sent_first_target_ms = 0,sent_vehicle_status_ms=0, sent_control_status_ms=0, 
    sent_down_reference_ms=0, sent_ne_reference_ms=0, sent_second_target_ms = 0, sent_third_target_ms = 0;
    high_resolution_clock::time_point t0 = high_resolution_clock::now();
    while( true )
    {
        if( intervalMs(high_resolution_clock::now(), t0) > 300 )
        {
            //cout << "Sending: " << timestampf() << endl;
            sendHeartBeat();
            sendStruct<PositionNED>(position_ned_topic, sent_position_body_ms, POSITION_NED_MSG, position_ned_mtx);
            sendStruct<VelocityBody>(velocity_body_topic, sent_velocity_body_ms, VELOCITY_BODY_MSG, velocity_body_mtx);
            sendStruct<EulerAngle>(attitude_topic, sent_attitude_ms, ATTITUDE_MSG, attitude_mtx);
            sendStruct<VehicleStatus>(vehicle_status_topic, sent_vehicle_status_ms, VEHICLE_STATUS_MSG, vehicle_status_mtx);
            sendStruct<DetectionResult>(first_target_topic, sent_first_target_ms, TARGET_MSG, first_target_mtx);
            sendStruct<DetectionResult>(second_target_topic, sent_second_target_ms, TARGET_MSG, second_target_mtx);
            sendStruct<DetectionResult>(third_target_topic, sent_third_target_ms, TARGET_MSG, third_target_mtx);
            sendStruct<int16_t>(control_status_topic, sent_control_status_ms, CONTROL_STATUS_MSG, control_status_mtx);
            sendStruct<InputAttitude>(input_attitude_topic, sent_input_attitude_ms, INPUT_ATTITUDE_MSG, input_attitude_mtx);
            sendStruct<float>(down_reference_topic, sent_down_reference_ms, REFERENCE_DOWN_MSG, down_reference_mtx);
            sendStruct<Vector2f>(ne_reference_topic, sent_ne_reference_ms, REFERENCE_NE_MSG, ne_reference_mtx);
            sendStruct<Vector2f>(pos_err_xy_topic, sent_err_ms, POS_XY_ERR_MSG, pos_err_xy_mtx);
            sendString();
            t0 = high_resolution_clock::now();
        }
        else{
            sendImg(gray, img_msg_resize, img_msg_quality);
        }
        this_thread::sleep_for( milliseconds( 100 ) );
    }
    cout << "[WARNING]: send node shutdown" << endl;
    return;
}

bool sendSocketInit()
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

void sendMsg( uint8_t msg_type, uint16_t length, void* buffer )
{
    if ( length > MAX_MSG_LENGTH + 7 ){
        cout << "[ERROR]: buffer size overflow" << endl;
        return;
    }
    uint16_t head = HEAD;
    uint16_t tail = TAIL;
    char msg[ sizeof head + sizeof msg_type + sizeof length + length + sizeof tail ];
    char* p = msg;
    memcpy(p, &head, sizeof head);
    p = p + sizeof head;
    memcpy(p, &msg_type, sizeof msg_type);
    p = p + sizeof msg_type;
    memcpy(p, &length, sizeof length);
    p = p + sizeof length;
    memcpy(p, buffer, length);
    p = p + length;
    memcpy(p, &tail, sizeof tail);
    if ( (int)sendto( fd, msg, sizeof msg, 0, (struct sockaddr *)&send_to_addr, sizeof send_to_addr ) < 0 )
    {
        cout << "[WARNING]: " + string(strerror(errno)) << endl;
    }
    return;
}

bool compress( Mat image, bool gray, double resize_k, int quality, vector<uchar>& img_buffer)
{
    vector< int > jpeg_quality{ IMWRITE_JPEG_QUALITY, quality };
    if( gray )
    {
        cvtColor( image, image, COLOR_BGR2GRAY );
    }
    resize( image, image, Size(), resize_k, resize_k );
    return imencode(".jpeg", image, img_buffer, jpeg_quality);
}

void sendHeartBeat()
{
    bool heart_beat = true;
    sendMsg( HEART_BEAT_MSG, 1, &heart_beat);
    return;
}

void sendImg( bool gray, double img_msg_resize, int img_msg_quality )
{
    int64_t timestamp;
    Mat image;
    vector< uchar > img_buffer;
    latest<Mat>(image_topic, timestamp, image, image_mtx);
    if( ! image.empty() && compress( image, gray, img_msg_resize, img_msg_quality, img_buffer ) ) {
        //cout << img_buffer.size() << endl;
        sendMsg( IMG_MSG, (uint16_t) img_buffer.size(), img_buffer.data() );
        img_buffer.clear();
    }
    return;
}

void sendString()
{
    string msg;
    vector<pair<int64_t, string>> string_vector;
    int64_t timestamp = 0;
    recent<string>(string_topic, string_vector, timestamp, string_mtx);
    clear<string>(string_topic, string_mtx);
    if( ! string_vector.empty() )
    {
        for( size_t i=0; i < string_vector.size(); i++)
        {
            msg = string_vector[i].second;
            sendMsg( LOG_MSG, (uint16_t) msg.size(), const_cast<char*>(msg.data()) );
        }
    }
    return;
}
