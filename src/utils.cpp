#include "utils.hpp"

int64_t intervalMs(high_resolution_clock::time_point end, high_resolution_clock::time_point start)
{
    duration<double> time_span = end - start;
    milliseconds d = duration_cast< milliseconds >( time_span );
    return d.count();
}

int64_t timestampf( void )
{
    return intervalMs(high_resolution_clock::now(), init_timepoint);
}

string getCurrentTime( void )
{
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer,sizeof(buffer),"%m-%d_%H-%M-%S",timeinfo);
    string current_time(buffer);
    return current_time;
}

void remotePrint( string msg )
{
    if( msg.size() > 500 )
    {
        cout << "[ERROR]: remote print overflow" << endl;
        return;
    }
    string_topic.update(msg);
    return;
}

float limit_values(float values, float min_values, float max_values) {
	if( values > max_values )
    {
        values = max_values;
    }
    else if( values < min_values )
    {
        values = min_values;
    }
    return values;
}

float deg2rad(float deg) {
	return deg * P_I / 180.0f;
}

float rad2deg(float rad)
{
	return rad * 180.0f / P_I;
}

Vector2f ne2xy( Vector2f ne, float yaw_deg )
{
    float yaw_rad = deg2rad(yaw_deg);
    float cos_yaw = cos(yaw_rad);
    float sin_yaw = sin(yaw_rad);
    Vector2f xy;
    xy.x = ne.x * cos_yaw + ne.y * sin_yaw;
    xy.y = -1 * ne.x * sin_yaw + ne.y * cos_yaw;
    return xy; 
}

Vector2f xy2ne( Vector2f xy, float yaw_deg )
{
    float yaw_rad = deg2rad(yaw_deg);
    float cos_yaw = cos(yaw_rad);
    float sin_yaw = sin(yaw_rad);
    Vector2f ne;
    ne.x = xy.x * cos_yaw - xy.y * sin_yaw;
    ne.y = xy.x * sin_yaw + xy.y * cos_yaw;
    return ne;
}

VelocityBody velocityNED2Body(VelocityNED velocity_ned, EulerAngle attitude)
{
    VelocityBody velocity_body;
    float yaw_rad = attitude.yaw_deg * P_I / 180;
    velocity_body.x_m_s = velocity_ned.north_m_s * cos(yaw_rad) + velocity_ned.east_m_s * sin(yaw_rad);
    velocity_body.y_m_s = velocity_ned.north_m_s * sin(-yaw_rad) + velocity_ned.east_m_s * cos(yaw_rad);
    velocity_body.z_m_s = velocity_ned.down_m_s;
    return velocity_body;
}