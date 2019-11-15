#ifndef _STRUCT_
#define _STRUCT_

#include <chrono>
#include <deque>

struct Vector2f{
    float x;
    float y;
    Vector2f operator+( const Vector2f right){
        Vector2f sum;
        sum.x = x + right.x;
        sum.y = y + right.y;
        return sum;
    }
    Vector2f operator-( const Vector2f right){
        Vector2f err;
        err.x = x - right.x;
        err.y = y - right.y;
        return err;
    }
    Vector2f operator*( const Vector2f right){
        Vector2f dot;
        dot.x = x * right.x;
        dot.y = y * right.y;
        return dot;
    }
    Vector2f operator*( float a){
        Vector2f dot;
        dot.x = x * a;
        dot.y = y * a;
        return dot;
    }
    Vector2f operator/(float a){
        Vector2f div;
        div.x = x / a;
        div.y = y / a;
        return div;
    }
};

struct Vector3f{
    float x;
    float y;
    float z;
};

struct Target{
    bool valid = false;
    bool done = false;
    float distance_m = 0;
    float x_m = 0;
    float y_m = 0;
    float z_m = 0;
    float y_deg = 0;
    float z_deg = 0;
};

//"qifffff" size: 8+4+4 + 8*2=32
struct DetectionResult{
    int32_t index = 0;
    float distance_m = 0;
    float x_m = 0;
    float y_m = 0;
    float z_m = 0;
    float confidence = -1;
};
//sizeof MissionCommand
struct MissionCommand{
    int16_t index = -3;
    uint16_t argc = 0;
    double argv[5] = {0};
};

//"qfff4x" size 8*3 = 24
struct PositionNED{
    float north_m = 0;
    float east_m = 0;
    float down_m = 0;
};

//"qfff4x" size 24
struct VelocityNED{
    float north_m_s = 0;
    float east_m_s = 0;
    float down_m_s = 0;
};

//"qfff4x" size 24
struct VelocityBody{
    float x_m_s = 0;
    float y_m_s = 0;
    float z_m_s = 0;
};

//"qfff4x" size 24
struct EulerAngle{
    float roll_deg = 0;
    float pitch_deg = 0;
    float yaw_deg = 0;
    EulerAngle operator+( const EulerAngle right){
        EulerAngle sum;
        sum.roll_deg = roll_deg + right.roll_deg;
        sum.pitch_deg = pitch_deg + right.pitch_deg;
        sum.yaw_deg = yaw_deg + right.yaw_deg;
        return sum;
    }
    EulerAngle operator/( const float right){
        EulerAngle sum;
        sum.roll_deg = roll_deg/right;
        sum.pitch_deg = pitch_deg/right;
        sum.yaw_deg = yaw_deg/right;
        return sum;
    }
};

//"qffff" size 24
struct InputVelocityBody{
    float forward_m_s = 0;
    float right_m_s = 0;
    float down_m_s = 0;
    float yawspeed_deg_s = 0;
};

//"qffff" size 24
struct InputAttitude{
    float roll_deg = 0;
    float pitch_deg = 0;
    float yaw_deg = 0;
    float thrust = 0;
};

//"q????fff50s6x" size 8 + 8 + 8 + 56 = 80
struct VehicleStatus{
    bool armed = false;
    bool in_air = false;
    bool rc_available_once = false;
    bool rc_available = false;
    float rc_signal_strength_percent = 0;
    float battery_voltage_v = 0;
    float battery_remaining_percent = 0;
    char flight_mode[50] = " ";
};

struct XYReference{
    float x_m = 0;
    float y_m = 0;
};

#endif