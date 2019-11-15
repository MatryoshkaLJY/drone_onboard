#ifndef _CLOCKS_
#define _CLOCKS_

#include <chrono>
#include <iostream>
#include <mutex>
#include <vector>
#include <math.h>
#include "struct.hpp"

#include "const.hpp"
#include <fstream>
#include "topic.hpp"

using namespace std::chrono;
using namespace std;

extern high_resolution_clock::time_point init_timepoint;
extern Topic<string> string_topic;

float limit_values(float values, float min_values, float max_values);

float deg2rad(float deg);

float rad2deg(float rad);

int64_t intervalMs(high_resolution_clock::time_point end, high_resolution_clock::time_point start);

int64_t timestampf( void );

string getCurrentTime( void );

void remotePrint( string msg );

Vector2f ne2xy( Vector2f ne, float yaw_deg );
Vector2f xy2ne( Vector2f ne, float yaw_deg );

VelocityBody velocityNED2Body(VelocityNED velocity_ned, EulerAngle attitude);

inline float mag3f(Vector3f vec){
	return sqrtf(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
}

inline float mag2f(Vector2f vec){
	return sqrtf(vec.x * vec.x + vec.y * vec.y);
}

inline vector<float> pos_ne2xy(vector<float>pos_ne, float yaw_rad) {
	vector<float> pos_xy = { 0.0f,0.0f };
	pos_xy[0] = cos(yaw_rad) * pos_ne[0] + sin(yaw_rad) * pos_ne[1];
	pos_xy[1] = -sin(yaw_rad) * pos_ne[0] + cos(yaw_rad) * pos_ne[1];
	return pos_xy;
}

inline vector<float> pos_xy2ne(vector<float>pos_xy, float yaw_rad) {
	vector<float> pos_ne = { 0.0f,0.0f };
	pos_ne[0] = cos(yaw_rad) * pos_xy[0] - sin(yaw_rad) * pos_xy[1];
	pos_ne[1] = sin(yaw_rad) * pos_xy[0] + cos(yaw_rad) * pos_xy[1];
	return pos_ne;
}

inline vector<float> operator+(vector<float>a, vector<float>b) {
	vector<float> res;
	for (size_t i = 0; i < a.size(); i++) {
		res.push_back(a[i] + b[i]);
	}
	return res;
}

inline vector<float> operator-(vector<float>a, vector<float>b) {
	vector<float> res;
	for (size_t i = 0; i < a.size(); i++) {
		res.push_back(a[i] - b[i]);
	}
	return res;
}

inline vector<float> operator*(vector<float>a, vector<float>b) {
	vector<float> res;
	for (size_t i = 0; i < a.size(); i++) {
		res.push_back(a[i] * b[i]);
	}
	return res;
}

inline vector<float> operator*(vector<float>a, float b) {
	vector<float> res;
	for (size_t i = 0; i < a.size(); i++) {
		res.push_back(a[i] * b);
	}
	return res;
}

inline vector<float> operator*(float a, vector<float> b) {
	vector<float> res;
	for (size_t i = 0; i < b.size(); i++) {
		res.push_back(b[i] * a);
	}
	return res;
}
inline vector<float> operator/(vector<float>a, float b) {
	vector<float> res;
	for (size_t i = 0; i < a.size(); i++) {
		res.push_back(a[i] / b);
	}
	return res;
}
#endif