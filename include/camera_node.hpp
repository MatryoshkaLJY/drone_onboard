#ifndef _CAMERA_NODE_
#define _CAMERA_NODE_

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <thread>
#include <mutex>

#include "detector.hpp"
#include "utils.hpp"
#include "topic.hpp"

using namespace std;
using namespace cv;

extern mutex image_mtx;
extern Topic<Mat> image_topic;

extern mutex camera_status_mtx;
extern Topic<int> camera_status_topic;

extern mutex first_target_mtx;
extern Topic<DetectionResult> first_target_topic;

extern mutex second_target_mtx;
extern Topic<DetectionResult> second_target_topic;

extern mutex third_target_mtx;
extern Topic<DetectionResult> third_target_topic;

void cameraLoop( FileNode camera_config );

void cameraLoopTest();

class Camera{
    public:
        Camera(mutex& mtx);
        bool init( int id, int _width, int _height );
        bool isOpened( void );
        bool read( Mat& frame );
        void release( void );
    private:
        int width, height;
        double fps, interval;
        bool new_frame;
        VideoCapture cap;
        mutex& cap_mutex;
        high_resolution_clock::time_point last_grab;
        void updating( void );
};

class Video{
    public:
        Video( string path, int width, int height );
        void open();
        void close();
        void writeImage(Mat image);
    private:
        string video_path;
        int width, height;
        VideoWriter writer;
};

#endif