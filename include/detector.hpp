#ifndef _DETECTOR_
#define _DETECTOR_

#include <iostream>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <chrono>

#include <opencv2/core/utility.hpp>
#include <opencv2/videoio.hpp>
#include <algorithm>

#include "hough.hpp"
#include "kcftracker.hpp"
#include "utils.hpp"

extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
#include "common/getopt.h"
}

using namespace cv;
using namespace std;
using namespace std::chrono;

enum DETECTOR_TYPE{
    PURE_DETECT = 0,
    PURE_TRACK = 1,
    DETECT_AND_TRACK = 2
};


enum COLOR{
    NONE = -1,
    RED = 0,
    BLUE = 1,
    GREEN = 2,
};

enum DETECTOR_STATUS{
    GLOBAL_DETECT = 0,
    LOCAL_ESTIMATE = 1,
    GLOBAL_ESTIMATE = 2,
    TRACKING = 3,
    DETECT_TEST = 4,
};

void drawSquare(Mat& image, vector<Vec2f> square);

void solveSquare(vector<Vec2f> square, float& distance, Vec3f& orientation, Mat camera_matrix, Mat distort_coeff, float real_scale);

void solvePosition(Rect2f rect, float& distance, Vec3f& orientation, Mat camera_matrix, Mat distort_coeff, float real_scale);

float theta(Vec2f v1, Vec2f v2);

class LowPassFilter{
    public:
    void init(float scale_cut_freq, float center_cut_freq);
    void run(Rect2f rect, Rect2f& rect_filtered, high_resolution_clock::time_point now);
    private:
    float scale_cut_freq, center_cut_freq;
    bool initialized = false;
    high_resolution_clock::time_point prev_time;
    Vec2f prev_center; //x, y, scale in pixel
    float prev_scale;
};

class CircleDetector{
    public:
    CircleDetector(DETECTOR_TYPE type, COLOR _color=NONE, float radius=0, float scale_cut_freq=5, float center_cut_freq=5);
    bool run(Mat image, Rect2f &rect, float& confidence);

    private:
    DETECTOR_STATUS status;
    DETECTOR_TYPE type;
    COLOR color;
    Rect2f prev_rec;
    uint8_t fail_cnt;
    uint8_t fail_tolerent = 2;
    KCFTracker tracker;
    LowPassFilter filter;
    Rect2f resizeRect( Rect2f rect, float ratio);
    void colorMask(Mat image, Mat& mask);
    void edgeDetection(Mat image, Mat& edges, Mat& dx, Mat& dy, float canny_thresh);
    void param_adapt( float r, float &canny_thresh, float &dp, float &delta_r, float &reject_thresh );
    float circleDetect(Mat edge, Mat dx, Mat dy, Rect2f& roi, float dp, float r_min, float r_max, float accum_thresh=15.0, float dist_min=5.0);
};

// class SquareDetector{
//     public:
//     SquareDetector(COLOR color);
//     //bool run(Mat image, vector<Vec2f> &square, float& confidence);
//     bool run(Mat image, Rect2f &rect, float& confidence);
//     //Rect2f square2Rect( vector<Vec2f> square );

//     private:
//     apriltag_family_t *tf = NULL;
//     apriltag_detector_t *td = NULL;
//     DETECTOR_STATUS status;
//     COLOR color;
//     KCFTracker tracker;
//     double res_k = 1.0;
//     Rect2f resizeRect( Rect2f rect, float ratio);
//     void colorMask(Mat image, Mat& mask);
//     void edgeDetection(Mat image, Mat& edges, Mat& dx, Mat& dy, float canny_thresh);
//     void zarray2vector(zarray_t *quads, vector<vector<Vec2f>>& result);
//     float squareRatio(vector<Vec2f> quad);
//     bool quadFilter( vector<vector<Vec2f>> quads, int& index, float& confidence);
//     bool squareDetect(Mat image, vector<Vec2f>& square, float& confidence);
// };

class SquareDetector{
    public:
    SquareDetector(COLOR color);
    //bool run(Mat image, vector<Vec2f> &square, float& confidence);
    bool run(Mat image, Rect2f &rect, float& confidence);
    //Rect2f square2Rect( vector<Vec2f> square );

    private:
    DETECTOR_STATUS status;
    COLOR color;
    KCFTracker tracker;
    LowPassFilter filter;
    Rect2f resizeRect( Rect2f rect, float ratio);
    void colorMask(Mat image, Mat& mask);
    void edgeDetection(Mat image, Mat& edges, Mat& dx, Mat& dy, float canny_thresh);
    bool squareDetect(Mat image, Rect2f &rect, float& confidence);
};
#endif
