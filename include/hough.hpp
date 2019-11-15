#include <opencv2/core/hal/intrin.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <iterator>

using namespace cv;

void HoughCirclesAlt( InputArray _edges, InputArray _dx, InputArray _dy, OutputArray _circles,
                          int method, double dp, double minDist,
                          double param2,
                          int minRadius, int maxRadius,
                          int maxCircles );