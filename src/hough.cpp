/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Copyright (C) 2014, Itseez, Inc, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

//#include "precomp.hpp"
//#include "opencl_kernels_imgproc.hpp"
#include "hough.hpp"

// Classical Hough Transform
struct LinePolar
{
    float rho;
    float angle;
};


struct hough_cmp_gt
{
    hough_cmp_gt(const int* _aux) : aux(_aux) {}
    inline bool operator()(int l1, int l2) const
    {
        return aux[l1] > aux[l2] || (aux[l1] == aux[l2] && l1 < l2);
    }
    const int* aux;
};

/****************************************************************************************\
*                                     Circle Detection                                   *
\****************************************************************************************/

struct EstimatedCircle
{
    EstimatedCircle(Vec3f _c, int _accum) :
        c(_c), accum(_accum) {}
    Vec3f c;
    int accum;
};

static bool cmpAccum(const EstimatedCircle& left, const EstimatedCircle& right)
{
    // Compare everything so the order is completely deterministic
    // Larger accum first
    if (left.accum > right.accum)
        return true;
    else if (left.accum < right.accum)
        return false;
    // Larger radius first
    else if (left.c[2] > right.c[2])
        return true;
    else if (left.c[2] < right.c[2])
        return false;
    // Smaller X
    else if (left.c[0] < right.c[0])
        return true;
    else if (left.c[0] > right.c[0])
        return false;
    // Smaller Y
    else if (left.c[1] < right.c[1])
        return true;
    else if (left.c[1] > right.c[1])
        return false;
    // Identical - neither object is less than the other
    else
        return false;
}

static inline Vec3f GetCircle(const EstimatedCircle& est)
{
    return est.c;
}

static inline Vec4f GetCircle4f(const EstimatedCircle& est)
{
    return Vec4f(est.c[0], est.c[1], est.c[2], (float)est.accum);
}

class NZPointList : public std::vector<Point>
{
private:
    NZPointList(const NZPointList& other); // non-copyable

public:
    NZPointList(int reserveSize = 256)
    {
        reserve(reserveSize);
    }
};

class NZPointSet
{
private:
    NZPointSet(const NZPointSet& other); // non-copyable

public:
    Mat_<uchar> positions;

    NZPointSet(int rows, int cols) :
        positions(rows, cols, (uchar)0)
    {
    }

    void insert(const Point& pt)
    {
        positions(pt) = 1;
    }

    void insert(const NZPointSet& from)
    {
        cv::bitwise_or(from.positions, positions, positions);
    }

    void toList(NZPointList& list) const
    {
        for (int y = 0; y < positions.rows; y++)
        {
            const uchar *ptr = positions.ptr<uchar>(y, 0);
            for (int x = 0; x < positions.cols; x++)
            {
                if (ptr[x])
                {
                    list.push_back(Point(x, y));
                }
            }
        }
    }
};

class HoughCirclesAccumInvoker : public ParallelLoopBody
{
public:
    HoughCirclesAccumInvoker(const Mat &_edges, const Mat &_dx, const Mat &_dy, int _minRadius, int _maxRadius, float _idp,
                             std::vector<Mat>& _accumVec, NZPointSet& _nz, Mutex& _mtx) :
        edges(_edges), dx(_dx), dy(_dy), minRadius(_minRadius), maxRadius(_maxRadius), idp(_idp),
        accumVec(_accumVec), nz(_nz), mutex(_mtx)
    {
        acols = cvCeil(edges.cols * idp), arows = cvCeil(edges.rows * idp);
        astep = acols + 2;
    }

    ~HoughCirclesAccumInvoker() { }

    void operator()(const Range &boundaries) const CV_OVERRIDE
    {
        Mat accumLocal = Mat(arows + 2, acols + 2, CV_32SC1, Scalar::all(0));
        int *adataLocal = accumLocal.ptr<int>();
        NZPointSet nzLocal(nz.positions.rows, nz.positions.cols);
        int startRow = boundaries.start;
        int endRow = boundaries.end;
        int numCols = edges.cols;

        if(edges.isContinuous() && dx.isContinuous() && dy.isContinuous())
        {
            numCols *= (boundaries.end - boundaries.start);
            endRow = boundaries.start + 1;
        }

        // Accumulate circle evidence for each edge pixel
        for(int y = startRow; y < endRow; ++y )
        {
            const uchar* edgeData = edges.ptr<const uchar>(y);
            const short* dxData = dx.ptr<const short>(y);
            const short* dyData = dy.ptr<const short>(y);
            int x = 0;

            for(; x < numCols; ++x )
            {
#if CV_SIMD
                {
                    v_uint8 v_zero = vx_setzero_u8();

                    for(; x <= numCols - 2*v_uint8::nlanes; x += 2*v_uint8::nlanes) {
                        v_uint8 v_edge1 = (vx_load(edgeData + x                  ) != v_zero);
                        v_uint8 v_edge2 = (vx_load(edgeData + x + v_uint8::nlanes) != v_zero);

                        if(v_check_any(v_edge1))
                        {
                            x += v_scan_forward(v_edge1);
                            goto _next_step;
                        }

                        if(v_check_any(v_edge2))
                        {
                            x += v_uint8::nlanes + v_scan_forward(v_edge2);
                            goto _next_step;
                        }
                    }
                }
#endif
                for(; x < numCols && !edgeData[x]; ++x)
                    ;

                if(x == numCols)
                    continue;
#if CV_SIMD
_next_step:
#endif
                float vx, vy;
                int sx, sy, x0, y0, x1, y1;

                vx = dxData[x];
                vy = dyData[x];

                if(vx == 0 && vy == 0)
                    continue;

                float mag = std::sqrt(vx*vx+vy*vy);

                if(mag < 1.0f)
                    continue;

                Point pt = Point(x % edges.cols, y + x / edges.cols);
                nzLocal.insert(pt);

                sx = cvRound((vx * idp) * 1024 / mag);
                sy = cvRound((vy * idp) * 1024 / mag);

                x0 = cvRound((pt.x * idp) * 1024);
                y0 = cvRound((pt.y * idp) * 1024);

                // Step from min_radius to max_radius in both directions of the gradient
                for(int k1 = 0; k1 < 2; k1++ )
                {
                    x1 = x0 + minRadius * sx;
                    y1 = y0 + minRadius * sy;

                    for(int r = minRadius; r <= maxRadius; x1 += sx, y1 += sy, r++ )
                    {
                        int x2 = x1 >> 10, y2 = y1 >> 10;
                        if( (unsigned)x2 >= (unsigned)acols ||
                            (unsigned)y2 >= (unsigned)arows )
                            break;

                        adataLocal[y2*astep + x2]++;
                    }

                    sx = -sx; sy = -sy;
                }
            }
        }

        { // TODO Try using TLSContainers
            AutoLock lock(mutex);
            accumVec.push_back(accumLocal);
            nz.insert(nzLocal);
        }
    }

private:
    const Mat &edges, &dx, &dy;
    int minRadius, maxRadius;
    float idp;
    std::vector<Mat>& accumVec;
    NZPointSet& nz;

    int acols, arows, astep;

    Mutex& mutex;
};

class HoughCirclesFindCentersInvoker : public ParallelLoopBody
{
public:
    HoughCirclesFindCentersInvoker(const Mat &_accum, std::vector<int> &_centers, int _accThreshold, Mutex& _mutex) :
        accum(_accum), centers(_centers), accThreshold(_accThreshold), _lock(_mutex)
    {
        acols = accum.cols;
        arows = accum.rows;
        adata = accum.ptr<int>();
    }

    ~HoughCirclesFindCentersInvoker() {}

    void operator()(const Range &boundaries) const CV_OVERRIDE
    {
        int startRow = boundaries.start;
        int endRow = boundaries.end;
        std::vector<int> centersLocal;
        bool singleThread = (boundaries == Range(1, accum.rows - 1));

        startRow = max(1, startRow);
        endRow = min(arows - 1, endRow);

        //Find possible circle centers
        for(int y = startRow; y < endRow; ++y )
        {
            int x = 1;
            int base = y * acols + x;

            for(; x < acols - 1; ++x, ++base )
            {
                if( adata[base] > accThreshold &&
                    adata[base] > adata[base-1] && adata[base] >= adata[base+1] &&
                    adata[base] > adata[base-acols] && adata[base] >= adata[base+acols] )
                    centersLocal.push_back(base);
            }
        }

        if(!centersLocal.empty())
        {
            if(singleThread)
                centers = centersLocal;
            else
            {
                AutoLock alock(_lock);
                centers.insert(centers.end(), centersLocal.begin(), centersLocal.end());
            }
        }
    }

private:
    const Mat &accum;
    std::vector<int> &centers;
    int accThreshold;

    int acols, arows;
    const int *adata;
    Mutex& _lock;
};

template<typename T>
static bool CheckDistance(const std::vector<T> &circles, size_t endIdx, const T& circle, float minDist2)
{
    bool goodPoint = true;
    for (uint j = 0; j < endIdx; ++j)
    {
        T pt = circles[j];
        float distX = circle[0] - pt[0], distY = circle[1] - pt[1];
        if (distX * distX + distY * distY < minDist2)
        {
            goodPoint = false;
            break;
        }
    }
    return goodPoint;
}

static void GetCircleCenters(const std::vector<int> &centers, std::vector<Vec3f> &circles, int acols, float minDist, float dr)
{
    size_t centerCnt = centers.size();
    float minDist2 = minDist * minDist;
    for (size_t i = 0; i < centerCnt; ++i)
    {
        int center = centers[i];
        int y = center / acols;
        int x = center - y * acols;
        Vec3f circle = Vec3f((x + 0.5f) * dr, (y + 0.5f) * dr, 0);

        bool goodPoint = CheckDistance(circles, circles.size(), circle, minDist2);
        if (goodPoint)
            circles.push_back(circle);
    }
}

static void GetCircleCenters(const std::vector<int> &centers, std::vector<Vec4f> &circles, int acols, float minDist, float dr)
{
    size_t centerCnt = centers.size();
    float minDist2 = minDist * minDist;
    for (size_t i = 0; i < centerCnt; ++i)
    {
        int center = centers[i];
        int y = center / acols;
        int x = center - y * acols;
        Vec4f circle = Vec4f((x + 0.5f) * dr, (y + 0.5f) * dr, 0, (float)center);

        bool goodPoint = CheckDistance(circles, circles.size(), circle, minDist2);
        if (goodPoint)
            circles.push_back(circle);
    }
}

template<typename T>
static void RemoveOverlaps(std::vector<T>& circles, float minDist)
{
    if (circles.size() <= 1u)
        return;
    float minDist2 = minDist * minDist;
    size_t endIdx = 1;
    for (size_t i = 1; i < circles.size(); ++i)
    {
        T circle = circles[i];
        if (CheckDistance(circles, endIdx, circle, minDist2))
        {
            circles[endIdx] = circle;
            ++endIdx;
        }
    }
    circles.resize(endIdx);
}

static void CreateCircles(const std::vector<EstimatedCircle>& circlesEst, std::vector<Vec3f>& circles)
{
    std::transform(circlesEst.begin(), circlesEst.end(), std::back_inserter(circles), GetCircle);
}

static void CreateCircles(const std::vector<EstimatedCircle>& circlesEst, std::vector<Vec4f>& circles)
{
    std::transform(circlesEst.begin(), circlesEst.end(), std::back_inserter(circles), GetCircle4f);
}

template<class NZPoints>
class HoughCircleEstimateRadiusInvoker : public ParallelLoopBody
{
public:
    HoughCircleEstimateRadiusInvoker(const NZPoints &_nz, int _nzSz, const std::vector<int> &_centers, std::vector<EstimatedCircle> &_circlesEst,
                                     int _acols, int _accThreshold, int _minRadius, int _maxRadius,
                                     float _dp, Mutex& _mutex) :
        nz(_nz), nzSz(_nzSz), centers(_centers), circlesEst(_circlesEst), acols(_acols), accThreshold(_accThreshold),
        minRadius(_minRadius), maxRadius(_maxRadius), dr(_dp), _lock(_mutex)
    {
        minRadius2 = (float)minRadius * minRadius;
        maxRadius2 = (float)maxRadius * maxRadius;
        centerSz = (int)centers.size();
        CV_Assert(nzSz > 0);
    }

    ~HoughCircleEstimateRadiusInvoker() {}

protected:
    inline int filterCircles(const Point2f& curCenter, float* ddata) const;

    void operator()(const Range &boundaries) const CV_OVERRIDE
    {
        std::vector<EstimatedCircle> circlesLocal;
        const int nBinsPerDr = 10;
        int nBins = cvRound((maxRadius - minRadius)/dr*nBinsPerDr);
        AutoBuffer<int> bins(nBins);
        AutoBuffer<float> distBuf(nzSz), distSqrtBuf(nzSz);
        float *ddata = distBuf.data();
        float *dSqrtData = distSqrtBuf.data();

        bool singleThread = (boundaries == Range(0, centerSz));
        int i = boundaries.start;

        // For each found possible center
        // Estimate radius and check support
        for(; i < boundaries.end; ++i)
        {
            int ofs = centers[i];
            int y = ofs / acols;
            int x = ofs - y * acols;

            //Calculate circle's center in pixels
            Point2f curCenter = Point2f((x + 0.5f) * dr, (y + 0.5f) * dr);
            int nzCount = filterCircles(curCenter, ddata);

            int maxCount = 0;
            float rBest = 0;
            if(nzCount)
            {
                Mat_<float> distMat(1, nzCount, ddata);
                Mat_<float> distSqrtMat(1, nzCount, dSqrtData);
                sqrt(distMat, distSqrtMat);

                memset(bins.data(), 0, sizeof(bins[0])*bins.size());
                for(int k = 0; k < nzCount; k++)
                {
                    int bin = std::max(0, std::min(nBins-1, cvRound((dSqrtData[k] - minRadius)/dr*nBinsPerDr)));
                    bins[bin]++;
                }

                for(int j = nBins - 1; j > 0; j--)
                {
                    if(bins[j])
                    {
                        int upbin = j;
                        int curCount = 0;
                        for(; j > upbin - nBinsPerDr && j >= 0; j--)
                        {
                            curCount += bins[j];
                        }
                        float rCur = (upbin + j)/2.f /nBinsPerDr * dr + minRadius;
                        if((curCount * rBest >= maxCount * rCur) ||
                            (rBest < FLT_EPSILON && curCount >= maxCount))
                        {
                            rBest = rCur;
                            maxCount = curCount;
                        }
                    }
                }
            }

            // Check if the circle has enough support
            if(maxCount > accThreshold)
            {
                circlesLocal.push_back(EstimatedCircle(Vec3f(curCenter.x, curCenter.y, rBest), maxCount));
            }
        }

        if(!circlesLocal.empty())
        {
            std::sort(circlesLocal.begin(), circlesLocal.end(), cmpAccum);
            if(singleThread)
            {
                std::swap(circlesEst, circlesLocal);
            }
            else
            {
                AutoLock alock(_lock);
                if (circlesEst.empty())
                    std::swap(circlesEst, circlesLocal);
                else
                    circlesEst.insert(circlesEst.end(), circlesLocal.begin(), circlesLocal.end());
            }
        }
    }

private:
    const NZPoints &nz;
    int nzSz;
    const std::vector<int> &centers;
    std::vector<EstimatedCircle> &circlesEst;
    int acols, accThreshold, minRadius, maxRadius;
    float dr;
    int centerSz;
    float minRadius2, maxRadius2;
    Mutex& _lock;
};

template<>
inline int HoughCircleEstimateRadiusInvoker<NZPointList>::filterCircles(const Point2f& curCenter, float* ddata) const
{
    int nzCount = 0;
    const Point* nz_ = &nz[0];
    int j = 0;
#if CV_SIMD
    {
        const v_float32 v_minRadius2 = vx_setall_f32(minRadius2);
        const v_float32 v_maxRadius2 = vx_setall_f32(maxRadius2);

        v_float32 v_curCenterX = vx_setall_f32(curCenter.x);
        v_float32 v_curCenterY = vx_setall_f32(curCenter.y);

        float CV_DECL_ALIGNED(CV_SIMD_WIDTH) rbuf[v_float32::nlanes];
        int CV_DECL_ALIGNED(CV_SIMD_WIDTH) rmask[v_int32::nlanes];
        for(; j <= nzSz - v_float32::nlanes; j += v_float32::nlanes)
        {
            v_float32 v_nzX, v_nzY;
            v_load_deinterleave((const float*)&nz_[j], v_nzX, v_nzY); // FIXIT use proper datatype

            v_float32 v_x = v_cvt_f32(v_reinterpret_as_s32(v_nzX));
            v_float32 v_y = v_cvt_f32(v_reinterpret_as_s32(v_nzY));

            v_float32 v_dx = v_x - v_curCenterX;
            v_float32 v_dy = v_y - v_curCenterY;

            v_float32 v_r2 = (v_dx * v_dx) + (v_dy * v_dy);
            v_float32 vmask = (v_minRadius2 <= v_r2) & (v_r2 <= v_maxRadius2);
            if (v_check_any(vmask))
            {
                v_store_aligned(rmask, v_reinterpret_as_s32(vmask));
                v_store_aligned(rbuf, v_r2);
                for (int i = 0; i < v_int32::nlanes; ++i)
                    if (rmask[i]) ddata[nzCount++] = rbuf[i];
            }
        }
    }
#endif

    // Estimate best radius
    for(; j < nzSz; ++j)
    {
        const Point pt = nz_[j];
        float _dx = curCenter.x - pt.x, _dy = curCenter.y - pt.y;
        float _r2 = _dx * _dx + _dy * _dy;

        if(minRadius2 <= _r2 && _r2 <= maxRadius2)
        {
            ddata[nzCount++] = _r2;
        }
    }
    return nzCount;
}

template<>
inline int HoughCircleEstimateRadiusInvoker<NZPointSet>::filterCircles(const Point2f& curCenter, float* ddata) const
{
    int nzCount = 0;
    const Mat_<uchar>& positions = nz.positions;

    const int rOuter = maxRadius + 1;
    const Range xOuter = Range(std::max(int(curCenter.x - rOuter), 0), std::min(int(curCenter.x + rOuter), positions.cols));
    const Range yOuter = Range(std::max(int(curCenter.y - rOuter), 0), std::min(int(curCenter.y + rOuter), positions.rows));

#if CV_SIMD
    float v_seq[v_float32::nlanes];
    for (int i = 0; i < v_float32::nlanes; ++i)
        v_seq[i] = (float)i;
    const v_float32 v_minRadius2 = vx_setall_f32(minRadius2);
    const v_float32 v_maxRadius2 = vx_setall_f32(maxRadius2);
    const v_float32 v_curCenterX_0123 = vx_setall_f32(curCenter.x) - vx_load(v_seq);
#endif

    for (int y = yOuter.start; y < yOuter.end; y++)
    {
        const uchar* ptr = positions.ptr(y, 0);
        float dy = curCenter.y - y;
        float dy2 = dy * dy;

        int x = xOuter.start;
#if CV_SIMD
        {
            const v_float32 v_dy2 = vx_setall_f32(dy2);
            const v_uint32 v_zero_u32 = vx_setall_u32(0);
            float CV_DECL_ALIGNED(CV_SIMD_WIDTH) rbuf[v_float32::nlanes];
            int CV_DECL_ALIGNED(CV_SIMD_WIDTH) rmask[v_int32::nlanes];
            for (; x <= xOuter.end - v_float32::nlanes; x += v_float32::nlanes)
            {
                v_uint32 v_mask = vx_load_expand_q(ptr + x);
                v_mask = v_mask != v_zero_u32;

                v_float32 v_x = v_cvt_f32(vx_setall_s32(x));
                v_float32 v_dx = v_x - v_curCenterX_0123;

                v_float32 v_r2 = (v_dx * v_dx) + v_dy2;
                v_float32 vmask = (v_minRadius2 <= v_r2) & (v_r2 <= v_maxRadius2) & v_reinterpret_as_f32(v_mask);
                if (v_check_any(vmask))
                {
                    v_store_aligned(rmask, v_reinterpret_as_s32(vmask));
                    v_store_aligned(rbuf, v_r2);
                    for (int i = 0; i < v_int32::nlanes; ++i)
                        if (rmask[i]) ddata[nzCount++] = rbuf[i];
                }
            }
        }
#endif
        for (; x < xOuter.end; x++)
        {
            if (ptr[x])
            {
                float _dx = curCenter.x - x;
                float _r2 = _dx * _dx + dy2;
                if(minRadius2 <= _r2 && _r2 <= maxRadius2)
                {
                    ddata[nzCount++] = _r2;
                }
            }
        }
    }
    return nzCount;
}

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
static bool cmpAccumAlt(const EstimatedCircle& left, const EstimatedCircle& right)
{
    // Compare everything so the order is completely deterministic
    // Larger accum first
    if ( left.accum  < right.accum )
        return true;
    else if( left.accum > right.accum )
        return false;
    else if( left.c[2] < right.c[2] )
        return true;
    else if( left.c[2] > right.c[2] )
        return false;
    else
        return false;
}

static inline Vec4f GetCircleAlt(const EstimatedCircle& est)
{
    return Vec4f(est.c[0], est.c[1], est.c[2], (float)est.accum);
}

static void CreateCirclesAlt(const std::vector<EstimatedCircle>& circlesEst, std::vector<Vec4f>& circles)
{
    std::transform(circlesEst.begin(), circlesEst.end(), std::back_inserter(circles), GetCircleAlt);
}

static void HoughCirclesGradientAlt(InputArray _edges, InputArray _dx, InputArray _dy, OutputArray _circles,
                                 float dp, float minDist,
                                 int minRadius, int maxRadius,
                                 int accThreshold, int maxCircles, bool centersOnly)
{
    dp = max(dp, 1.f);
    float idp = 1.f/dp;

    Mat edges = _edges.getMat();
    Mat dx = _dx.getMat();
    Mat dy = _dy.getMat();
    
    Mutex mtx;
    int numThreads = std::max(1, getNumThreads());
    std::vector<Mat> accumVec;
    NZPointSet nz(_edges.rows(), _edges.cols());
    parallel_for_(Range(0, edges.rows),
                  HoughCirclesAccumInvoker(edges, dx, dy, minRadius, maxRadius, idp, accumVec, nz, mtx),
                  numThreads);
    int nzSz = cv::countNonZero(nz.positions);
    if(nzSz <= 0)
        return;

    Mat accum = accumVec[0];
    for(size_t i = 1; i < accumVec.size(); i++)
    {
        accum += accumVec[i];
    }
    accumVec.clear();

    std::vector<int> centers;

    // 4 rows when multithreaded because there is a bit overhead
    // and on the other side there are some row ranges where centers are concentrated
    parallel_for_(Range(1, accum.rows - 1),
                  HoughCirclesFindCentersInvoker(accum, centers, accThreshold, mtx),
                  (numThreads > 1) ? ((accum.rows - 2) / 4) : 1);

    int centerCnt = (int)centers.size();
    if(centerCnt == 0)
        return;

    std::sort(centers.begin(), centers.end(), hough_cmp_gt(accum.ptr<int>()));

    std::vector<Vec4f> circles;
    circles.reserve(256);
    if (centersOnly)
    {
        // Just get the circle centers
        GetCircleCenters(centers, circles, accum.cols, minDist, dp);
    }
    else
    {
        std::vector<EstimatedCircle> circlesEst;
        if (nzSz < maxRadius * maxRadius)
        {
            // Faster to use a list
            NZPointList nzList(nzSz);
            nz.toList(nzList);
            // One loop iteration per thread if multithreaded.
            parallel_for_(Range(0, centerCnt),
                HoughCircleEstimateRadiusInvoker<NZPointList>(nzList, nzSz, centers, circlesEst, accum.cols,
                    accThreshold, minRadius, maxRadius, dp, mtx),
                numThreads);
        }
        else
        {
            // Faster to use a matrix
            // One loop iteration per thread if multithreaded.
            parallel_for_(Range(0, centerCnt),
                HoughCircleEstimateRadiusInvoker<NZPointSet>(nz, nzSz, centers, circlesEst, accum.cols,
                    accThreshold, minRadius, maxRadius, dp, mtx),
                numThreads);
        }
        if( maxCircles == 1 )
        {
            if( circlesEst.size() > 1 ){
                EstimatedCircle circle = *std::max_element( circlesEst.begin(), circlesEst.end(), cmpAccumAlt );
                Vec4f result( circle.c[0], circle.c[1], circle.c[2], (float)circle.accum );
                circles.clear();
                circles.push_back(result);
            }
            else{
                CreateCirclesAlt(circlesEst, circles);
            }
        }
        else{
            // Sort by accumulator value
            std::sort(circlesEst.begin(), circlesEst.end(), cmpAccum);
            // Create Circles
            CreateCirclesAlt(circlesEst, circles);
            RemoveOverlaps(circles, minDist);
        }
    }
    if (circles.size() > 0)
    {
        int numCircles = std::min(maxCircles, int(circles.size()));
        Mat(1, numCircles, cv::traits::Type<Vec4f>::value, &circles[0]).copyTo(_circles);
        return;
    }
}

void HoughCirclesAlt( InputArray _edges, InputArray _dx, InputArray _dy, OutputArray _circles,
                          int method, double dp, double minDist,
                          double param2,
                          int minRadius, int maxRadius,
                          int maxCircles )
{
    //CV_INSTRUMENT_REGION();

    int type = CV_32FC4;
    if( _circles.fixedType() )
    {
        type = _circles.type();
        CV_CheckType(type, type == CV_32FC4, "Wrong type of output circles");
    }

    CV_Assert(!_edges.empty() && _edges.type() == CV_8UC1 && (_edges.isMat() || _edges.isUMat()));
    CV_Assert(_circles.isMat() || _circles.isVector());

    if( dp <= 0 || minDist <= 0 || param2 <= 0)
        CV_Error( Error::StsOutOfRange, "dp, min_dist and acc_threshold must be all positive numbers" );

    int accThresh = cvRound(param2);

    minRadius = std::max(0, minRadius);

    if(maxCircles < 0)
        maxCircles = INT_MAX;

    bool centersOnly = (maxRadius < 0);

    if( maxRadius <= 0 )
        maxRadius = std::max( _edges.rows(), _edges.cols() );
    else if( maxRadius <= minRadius )
        maxRadius = minRadius + 2;

    switch( method )
    {
    case HOUGH_GRADIENT:
        if (type == CV_32FC4)
            HoughCirclesGradientAlt(_edges, _dx, _dy, _circles, (float)dp, (float)minDist,
                                        minRadius, maxRadius,
                                        accThresh, maxCircles, centersOnly);
        else
            CV_Error(Error::StsError, "Internal error");
        break;
    default:
        CV_Error( Error::StsBadArg, "Unrecognized method id. Actually only CV_HOUGH_GRADIENT is supported." );
    }
}
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
/* End of file. */
