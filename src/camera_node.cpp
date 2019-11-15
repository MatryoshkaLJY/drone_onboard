#include "camera_node.hpp"

void cameraLoop( FileNode camera_config )
{
    int enable;
    int id, width, height;
    string calib_file;
    camera_config["ENABLE"] >> enable;
    camera_config["ID"] >> id;
    camera_config["WIDTH"] >> width;
    camera_config["HEIGHT"] >> height;
    camera_config["CAMERA_INTRINSIC_XML"] >> calib_file;
    FileStorage calib(calib_file, FileStorage::READ);
    Mat camera_matrix, distort_coeff;
    calib["camera_matrix"] >> camera_matrix;
    calib["distortion_coefficients"] >> distort_coeff;
    if( enable == 0 )
    {
        cout << "[WARNING]: camera node disabled" << endl;
        return;
    }

    Mat image;
    int camera_status = 2;
    int64_t timestamp;

    //VideoCapture cap("../testset/demo4.avi");
    mutex cap_mutex;
    Camera cap(cap_mutex);
    Video video("../", width, height);
    CircleDetector first_detector(DETECT_AND_TRACK, GREEN, 0);
    CircleDetector second_detector(DETECT_AND_TRACK, BLUE, 0);
    SquareDetector third_detector(RED);

    Rect2f roi;
    float confidence, distance;
    Vec3f orientation;
    DetectionResult result;
    Mat src;
    if ( ! cap.init( id, width, height) ){
        cout << "[ERROR]: fail to init camera" << endl;
        cout << "[WARNING]: camera node shut down" << endl;
        return;
    }
    while( true )
    {
        if (latest<int>(camera_status_topic, timestamp, camera_status, camera_status_mtx))
		{
			clear<int>(camera_status_topic, camera_status_mtx);
		}

        cap.read( image );
        video.open();
        video.writeImage(image.clone());
        //cout << image.clone().empty() << endl;
        switch( camera_status )
        {
        case 0:
            update<Mat>(image_topic, image.clone(), image_mtx);
            break;
        case 1:
            src = image.clone();
            if( first_detector.run(src, roi, confidence) )
            {
                solvePosition(roi, distance, orientation, camera_matrix, distort_coeff, FIRST_RING_DIAMETER_M);
                result.index = 0;
                result.confidence = confidence;
                result.distance_m = distance;
                result.x_m = orientation[0];
                result.y_m = orientation[1];
                result.z_m = orientation[2];
                update<DetectionResult>(first_target_topic, result, first_target_mtx);
                rectangle( image, roi, Scalar( 0, 255, 0 ), 8, 1 );
            }
            update<Mat>(image_topic, image.clone(), image_mtx);
            //cout << "TARGET 1 DETECTION DISABLE!" << endl;
            break;
        case 2:
            src = image.clone();
            if( second_detector.run(src, roi, confidence) )
            {
                solvePosition(roi, distance, orientation, camera_matrix, distort_coeff, SECOND_RING_DIAMETER_M);
                result.index = 1;
                result.confidence = confidence;
                result.distance_m = distance;
                result.x_m = orientation[0];
                result.y_m = orientation[1];
                result.z_m = orientation[2];
                update<DetectionResult>(second_target_topic, result, second_target_mtx);
                rectangle( image, roi, Scalar( 0, 255, 0 ), 8, 1 );
            }
            update<Mat>(image_topic, image.clone(), image_mtx);
            break;
        case 3:
            src = image.clone();
            if( third_detector.run(src, roi, confidence) )
            {
                solvePosition(roi, distance, orientation, camera_matrix, distort_coeff, THIRD_RING_DIAMETER_M);
                result.index = 2;
                result.confidence = confidence;
                result.distance_m = distance;
                result.x_m = orientation[0];
                result.y_m = orientation[1];
                result.z_m = orientation[2];
                update<DetectionResult>(third_target_topic, result, third_target_mtx);
                //update<DetectionResult>(third_target_topic, result, third_target_mtx);
                rectangle( image, roi, Scalar( 0, 255, 0 ), 8, 1 );
            }
            update<Mat>(image_topic, image.clone(), image_mtx);
            break;
        case 4:
            src = image.clone();
            if( first_detector.run(src, roi, confidence) )
            {
                solvePosition(roi, distance, orientation, camera_matrix, distort_coeff, FIRST_RING_DIAMETER_M);
                result.index = 0;
                result.confidence = confidence;
                result.distance_m = distance;
                result.x_m = orientation[0];
                result.y_m = orientation[1];
                result.z_m = orientation[2];
                update<DetectionResult>(first_target_topic, result, first_target_mtx);
                rectangle( image, roi, Scalar( 0, 255, 0 ), 8, 1 );
            }
            if( second_detector.run(src, roi, confidence) )
            {
                solvePosition(roi, distance, orientation, camera_matrix, distort_coeff, SECOND_RING_DIAMETER_M);
                result.index = 1;
                result.confidence = confidence;
                result.distance_m = distance;
                result.x_m = orientation[0];
                result.y_m = orientation[1];
                result.z_m = orientation[2];
                update<DetectionResult>(second_target_topic, result, second_target_mtx);
                rectangle( image, roi, Scalar( 0, 255, 0 ), 8, 1 );
            }
            update<Mat>(image_topic, image.clone(), image_mtx);
            break;
        case 5:
            cout << "REC!" << endl;
            video.open();
            video.writeImage(image.clone());
            update<Mat>(image_topic, image.clone(), image_mtx);
            break;
        default:
            break;
        }
    }
    cout << "[WARNING]: camera node shut down" << endl;
    return;
}

void cameraLoopTest()
{
    int64_t timestamp;
    Mat image;
    while( true )
    {
        latest<Mat>(image_topic, timestamp, image, image_mtx);
        if( image.empty() ){
            continue;
        }
        imshow( "image", image );
        waitKey(30);
    }
    return;
}

Camera::Camera(mutex& mtx):
    cap_mutex(mtx)
{
    return;
}
bool Camera::init( int id, int _width, int _height)
{
    if ( ! cap.open( id ) ){
        cout << "[ERROR]: fail to open camera" << endl;
        return false;
    }
    cap.set( CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    //cap.set( CAP_PROP_FOURCC, VideoWriter::fourcc('Y', 'U', 'Y', '2'));
    cap.set( CAP_PROP_FRAME_WIDTH, _width);
    cap.set( CAP_PROP_FRAME_HEIGHT, _height);
    cap.set( CAP_PROP_BUFFERSIZE, 2);
    //cout << cap.get(CV_CAP_PROP_BUFFERSIZE) << endl;
    width = cap.get( CAP_PROP_FRAME_WIDTH );
    height = cap.get( CAP_PROP_FRAME_HEIGHT);
    cap.set(CAP_PROP_CONTRAST, 100);
    cap.set(CAP_PROP_BRIGHTNESS, -60);
    cap.set(CAP_PROP_SATURATION, 100);
    cap.set(CAP_PROP_SHARPNESS, 10);
    cap.set(CAP_PROP_GAIN, 80);
    cap.set(CAP_PROP_HUE, 360);
    fps = cap.get(CAP_PROP_FPS);
    if ( width != _width || height != _height ){
        cout << "[ERROR]: set camera resolution fail" << endl;
        return false;
    }
    interval = 1000.0 / fps;
    for ( int i = 5; i > 0; i--)
        cap.grab();
    new_frame = true;
    last_grab = high_resolution_clock::now();
    thread updating_thread( &Camera::updating, this );
    updating_thread.detach();
    return true;
}

bool Camera::isOpened( void ){
    cap_mutex.lock();
    bool flag = cap.isOpened();
    cap_mutex.unlock();
    return flag;
}

bool Camera::read( Mat& image ){
    cap_mutex.lock();
    if ( ! cap.isOpened() ){
        cap_mutex.unlock();
        cout << "[ERROR]: cap closed" << endl;
        return false;
    }
    if ( intervalMs( high_resolution_clock::now(), last_grab ) > (int64_t) (interval * 0.3) || new_frame == false ){
        cap.grab();
        new_frame = true;
        last_grab = high_resolution_clock::now();
    }
    bool flag = cap.retrieve( image );
    new_frame = false;
    cap_mutex.unlock();
    return flag;
}

void Camera::release( void ){
    cap_mutex.lock();
    cap.release();
    cap_mutex.unlock();
    return;
}

void Camera::updating( void ){
    while( true ){
        cap_mutex.lock();
        if ( ! cap.isOpened() ){
            cap_mutex.unlock();
            cout << "[ERROR]: cap closed" << endl;
            break;
        }
        if ( intervalMs(high_resolution_clock::now(), last_grab) > (int64_t) (interval * 0.9) ){
            cap.grab();
            new_frame = true;
            last_grab = high_resolution_clock::now();
        }
        cap_mutex.unlock();
        this_thread::sleep_for(milliseconds( (int) interval ) );
    }
    return;
}

Video::Video( string path, int img_width, int img_height )
{
    video_path = path;
    width = img_width;
    height = img_height;
}

void Video::open()
{
    if( ! writer.isOpened() )
    {
        writer.open( video_path + getCurrentTime() + string(".avi"), VideoWriter::fourcc('D', 'I', 'V', 'X'), 30, Size(width, height) );
    }
}

void Video::close()
{
    if( writer.isOpened() )
    {
        writer.release();
    }
}

void Video::writeImage(Mat image)
{
    if( writer.isOpened() )
    {
        if( image.empty() )
            return;
        resize(image, image, Size(width, height));
        writer.write( image );
    }
}