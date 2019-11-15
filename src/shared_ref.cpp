#include <opencv2/core/core.hpp>
#include <chrono>
#include <unistd.h>
#include <linux/in.h>
#include <mutex>
#include <iostream>
#include <fstream>

#include "struct.hpp"
#include "topic.hpp"

using namespace std;
using namespace cv;
using namespace std::chrono;

/*declaration of shared constant*/
high_resolution_clock::time_point init_timepoint = high_resolution_clock::now();

/*declaration of shared topics( or variables ) between nodes,
CAUTIOUS: every topics must own a mutex*/
/*sockets_node*/
int fd = -1;
mutex fd_mutex;

mutex image_mtx;
Topic<Mat> image_topic(init_timepoint, 1);

mutex camera_status_mtx;
Topic<int> camera_status_topic(init_timepoint, 1);

mutex first_target_mtx;
Topic<DetectionResult> first_target_topic(init_timepoint, 10);

mutex second_target_mtx;
Topic<DetectionResult> second_target_topic(init_timepoint, 10);

mutex third_target_mtx;
Topic<DetectionResult> third_target_topic(init_timepoint, 10);

mutex string_mtx;
Topic<string> string_topic(init_timepoint, 5);

mutex position_ned_mtx;
Topic<PositionNED> position_ned_topic(init_timepoint, 40);

mutex velocity_ned_mtx;
Topic<VelocityNED> velocity_ned_topic(init_timepoint, 40);

mutex velocity_body_mtx;
Topic<VelocityBody> velocity_body_topic(init_timepoint, 40);

mutex attitude_mtx;
Topic<EulerAngle> attitude_topic(init_timepoint, 40);

mutex vehicle_status_mtx;
Topic<VehicleStatus> vehicle_status_topic(init_timepoint, 40);

mutex input_attitude_mtx;
Topic<InputAttitude> input_attitude_topic(init_timepoint, 40);

mutex down_reference_mtx;
Topic<float> down_reference_topic(init_timepoint, 40);

mutex ne_reference_mtx;
Topic<Vector2f> ne_reference_topic(init_timepoint, 40);

mutex mission_command_mtx;
Topic<MissionCommand> mission_command_topic(init_timepoint, 1);

mutex control_status_mtx;
Topic<int16_t> control_status_topic(init_timepoint, 10);

mutex pos_err_xy_mtx;
Topic<Vector2f> pos_err_xy_topic(init_timepoint, 40);