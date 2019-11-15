#ifndef _TELEM_
#define _TELEM_

#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <math.h>
#include <mutex>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include "utils.hpp"
#include "struct.hpp"

using namespace std;
using namespace mavsdk;
using namespace std::chrono;

extern mutex position_ned_mtx;
extern Topic<PositionNED> position_ned_topic;

extern mutex velocity_ned_mtx;
extern Topic<VelocityNED> velocity_ned_topic;

extern mutex velocity_body_mtx;
extern Topic<VelocityBody> velocity_body_topic;

extern mutex attitude_mtx;
extern Topic<EulerAngle> attitude_topic;

extern mutex vehicle_status_mtx;
extern Topic<VehicleStatus> vehicle_status_topic;

extern mutex string_mtx;
extern Topic<string> string_topic;

void setTelemetry( shared_ptr<Telemetry> telemetry );

#endif