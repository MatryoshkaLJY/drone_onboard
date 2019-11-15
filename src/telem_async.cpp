#include "telem_async.hpp"

void setTelemetry( shared_ptr<Telemetry> telemetry )
{
    Telemetry::Result set_rate_result;
    set_rate_result = telemetry->set_rate_position_velocity_ned( 250 );
    if (set_rate_result != Telemetry::Result::SUCCESS) {
        cout << string("[ERROR]: ") + Telemetry::result_str(set_rate_result) << endl;
        return;
    }
    set_rate_result = telemetry->set_rate_attitude( 250 );
    if (set_rate_result != Telemetry::Result::SUCCESS) {
        cout << string("[ERROR]: ") + Telemetry::result_str(set_rate_result) << endl;
        return;
    }

    // Set up callback to monitor altitude while the vehicle is in flight
    telemetry->position_velocity_ned_async([](Telemetry::PositionVelocityNED position_velocity_ned){
        PositionNED position_ned;
        VelocityNED velocity_ned;
        VelocityBody velocity_body;
        EulerAngle attitude;
        int64_t timestamp;
        
        // position_ned.north_m = position_velocity_ned.position.north_m;
        // position_ned.east_m = position_velocity_ned.position.east_m;
        // position_ned.down_m = position_velocity_ned.position.down_m;
        memcpy(&position_ned, &position_velocity_ned.position, sizeof position_ned);
        update<PositionNED>(position_ned_topic, position_ned, position_ned_mtx);

        // velocity_ned.north_m_s = position_velocity_ned.velocity.north_m_s;
        // velocity_ned.east_m_s = position_velocity_ned.velocity.east_m_s;
        // velocity_ned.down_m_s = position_velocity_ned.velocity.down_m_s;
        memcpy(&velocity_ned, &position_velocity_ned.velocity, sizeof velocity_ned);
        update<VelocityNED>(velocity_ned_topic, velocity_ned, velocity_ned_mtx);

        if( attitude_topic.latest(timestamp, attitude) )
        {
            velocity_body = velocityNED2Body(velocity_ned, attitude);
            update<VelocityBody>(velocity_body_topic, velocity_body, velocity_body_mtx);
        }
    });

    telemetry->attitude_euler_angle_async([](Telemetry::EulerAngle attitude_euler_angle){
        EulerAngle euler_angle;
        // euler_angle.roll_deg = attitude_euler_angle.roll_deg;
        // euler_angle.pitch_deg = attitude_euler_angle.pitch_deg;
        // euler_angle.yaw_deg = attitude_euler_angle.yaw_deg;
        memcpy(&euler_angle, &attitude_euler_angle, sizeof euler_angle);
        update<EulerAngle>(attitude_topic, euler_angle, attitude_mtx);
    });

    telemetry->armed_async([](bool armed){
        VehicleStatus vehicle_status;
        int64_t timestamp;
        latest<VehicleStatus>(vehicle_status_topic, timestamp, vehicle_status, vehicle_status_mtx);
        vehicle_status.armed = armed;
        update<VehicleStatus>(vehicle_status_topic, vehicle_status, vehicle_status_mtx);
    });

    telemetry->in_air_async([](bool in_air){
        VehicleStatus vehicle_status;
        int64_t timestamp;
        latest<VehicleStatus>(vehicle_status_topic, timestamp, vehicle_status, vehicle_status_mtx);
        vehicle_status.in_air = in_air;
        update<VehicleStatus>(vehicle_status_topic, vehicle_status, vehicle_status_mtx);
    });

    telemetry->rc_status_async([](Telemetry::RCStatus rc_status){
        VehicleStatus vehicle_status;
        int64_t timestamp;
        latest<VehicleStatus>(vehicle_status_topic, timestamp, vehicle_status, vehicle_status_mtx);
        vehicle_status.rc_available_once = rc_status.available_once;
        vehicle_status.rc_available = rc_status.available;
        vehicle_status.rc_signal_strength_percent = rc_status.signal_strength_percent;
        update<VehicleStatus>(vehicle_status_topic, vehicle_status, vehicle_status_mtx);
    });

    telemetry->battery_async([](Telemetry::Battery battery){
        VehicleStatus vehicle_status;
        int64_t timestamp;
        latest<VehicleStatus>(vehicle_status_topic, timestamp, vehicle_status, vehicle_status_mtx);
        vehicle_status.battery_voltage_v = battery.voltage_v;
        vehicle_status.battery_remaining_percent = battery.remaining_percent;
        update<VehicleStatus>(vehicle_status_topic, vehicle_status, vehicle_status_mtx);
    });

    telemetry->flight_mode_async([](Telemetry::FlightMode flight_mode){
        string mode = Telemetry::flight_mode_str(flight_mode);
        VehicleStatus vehicle_status;
        int64_t timestamp;
        latest<VehicleStatus>(vehicle_status_topic, timestamp, vehicle_status, vehicle_status_mtx);
        strncpy(vehicle_status.flight_mode, mode.c_str(), sizeof(vehicle_status.flight_mode));
        vehicle_status.flight_mode[sizeof(vehicle_status.flight_mode) - 1] = 0;
        update<VehicleStatus>(vehicle_status_topic, vehicle_status, vehicle_status_mtx);
    });

    telemetry->status_text_async([](Telemetry::StatusText status_text){
        string prefix;
        switch (status_text.type){
            case Telemetry::StatusText::StatusType::CRITICAL:
                prefix = "[CRITICAL]: ";
                break;
            case Telemetry::StatusText::StatusType::INFO:
                prefix = "[INFO]: ";
                break;
            case Telemetry::StatusText::StatusType::WARNING:
                prefix = "[WARNING]: ";
                break;
            default:
                prefix = "[UNKNOWN]: ";
                break;
        }
        string msg = prefix + status_text.text;
        update<string>(string_topic, msg, string_mtx);
    });
    return;
}