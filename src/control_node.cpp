#include "control_node.hpp"

void controlLoop( FileNode control_config, FileNode altitude_pid, FileNode vision_pid, FileNode flow_pid, FileNode mission_param )
{
    int enable;
    string url;

    control_config["ENABLE"] >> enable;
    control_config["URL"] >> url;
    if( enable == 0 )
    {
        cout << "[WARNING]: control node disabled" << endl;
        return;
    }

    /*Connect to pixhawk*/
    Mavsdk dc;
    ConnectionResult connection_result;
    connection_result = dc.add_any_connection(url);
    if (connection_result != ConnectionResult::SUCCESS) {
        cout <<  string("[ERROR]: ") + connection_result_str(connection_result) << endl;
        cout << "[WARNING]: control node shut down" << endl;
        return;
    }
    while (!dc.is_connected()) {
        cout << "[LOGGING]: Wait for system to connect via heartbeat" << endl;
        this_thread::sleep_for(seconds(1));
    }
    System& system = dc.system();
    auto action = std::make_shared<Action>(system);
    auto offboard = std::make_shared<Offboard>(system);
    auto telemetry = std::make_shared<Telemetry>(system);
    /*Health Check*/
    //healthCheck( telemetry );
    setTelemetry( telemetry );
    
    #if (TEST_LEVEL == LEVEL_FINAL0) || (TEST_LEVEL == LEVEL_FINAL1) || (TEST_LEVEL == LEVEL_FINAL2) || (TEST_LEVEL == LEVEL_FINAL3)
    missionLoop( telemetry, offboard, altitude_pid, vision_pid, flow_pid, mission_param);
    #else
    testLoop( telemetry, offboard, altitude_pid, vision_pid, flow_pid);
    #endif
    
    cout << "[WARNING]: control node shut down" << endl;
    return;
}