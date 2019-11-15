#include "control_sync.hpp"

void FlowPosThrustControl::reset(FileNode flow_pid, FileNode altitude_pid) {
	flow_pid["Kp_x"] >> Kp_x;
	flow_pid["Ki_x"] >> Ki_x;
	flow_pid["Kd_x"] >> Kd_x;
	flow_pid["Kp_y"] >> Kp_y;
	flow_pid["Ki_y"] >> Ki_y;
	flow_pid["Kd_y"] >> Kd_y;
    Kp = {Kp_x, Kp_y};
    Ki = {Ki_x, Ki_y};
    Kd = {Kd_x, Kd_y};
	int_pos_xy = { 0.0f,0.0f };
	altitude_thrust_control.reset(altitude_pid);
    cout << "FLOW PID:" << endl;
    cout << this->Kp_x << endl;
    cout << this->Ki_x << endl;
    cout << this->Kd_x << endl;

    cout << this->Kp_y << endl;
    cout << this->Ki_y << endl;
    cout << this->Kd_y << endl;
	return;
}

void FlowPosThrustControl::positionBodyOffset( float& roll_deg, float& pitch_deg, float& thrust, Vector3f offset_body, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms)
{
    time_change = dt_ms / 1000.0f;
    Vector2f pos_ne = {pos_ned.north_m, pos_ned.east_m};
    pos_err_xy = {offset_body.x, offset_body.y};
    pos_err_ne = xy2ne(pos_err_xy, attitude.yaw_deg);
    pos_sp_ne = pos_ne + pos_err_ne;
    vel_err_xy = {-vel_body.x_m_s, -vel_body.y_m_s};
    alt_thrust = altitude_thrust_control.downOffset(offset_body.z, pos_ned, vel_body, attitude, dt_ms);
    calcRollPitchThrust(roll_deg, pitch_deg, thrust,attitude);
}

void FlowPosThrustControl::hold(float& roll_deg, float& pitch_deg, float& thrust, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms)
{
    time_change = dt_ms / 1000.0f;
    Vector2f pos_ne = {pos_ned.north_m, pos_ned.east_m};
    pos_err_ne = pos_sp_ne - pos_ne;
    pos_err_xy = ne2xy(pos_err_ne, attitude.yaw_deg);
    vel_err_xy = {-vel_body.x_m_s, -vel_body.y_m_s};
    alt_thrust = altitude_thrust_control.hold(pos_ned, vel_body, attitude, dt_ms);
    calcRollPitchThrust(roll_deg, pitch_deg, thrust,attitude);
}

void FlowPosThrustControl::calcRollPitchThrust(float& roll_deg, float& pitch_deg, float& thrust,EulerAngle attitude)
{
    update<Vector2f>(pos_err_xy_topic, pos_err_xy, pos_err_xy_mtx);
    update<Vector2f>(ne_reference_topic, pos_sp_ne, ne_reference_mtx);

    Vector2f thrust_xy = {0.0f, 0.0f};
    Vector2f thrust_desired = {0.0f, 0.0f};
    thrust_desired = Kp * pos_err_xy + Kd * vel_err_xy + int_pos_xy;
    thrust_xy = thrust_desired;

    float thrust_max_NE_tilt = fabsf(alt_thrust) * tanf(tilt_max);//while in take_off or landing state i think the "cos(_pitch) * cos(_roll) = 1" => alt_thrust = thrust_z 
	float thrust_max_NE = sqrtf(thrust_max * thrust_max - alt_thrust * alt_thrust);
	thrust_max_NE = thrust_max_NE_tilt < thrust_max_NE ? thrust_max_NE_tilt : thrust_max_NE;
	// Saturate thrust in NE-direction.

	if ((thrust_xy.x * thrust_xy.x + thrust_xy.y * thrust_xy.y) > thrust_max_NE * thrust_max_NE) {
		float mag = sqrtf(thrust_xy.x * thrust_xy.x + thrust_xy.y * thrust_xy.y);
		thrust_xy = thrust_xy / ( mag / thrust_max_NE);
	}
	// Use tracking Anti-Windup for NE-direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	float arw_gain = 2.f / Kp.x;

	Vector2f vel_err_lim = { 0.0f,0.0f };
	vel_err_lim = pos_err_xy - (thrust_desired - thrust_xy) * arw_gain;


	// Update integral
	int_pos_xy = int_pos_xy + Ki * vel_err_lim * time_change;
    Vector3f thrust_xyz = {thrust_xy.x, thrust_xy.y, alt_thrust};
    //thrust = alt_thrust;
    thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) ); //kfymention: thrust = alt_thrust;
    roll_deg = rad2deg( atan2f(thrust_xyz.y, thrust_xyz.z) );
    pitch_deg = rad2deg( atan2f(-thrust_xyz.x, sqrtf(thrust_xyz.y*thrust_xyz.y+thrust_xyz.z*thrust_xyz.z)) );
    //roll_deg = rad2deg( asinf(thrust_xyz.y / thrust_xyz.z) );
	//pitch_deg = rad2deg(-asinf(thrust_xyz.x / thrust_xyz.z) );

    roll_deg=limit_values(roll_deg,-10.0f,10.0f);
    pitch_deg= limit_values(pitch_deg,-10.0f,10.0f);
}
void FlowPosThrustControl::climb(float altitude_set,float& roll_deg, float& pitch_deg, float& thrust, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms)
{
    time_change = dt_ms / 1000.0f;
    Vector2f pos_ne = {pos_ned.north_m, pos_ned.east_m};
    pos_err_ne = pos_sp_ne - pos_ne;
    pos_err_xy = ne2xy(pos_err_ne, attitude.yaw_deg);
    vel_err_xy = {-vel_body.x_m_s, -vel_body.y_m_s};
    alt_thrust = altitude_thrust_control.down(altitude_set,pos_ned, vel_body, attitude, dt_ms);
    calcRollPitchThrust(roll_deg, pitch_deg, thrust,attitude);
}

void VisionRollThrustControl::reset(FileNode vision_pid, FileNode altitude_pid)
{
    vision_pid["Ky"] >> Ky;
    vision_pid["Kz"] >> Kz;
    vision_pid["Kp_y"] >> Kp_y;
    vision_pid["Ki_y"] >> Ki_y;
    vision_pid["Kd_y"] >> Kd_y;
    vision_pid["Kp_x_vel"] >> Kp_x_vel;
    vision_pid["Ki_x_vel"] >> Ki_x_vel;

    altitude_thrust_control.reset(altitude_pid);

    cout << "Vision Gain:" << endl;
    cout << this->Ky << endl;
    cout << this->Kz << endl;

    cout << "Vision PID:" << endl;
    cout << this->Kp_y << endl;
    cout << this->Ki_y << endl;
    cout << this->Kd_y << endl;
    cout << this->Kp_x_vel << endl;
    cout << this->Ki_x_vel << endl;

    K_lock=false;
    Y_RAD_LOCK=false;
    int_pos_y=0;
    vel_x_int=0;
    target_x_m=10;
    target_y_m=0;
    target_z_m=0;
    can_through_ring_flag = false;
    can_start_the_first_thr = false;
}

/*
void VisionRollThrustControl::angleOffset( float& roll_deg, float& thrust, DetectionResult target, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms )
{
    time_change = dt_ms / 1000.0f;
    float roll_rad = deg2rad( attitude.roll_deg );
    //convert target from camera to body
    float target_y_m = target.x_m * cos( roll_rad ) - target.y_m * sin( roll_rad );
    float target_z_m = target.x_m * sin( roll_rad ) + target.y_m * cos( roll_rad );
    float target_x_m = target.z_m;
    z_deg = rad2deg( atan2f(target_z_m, target_x_m) );
    pos_sp_z = Kz * z_deg + pos_ned.down_m;
    alt_thrust = altitude_thrust_control.down(pos_sp_z, pos_ned, vel_body, attitude, dt_ms);

    vel_y = vel_body.y_m_s;
    vel_sp_y = vel_body.x_m_s * target_y_m / target_x_m;
    roll_deg = calcRoll();
    thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) );
    limit_values(calcRoll(), -45.0f, 45.0f);
    limit_values(thrust, 0.25f, 0.85f);
    return;
}

void VisionRollThrustControl::hold(float& roll_deg, float& thrust, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms)
{
    time_change = dt_ms / 1000.0f;
    alt_thrust = altitude_thrust_control.hold(pos_ned, vel_body, attitude, dt_ms);
    
    vel_y = vel_body.y_m_s;
    roll_deg = calcRoll();
    thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) );
    limit_values(calcRoll(), -45.0f, 45.0f);
    limit_values(thrust, 0.25f, 0.85f);
    return;
}

float VisionRollThrustControl::calcRoll()
{
    vel_err_y = vel_sp_y - vel_y;
    return Kp_y * vel_err_y;
}
*/
void VisionRollThrustControl::angleOffset( float& roll_deg, float& thrust, Target target, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms )
{
    time_change = dt_ms / 1000.0f;

    if( target.x_m > 4.0f && ! K_lock)
    {
        Ky = 1.0f;
    }
    else{
        Ky = 0.5f;
        K_lock = true;
    }
    
    target_x_m = target.x_m;
    target_y_m = target.y_m;
    target_z_m = target.z_m;
    y_rad = deg2rad(target.y_deg);
    z_rad = deg2rad(target.z_deg);

    err_pos_y = Ky * y_rad;
    err_pos_z = Kz * z_rad;
    vel_err = -vel_body.y_m_s;

    if( target.x_m > 5.0f && !Y_RAD_LOCK )
    {
        pos_sp_z = pos_ned.down_m + err_pos_z;
    }
    else{
        Y_RAD_LOCK = true;
    }
    alt_thrust = altitude_thrust_control.down(pos_sp_z, pos_ned, vel_body, attitude, dt_ms);

    roll_deg = calcRoll();
	thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) );
    return;
}


void VisionRollThrustControl::angleOffset_vel(float vel_set, float& pitch_deg, float& roll_deg, float& thrust, Target target, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms )
{
    time_change = dt_ms / 1000.0f;

    if( target.x_m > 4.0f && ! K_lock)
    {
        Ky = 1.0f;
    }
    else{
        Ky = 0.5f;
        K_lock = true;
    }
    
    target_x_m = target.x_m;
    target_y_m = target.y_m;
    target_z_m = target.z_m;
    y_rad = deg2rad(target.y_deg);
    z_rad = deg2rad(target.z_deg);

    err_pos_y = Ky * y_rad;
    err_pos_z = Kz * z_rad;
    vel_err = -vel_body.y_m_s;
    vel_x_err = vel_set - vel_body.x_m_s;
    
    if( target.x_m > 5.0f && !Y_RAD_LOCK )
    {
        pos_sp_z = pos_ned.down_m + err_pos_z;
    }
    else{
        Y_RAD_LOCK = true;
    }
    alt_thrust = altitude_thrust_control.down(pos_sp_z, pos_ned, vel_body, attitude, dt_ms);
    Vector2f thrust_xy;
    thrust_xy.x= Kp_x_vel*vel_x_err;
    thrust_xy.y= Kp_y * err_pos_y + Kd_y * vel_err + int_pos_y;
    thrust_desired_y=thrust_xy.y;
    float thrust_max_tilt = fabsf(alt_thrust) * tanf(tilt_max);//while in take_off or landing state i think the "cos(_pitch) * cos(_roll) = 1" => alt_thrust = thrust_z 
	float thrust_max_xy = sqrtf(thrust_max * thrust_max - alt_thrust * alt_thrust);
	thrust_max_xy = thrust_max_tilt < thrust_max_xy ? thrust_max_tilt : thrust_max_xy;
	// Saturate thrust in NE-direction.
    float mag = mag2f(thrust_xy);
	if (mag * mag > thrust_max_xy * thrust_max_xy) {
		thrust_xy = thrust_xy / (mag / thrust_max_xy);
	}
	// Use tracking Anti-Windup for NE-direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	float arw_gain = 2.f / Kp_y;
    float vel_err_lim = 0.0f;
	vel_err_lim = err_pos_y - (thrust_desired_y - thrust_xy.y) * arw_gain;
	
    // Update integral
	int_pos_y = int_pos_y + Ki_y * vel_err_lim * time_change;
	//_thr_int(1) += _param_mpc_xy_vel_i.get() * vel_err_lim(1) * dt;

    Vector3f thrust_xyz = {thrust_xy.x, thrust_xy.y, alt_thrust};
    roll_deg = rad2deg( atan2f(thrust_xyz.y, thrust_xyz.z) );
    pitch_deg = rad2deg( atan2f(-thrust_xyz.x, sqrtf(thrust_xyz.y*thrust_xyz.y+thrust_xyz.z*thrust_xyz.z)) );

    roll_deg=limit_values(roll_deg,-30.0f,30.0f);
    pitch_deg= limit_values(pitch_deg,-30.0f,30.0f);

    thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) );

    return;
}

void VisionRollThrustControl::angleOffset_vel_z(float vel_set, float _pos_sp_z, float& pitch_deg, float& roll_deg, float& thrust, Target target, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms )
{
    time_change = dt_ms / 1000.0f;

    if( target.x_m > 4.0f && ! K_lock)
    {
        Ky = 1.0f;
    }
    else{
        Ky = 0.5f;
        K_lock = true;
    }
    
    target_x_m = target.x_m;
    target_y_m = target.y_m;
    //target_z_m = target.z_m;
    y_rad = deg2rad(target.y_deg);
    //z_rad = deg2rad(target.z_deg);

    err_pos_y = Ky * y_rad;
    //err_pos_z = Kz * z_rad;
    vel_err = -vel_body.y_m_s;
    vel_x_err = vel_set - vel_body.x_m_s;
    // if( target.x_m > 5.0f && !Y_RAD_LOCK )
    // {
    //     pos_sp_z = pos_ned.down_m + err_pos_z;
    // }
    // else{
    //     Y_RAD_LOCK = true;
    // }
    pos_sp_z = _pos_sp_z;
    alt_thrust = altitude_thrust_control.down(pos_sp_z, pos_ned, vel_body, attitude, dt_ms);
    Vector2f thrust_xy;
    thrust_xy.x= Kp_x_vel*vel_x_err+ Ki_x_vel* vel_x_int;
    thrust_xy.y= Kp_y * err_pos_y + Kd_y * vel_err + int_pos_y;
    thrust_desired_y=thrust_xy.y;
    float thrust_max_tilt = fabsf(alt_thrust) * tanf(tilt_max);//while in take_off or landing state i think the "cos(_pitch) * cos(_roll) = 1" => alt_thrust = thrust_z 
	float thrust_max_xy = sqrtf(thrust_max * thrust_max - alt_thrust * alt_thrust);
	thrust_max_xy = thrust_max_tilt < thrust_max_xy ? thrust_max_tilt : thrust_max_xy;
	// Saturate thrust in NE-direction.
    float mag = mag2f(thrust_xy);
	if (mag * mag > thrust_max_xy * thrust_max_xy) {
		thrust_xy = thrust_xy / (mag / thrust_max_xy);
	}
	// Use tracking Anti-Windup for NE-direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	float arw_gain = 2.f / Kp_y;
    float vel_err_lim = 0.0f;
	vel_err_lim = err_pos_y - (thrust_desired_y - thrust_xy.y) * arw_gain;
	
    // Update integral
	int_pos_y = int_pos_y + Ki_y * vel_err_lim * time_change;
	//_thr_int(1) += _param_mpc_xy_vel_i.get() * vel_err_lim(1) * dt;
    vel_x_int += vel_x_err;
    if(vel_x_int>200 ){
        vel_x_int=200;
    }else if(vel_x_int< -200){
        vel_x_int=-200;
    }

    Vector3f thrust_xyz = {thrust_xy.x, thrust_xy.y, alt_thrust};
    roll_deg = rad2deg( atan2f(thrust_xyz.y, thrust_xyz.z) );
    pitch_deg = rad2deg( atan2f(-thrust_xyz.x, sqrtf(thrust_xyz.y*thrust_xyz.y+thrust_xyz.z*thrust_xyz.z)) );

    roll_deg=limit_values(roll_deg,-30.0f,30.0f);
    pitch_deg= limit_values(pitch_deg,-30.0f,30.0f);

    thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) );

    return;
}
void VisionRollThrustControl::collimation1(  float& yaw_deg, float& thrust, Target target1,PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms ){
    time_change = dt_ms / 1000.0f;
    //float roll_rad = deg2rad( attitude.roll_deg );
    y_rad = deg2rad(target1.y_deg);
    z_rad = deg2rad(target1.z_deg);

    Kz = 1.0f;
    //err_pos_z = Kz * z_rad;//z - no collimation
    err_pos_z = -Kz * (z_f_rad - z_rad);//z - collimation
    

    pos_sp_z = pos_ned.down_m + err_pos_z;
    alt_thrust = altitude_thrust_control.down(pos_sp_z, pos_ned, vel_body, attitude, dt_ms);
    yaw_deg = attitude.yaw_deg + rad2deg(y_rad);
	thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) );
    return;
}

void VisionRollThrustControl::collimation2(  float& roll_deg, float& thrust, Target target2, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms ){
    time_change = dt_ms / 1000.0f;

    z_f_rad = deg2rad(target2.z_deg);
    y_f_rad = deg2rad(target2.y_deg);

    Ky = 0.5f; 
    Kz = 1.0f;
    err_pos_y = -Ky * (y_f_rad - y_rad);//y - collimation
    //err_pos_z = Kz * z_rad;//z - no collimation
    err_pos_z = -Kz * (z_f_rad - z_rad);//z - collimation
    vel_err = -vel_body.y_m_s;

    pos_sp_z = pos_ned.down_m + err_pos_z;
    alt_thrust = altitude_thrust_control.down(pos_sp_z, pos_ned, vel_body, attitude, dt_ms);
 
    roll_deg = calcRoll();
    roll_deg = limit_values( roll_deg , -5.0f,5.0f);
	thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) );
    return;
}

void VisionRollThrustControl::collimation(  float& yaw_deg, float& roll_deg, float& thrust, Target target1, Target target2, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms )
{
    time_change = dt_ms / 1000.0f;
    //float roll_rad = deg2rad( attitude.roll_deg );

    y_rad = deg2rad(target1.y_deg);
    z_rad = deg2rad(target1.z_deg);

    y_f_rad = deg2rad(target2.y_deg);
    z_f_rad = deg2rad(target2.z_deg);

    Ky = 0.5f; 
    Kz = 1.0f;
    err_pos_y = -Ky * (y_f_rad - y_rad);//y - collimation
    err_pos_z = Kz * z_rad;//z - no collimation
    //err_pos_z = -Kz * (z_f_rad - z_rad);//z - collimation
    vel_err = -vel_body.y_m_s;

    pos_sp_z = pos_ned.down_m + err_pos_z;
    alt_thrust = altitude_thrust_control.down(pos_sp_z, pos_ned, vel_body, attitude, dt_ms);
    yaw_deg = attitude.yaw_deg + rad2deg(y_rad);
    roll_deg = calcRoll();
    roll_deg = limit_values( roll_deg , -5.0f,5.0f);
	thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) );
    return;
}

void VisionRollThrustControl::hold(float& roll_deg, float& thrust, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms)
{
    time_change = dt_ms / 1000.0f;
    // if(attitude.roll_deg > 0 && vel_body.y_m_s < 0) vel_body.y_m_s = -vel_body.y_m_s;
    // else if(attitude.roll_deg < 0 && vel_body.y_m_s > 0) vel_body.y_m_s = -vel_body.y_m_s;
    // if(attitude.pitch_deg > 0 && vel_body.x_m_s > 0) vel_body.x_m_s = -vel_body.y_m_s;
    // if(attitude.pitch_deg < 0 && vel_body.x_m_s < 0) vel_body.y_m_s = -vel_body.y_m_s;
    if( target_x_m > 4.0f && ! K_lock)
    {
        Ky = 0.4f;
        // target_x_m = target_x_m - vel_body.x_m_s * time_change;
        // target_y_m = target_y_m - vel_body.y_m_s * time_change;
        // target_z_m = target_z_m - vel_body.z_m_s * time_change;
        // y_rad = atan2f(target_y_m, target_x_m);
    }
    else{
        Ky = 0.2f;
        K_lock = true;
    }
    //z_rad = atan2f(target_z_m, target_x_m);
    
    err_pos_y = Ky * y_rad;
    //err_pos_z = Kz * z_rad;
    vel_err = -vel_body.y_m_s;

    alt_thrust = altitude_thrust_control.hold(pos_ned, vel_body, attitude, dt_ms);
    
    roll_deg = calcRoll();
    thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) );
    return;
}

float VisionRollThrustControl::calcRoll()
{
    //cout << "err_pos_y: " << rad2deg(err_pos_y) << endl;
    //cout << "err_pos_z: " << rad2deg(err_pos_z) << endl;
    float thr_sp_y;
	thrust_desired_y = Kp_y * err_pos_y + Kd_y * vel_err + int_pos_y;
	float thrust_max_y_tilt = fabsf(alt_thrust) * tanf(tilt_max);//while in take_off or landing state i think the "cos(_pitch) * cos(_roll) = 1" => alt_thrust = thrust_z 
	float thrust_max_y = sqrtf(thrust_max * thrust_max - alt_thrust * alt_thrust);
	thrust_max_y = thrust_max_y_tilt < thrust_max_y ? thrust_max_y_tilt : thrust_max_y;
	// Saturate thrust in NE-direction.
    thr_sp_y = thrust_desired_y;

	if (thr_sp_y  > thrust_max_y) {
		thr_sp_y = thrust_max_y;			
	}
	// Use tracking Anti-Windup for NE-direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	float arw_gain = 2.f / Kp_y;
    float vel_err_lim = 0.0f;
	vel_err_lim = err_pos_y - (thrust_desired_y - thr_sp_y) * arw_gain;
	
    // Update integral
	int_pos_y = int_pos_y + Ki_y * vel_err_lim * time_change;
	//_thr_int(1) += _param_mpc_xy_vel_i.get() * vel_err_lim(1) * dt;

	thr_sp_y = limit_values(thr_sp_y, -0.707f * alt_thrust, 0.707f * alt_thrust);
	//cout << "_thr_sp[0] = " << _thr_sp[0] << " " << "_thr_sp[1] = " << _thr_sp[1] << " " << "_thr_sp[2] = " << _thr_sp[2] << endl;
	//kfy:float roll_sp = asinf(thr_sp_y / alt_thrust);
    float roll_sp = atan2f(thr_sp_y, alt_thrust);
    //cout << "roll_deg: " << rad2deg(roll_sp) << endl;
    return limit_values( rad2deg(roll_sp), -30.0f, 30.0f);
}

void VisionRollThrustControl::angleOffset_Yaw_z( float& yaw_deg, float& thrust, float _pos_sp_z, Target target, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms )
{
    time_change = dt_ms / 1000.0f;
    target_x_m = target.x_m;
    target_y_m = target.y_m;
    target_z_m = target.z_m;
    y_rad = deg2rad(target.y_deg);
    z_rad = deg2rad(target.z_deg);
    
    err_pos_z = Kz * z_rad;
    //vel_err = -vel_body.y_m_s;
    pos_sp_z = _pos_sp_z;
/*
    if( fabs(z_rad) > 0.02 && !Y_RAD_LOCK )
    {
        pos_sp_z = pos_ned.down_m + err_pos_z;
    }
    else{
        Y_RAD_LOCK = true;
    }*/
    alt_thrust = altitude_thrust_control.down(pos_sp_z, pos_ned, vel_body, attitude, dt_ms);

    yaw_deg = attitude.yaw_deg + rad2deg(y_rad);
	thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) );
    return;
}

void VisionRollThrustControl::angleOffset_Yaw( float& yaw_deg, float& thrust, Target target, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms )
{
    time_change = dt_ms / 1000.0f;
    target_x_m = target.x_m;
    target_y_m = target.y_m;
    target_z_m = target.z_m;
    y_rad = deg2rad(target.y_deg);
    z_rad = deg2rad(target.z_deg);
    
    err_pos_z = Kz * z_rad;
    //vel_err = -vel_body.y_m_s;

    if( fabs(z_rad) > 0.02 && !Y_RAD_LOCK )
    {
        pos_sp_z = pos_ned.down_m + err_pos_z;
    }
    else{
        Y_RAD_LOCK = true;
    }
    alt_thrust = altitude_thrust_control.down(pos_sp_z, pos_ned, vel_body, attitude, dt_ms);

    yaw_deg = attitude.yaw_deg + rad2deg(y_rad);
	thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) );
    return;
}

void VisionRollThrustControl::hold_yaw(float& thrust, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms)
{
    time_change = dt_ms / 1000.0f;
    // if(attitude.roll_deg > 0 && vel_body.y_m_s < 0) vel_body.y_m_s = -vel_body.y_m_s;
    // else if(attitude.roll_deg < 0 && vel_body.y_m_s > 0) vel_body.y_m_s = -vel_body.y_m_s;
    // if(attitude.pitch_deg > 0 && vel_body.x_m_s > 0) vel_body.x_m_s = -vel_body.y_m_s;
    // if(attitude.pitch_deg < 0 && vel_body.x_m_s < 0) vel_body.y_m_s = -vel_body.y_m_s;
    /*if( target_x_m > 4.0f && ! K_lock)
    {
        Ky = 0.4f;
        target_x_m = target_x_m - vel_body.x_m_s * time_change;
        target_y_m = target_y_m - vel_body.y_m_s * time_change;
        target_z_m = target_z_m - vel_body.z_m_s * time_change;
        y_rad = atan2f(target_y_m, target_x_m);
    }
    else{
        Ky = 0.2f;
        K_lock = true;
    }*/
    //z_rad = atan2f(target_z_m, target_x_m);
    
    //err_pos_y = Ky * y_rad;
    //err_pos_z = Kz * z_rad;
    //vel_err = -vel_body.y_m_s;

    alt_thrust = altitude_thrust_control.hold(pos_ned, vel_body, attitude, dt_ms);
    //yaw_deg = attitude.yaw_deg + rad2deg(y_rad);
    
    thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) );
    return;
}

void VisionRollThrustControl::braking(float& roll_deg, float& pitch_deg, float& thrust, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms) {
	float alt_thrust = altitude_thrust_control.hold(pos_ned, vel_body, attitude, dt_ms);
	Vector2f Vxy = { vel_body.x_m_s, vel_body.y_m_s};
	//Vector2f Vxy_sp = { min_vx_find_loop ,0.0f };
    Vector2f Vxy_sp = { -0.1f ,0.0f };
	Vector2f Vxy_err = Vxy_sp - Vxy;
	Vector2f Kp_brake = { 0.1f,0.1f };
	
	//float Ki_brank = 0.01f;
	//float thrustx = Kp_brank * Vx_err + _int_vel_x;
	Vector2f thrust_xy = Kp_brake * Vxy_err;
	//_int_vel_x += Ki_brank * dt * Vx_err;

	float thrust_max_tilt = fabsf(alt_thrust) * tanf(tilt_max);//while in take_off or landing state i think the "cos(_pitch) * cos(_roll) = 1" => alt_thrust = thrust_z 
	float thrust_max_xy = sqrtf(thrust_max * thrust_max - alt_thrust * alt_thrust);
	thrust_max_xy = thrust_max_tilt < thrust_max_xy ? thrust_max_tilt : thrust_max_xy;
	// Saturate thrust in NE-direction.
    float mag = mag2f(thrust_xy);
	if (mag * mag > thrust_max_xy * thrust_max_xy) {
		thrust_xy = thrust_xy / (mag / thrust_max_xy);
	}
    //if (Vx_err > 0 && _int_vel_x < 0) _int_vel_x
	//kfy :float pitch = -atanf(thrust_xy.x / alt_thrust);
	//float roll = atanf(thrust_xy.y / alt_thrust);
    Vector3f thrust_xyz = {thrust_xy.x, thrust_xy.y, alt_thrust};
    roll_deg = rad2deg( atan2f(thrust_xyz.y, thrust_xyz.z) );
    pitch_deg = rad2deg( atan2f(-thrust_xyz.x, sqrtf(thrust_xyz.y*thrust_xyz.y+thrust_xyz.z*thrust_xyz.z)) );

//	pitch = limit_values(pitch, -P_I / 6.0f, P_I / 6.0f);
//	roll = limit_values(roll, -P_I / 6.0f, P_I / 6.0f);
//	roll_deg = rad2deg(roll);
//	pitch_deg = rad2deg(pitch);
    roll_deg=limit_values(roll_deg,-30.0f,30.0f);
    pitch_deg= limit_values(pitch_deg,-30.0f,30.0f);

	//thrust = alt_thrust;
    thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) );

}

void VisionRollThrustControl::update_the_first_thr_flag(VelocityBody vel_body){
    if(fabs(y_f_rad - y_rad) <= 0.05f && fabs(z_f_rad - z_rad) <= 0.05f &&  fabs(vel_body.y_m_s) <= 0.05f && fabs(vel_body.z_m_s) <= 0.05f) can_start_the_first_thr = true;
    else can_start_the_first_thr = false;
}

void VisionRollThrustControl::update_thr_ring_flag(Target target){
    if (target.y_m*target.y_m + target.z_m*target.z_m < 0.4*0.4) can_through_ring_flag = true;
	else can_through_ring_flag = false;
}

void VisionRollThrustControl::angleOffset_roll_yaw( float& roll_deg,float& yaw_deg, float& thrust, Target target, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms )
{
    time_change = dt_ms / 1000.0f;
    target_x_m = target.x_m;
    target_y_m = target.y_m;
    target_z_m = target.z_m;
    y_rad = deg2rad(target.y_deg);
    z_rad = deg2rad(target.z_deg);
    
    if( target_x_m > 5.0f && ! K_lock)
    {
        Ky = 0.4f;
    }
    else{
        Ky = 0.2f;
        K_lock = true;
    }

    err_pos_y = Ky * y_rad;
    err_pos_z = Kz * z_rad;
    if( fabs(z_rad) > 0.02 && !Y_RAD_LOCK )
    {
        pos_sp_z = pos_ned.down_m + err_pos_z;
    }
    else{
        Y_RAD_LOCK = true;
    }
    alt_thrust = altitude_thrust_control.down(pos_sp_z, pos_ned, vel_body, attitude, dt_ms);

    yaw_deg = attitude.yaw_deg + rad2deg(0.8*y_rad);
    roll_deg = calcRoll_kfy();
	thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) );
    return;
}

void VisionRollThrustControl::angleOffset_roll_yaw_z( float& roll_deg,float& yaw_deg, float& thrust, float _pos_sp_z, Target target, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms )
{
    time_change = dt_ms / 1000.0f;
    target_x_m = target.x_m;
    target_y_m = target.y_m;
    target_z_m = target.z_m;
    y_rad = deg2rad(target.y_deg);
    //z_rad = deg2rad(target.z_deg);
    
    if( target_x_m > 5.0f && ! K_lock)
    {
        Ky = 0.4f;
    }
    else{
        Ky = 0.2f;
        K_lock = true;
    }

    err_pos_y = Ky * y_rad;
    // err_pos_z = Kz * z_rad;
    // if( fabs(z_rad) > 0.02 && !Y_RAD_LOCK )
    // {
    //     pos_sp_z = pos_ned.down_m + err_pos_z;
    // }
    // else{
    //     Y_RAD_LOCK = true;
    // }
    pos_sp_z = _pos_sp_z;
    alt_thrust = altitude_thrust_control.down(pos_sp_z, pos_ned, vel_body, attitude, dt_ms);

    yaw_deg = attitude.yaw_deg + rad2deg(0.8*y_rad);
    roll_deg = calcRoll_kfy();
	thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) );
    return;
}

void VisionRollThrustControl::angleOffset_roll_yaw_vel(float vel_set, float& pitch_deg, float& roll_deg,float& yaw_deg, float& thrust, Target target, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms )
{
    time_change = dt_ms / 1000.0f;
    target_x_m = target.x_m;
    target_y_m = target.y_m;
    target_z_m = target.z_m;
    y_rad = deg2rad(target.y_deg);
    z_rad = deg2rad(target.z_deg);
    
    if( target_x_m > 5.0f && ! K_lock)
    {
        Ky = 0.4f;
    }
    else{
        Ky = 0.2f;
        K_lock = true;
    }

    err_pos_y = Ky * y_rad;
    err_pos_z = Kz * z_rad;
    vel_x_err = vel_set - vel_body.x_m_s;
    if( fabs(z_rad) > 0.02 && !Y_RAD_LOCK )
    {
        pos_sp_z = pos_ned.down_m + err_pos_z;
    }
    else{
        Y_RAD_LOCK = true;
    }
    alt_thrust = altitude_thrust_control.down(pos_sp_z, pos_ned, vel_body, attitude, dt_ms);

    Vector2f thrust_xy;
    thrust_xy.x= Kp_x_vel * vel_x_err;
    thrust_xy.y= Kp_y * err_pos_y ;
    thrust_desired_y=thrust_xy.y;
    float thrust_max_tilt = fabsf(alt_thrust) * tanf(tilt_max);//while in take_off or landing state i think the "cos(_pitch) * cos(_roll) = 1" => alt_thrust = thrust_z 
	float thrust_max_xy = sqrtf(thrust_max * thrust_max - alt_thrust * alt_thrust);
	thrust_max_xy = thrust_max_tilt < thrust_max_xy ? thrust_max_tilt : thrust_max_xy;
	// Saturate thrust in NE-direction.
    float mag = mag2f(thrust_xy);
	if (mag * mag > thrust_max_xy * thrust_max_xy) {
		thrust_xy = thrust_xy / (mag / thrust_max_xy);
	}
	// Use tracking Anti-Windup for NE-direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	float arw_gain = 2.f / Kp_y;
    float vel_err_lim = 0.0f;
	vel_err_lim = err_pos_y - (thrust_desired_y - thrust_xy.y) * arw_gain;
	
    // Update integral
	int_pos_y = int_pos_y + Ki_y * vel_err_lim * time_change;
	//_thr_int(1) += _param_mpc_xy_vel_i.get() * vel_err_lim(1) * dt;

    Vector3f thrust_xyz = {thrust_xy.x, thrust_xy.y, alt_thrust};
    roll_deg = rad2deg( atan2f(thrust_xyz.y, thrust_xyz.z) );
    pitch_deg = rad2deg( atan2f(-thrust_xyz.x, sqrtf(thrust_xyz.y*thrust_xyz.y+thrust_xyz.z*thrust_xyz.z)) );

    roll_deg=limit_values(roll_deg,-30.0f,30.0f);
    pitch_deg= limit_values(pitch_deg,-30.0f,30.0f);
    yaw_deg = attitude.yaw_deg + rad2deg(0.8*y_rad);

	thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) );
    return;
}

void VisionRollThrustControl::angleOffset_roll_yaw_vel_z(float vel_set, float _pos_sp_z, float& pitch_deg, float& roll_deg,float& yaw_deg, float& thrust, Target target, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms )
{
    time_change = dt_ms / 1000.0f;
    target_x_m = target.x_m;
    target_y_m = target.y_m;
    target_z_m = target.z_m;
    y_rad = deg2rad(target.y_deg);
    //z_rad = deg2rad(target.z_deg);
    
    if( target_x_m > 5.0f && ! K_lock)
    {
        Ky = 0.4f;
    }
    else{
        Ky = 0.2f;
        K_lock = true;
    }

    err_pos_y = Ky * y_rad;
    //err_pos_z = Kz * z_rad;
    vel_x_err = vel_set - vel_body.x_m_s;
    // if( fabs(z_rad) > 0.02 && !Y_RAD_LOCK )
    // {
    //     pos_sp_z = pos_ned.down_m + err_pos_z;
    // }
    // else{
    //     Y_RAD_LOCK = true;
    // }
    pos_sp_z = _pos_sp_z;
    alt_thrust = altitude_thrust_control.down(pos_sp_z, pos_ned, vel_body, attitude, dt_ms);

    Vector2f thrust_xy;
    thrust_xy.x= Kp_x_vel * vel_x_err+ Ki_x_vel*vel_x_int;
    thrust_xy.y= Kp_y * err_pos_y ;
    thrust_desired_y=thrust_xy.y;
    float thrust_max_tilt = fabsf(alt_thrust) * tanf(tilt_max);//while in take_off or landing state i think the "cos(_pitch) * cos(_roll) = 1" => alt_thrust = thrust_z 
	float thrust_max_xy = sqrtf(thrust_max * thrust_max - alt_thrust * alt_thrust);
	thrust_max_xy = thrust_max_tilt < thrust_max_xy ? thrust_max_tilt : thrust_max_xy;
	// Saturate thrust in NE-direction.
    float mag = mag2f(thrust_xy);
	if (mag * mag > thrust_max_xy * thrust_max_xy) {
		thrust_xy = thrust_xy / (mag / thrust_max_xy);
	}
	// Use tracking Anti-Windup for NE-direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	float arw_gain = 2.f / Kp_y;
    float vel_err_lim = 0.0f;
	vel_err_lim = err_pos_y - (thrust_desired_y - thrust_xy.y) * arw_gain;
	
    // Update integral
	int_pos_y = int_pos_y + Ki_y * vel_err_lim * time_change;
	//_thr_int(1) += _param_mpc_xy_vel_i.get() * vel_err_lim(1) * dt;
    vel_x_int += vel_x_err;
    if(vel_x_int>100 ){
        vel_x_int=100;
    }else if(vel_x_int< -100){
        vel_x_int=-100;
    }

    Vector3f thrust_xyz = {thrust_xy.x, thrust_xy.y, alt_thrust};
    roll_deg = rad2deg( atan2f(thrust_xyz.y, thrust_xyz.z) );
    pitch_deg = rad2deg( atan2f(-thrust_xyz.x, sqrtf(thrust_xyz.y*thrust_xyz.y+thrust_xyz.z*thrust_xyz.z)) );

    roll_deg=limit_values(roll_deg,-30.0f,30.0f);
    pitch_deg= limit_values(pitch_deg,-30.0f,30.0f);
    yaw_deg = attitude.yaw_deg + rad2deg(0.8*y_rad);

	thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) );
    return;
}
void VisionRollThrustControl::hold_roll_yaw( float& thrust, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms)
{
    time_change = dt_ms / 1000.0f;
    
    alt_thrust = altitude_thrust_control.hold(pos_ned, vel_body, attitude, dt_ms);
 	thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) );
    return;
}

float VisionRollThrustControl::calcRoll_kfy()
{
    //cout << "err_pos_y: " << rad2deg(err_pos_y) << endl;
    //cout << "err_pos_z: " << rad2deg(err_pos_z) << endl;
    float thr_sp_y;
	thrust_desired_y = Kp_y * err_pos_y;
    //thrust_desired_y = Kp_y * err_pos_y + int_pos_y;
    
	float thrust_max_y_tilt = fabsf(alt_thrust) * tanf(tilt_max);//while in take_off or landing state i think the "cos(_pitch) * cos(_roll) = 1" => alt_thrust = thrust_z 
	float thrust_max_y = sqrtf(thrust_max * thrust_max - alt_thrust * alt_thrust);
	thrust_max_y = thrust_max_y_tilt < thrust_max_y ? thrust_max_y_tilt : thrust_max_y;
	// Saturate thrust in NE-direction.
    thr_sp_y = thrust_desired_y;

	if (thr_sp_y  > thrust_max_y) {
		thr_sp_y = thrust_max_y;			
	}
	// Use tracking Anti-Windup for NE-direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	float arw_gain = 2.f / Kp_y;
    float vel_err_lim = 0.0f;
	vel_err_lim = err_pos_y - (thrust_desired_y - thr_sp_y) * arw_gain;
    // Update integral
	int_pos_y = int_pos_y + Ki_y * vel_err_lim * time_change;
	//_thr_int(1) += _param_mpc_xy_vel_i.get() * vel_err_lim(1) * dt;

	thr_sp_y = limit_values(thr_sp_y, -0.707f * alt_thrust, 0.707f * alt_thrust);
	//cout << "_thr_sp[0] = " << _thr_sp[0] << " " << "_thr_sp[1] = " << _thr_sp[1] << " " << "_thr_sp[2] = " << _thr_sp[2] << endl;
	//kfy:float roll_sp = asinf(thr_sp_y / alt_thrust);
    float roll_sp = atan2f(thr_sp_y, alt_thrust);
    //cout << "roll_deg: " << rad2deg(roll_sp) << endl;
    return limit_values( rad2deg(roll_sp), -30.0f, 30.0f);
}

float VisionRollThrustControl::set_pos_sp_z( float _pos_sp_z, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms)
{
    pos_sp_z = _pos_sp_z;
    altitude_thrust_control.down(pos_sp_z, pos_ned, vel_body, attitude, dt_ms);
}

void AltitudeThrustControl::reset(FileNode altitude_pid)
{
    altitude_pid["Kp_z"] >> Kp_z;
    altitude_pid["Ki_z"] >> Ki_z;
    altitude_pid["Kd_z"] >> Kd_z;
    int_pos_z = 0;
    stop_integral_D = false;

    cout << "Altitude PID:" << endl;
    cout << this->Kp_z << endl;
    cout << this->Ki_z << endl;
    cout << this->Kd_z << endl;
    return;
}

void AltitudeThrustControl::set_pos_sp_z(float _pos_sp_z){
    this->pos_sp_z = _pos_sp_z;
}

void AltitudeThrustControl::takeoff(float alttitude_set,float& roll_deg, float& pitch_deg, float& thrust, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms){
    float alt_thrust = down(alttitude_set,pos_ned, vel_body, attitude, dt_ms);
	Vector2f Vxy = { vel_body.x_m_s, vel_body.y_m_s};
	Vector2f Vxy_sp = { 0.0f ,0.0f };
	Vector2f Vxy_err = Vxy_sp - Vxy;
	Vector2f Kp_vel = { 0.1f,0.1f };
	
	//float Ki_brank = 0.01f;
	//float thrustx = Kp_brank * Vx_err + _int_vel_x;
	Vector2f thrust_xy = Kp_vel * Vxy_err;
	//_int_vel_x += Ki_brank * dt * Vx_err;

	float thrust_max_tilt = fabsf(alt_thrust) * tanf(tilt_max);//while in take_off or landing state i think the "cos(_pitch) * cos(_roll) = 1" => alt_thrust = thrust_z 
	float thrust_max_xy = sqrtf(thrust_max * thrust_max - alt_thrust * alt_thrust);
	thrust_max_xy = thrust_max_tilt < thrust_max_xy ? thrust_max_tilt : thrust_max_xy;
	// Saturate thrust in NE-direction.
    float mag = mag2f(thrust_xy);
	if (mag * mag > thrust_max_xy * thrust_max_xy) {
		thrust_xy = thrust_xy / (mag / thrust_max_xy);
	}
	//if (Vx_err > 0 && _int_vel_x < 0) _int_vel_x
	//kfy :float pitch = -atanf(thrust_xy.x / alt_thrust);
	//float roll = atanf(thrust_xy.y / alt_thrust);
    Vector3f thrust_xyz = {thrust_xy.x, thrust_xy.y, alt_thrust};
    roll_deg = rad2deg( atan2f(thrust_xyz.y, thrust_xyz.z) );
    pitch_deg = rad2deg( atan2f(-thrust_xyz.x, sqrtf(thrust_xyz.y*thrust_xyz.y+thrust_xyz.z*thrust_xyz.z)) );

//	pitch = limit_values(pitch, -P_I / 6.0f, P_I / 6.0f);
//	roll = limit_values(roll, -P_I / 6.0f, P_I / 6.0f);
//	roll_deg = rad2deg(roll);
//	pitch_deg = rad2deg(pitch);
    roll_deg=limit_values(roll_deg,-30.0f,30.0f);
    pitch_deg= limit_values(pitch_deg,-30.0f,30.0f);
//  thrust = alt_thrust;
	thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) );
}
void AltitudeThrustControl::takeoff_vx(float vel_x,float alttitude_set,float& roll_deg, float& pitch_deg, float& thrust, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms){
    float alt_thrust = down(alttitude_set,pos_ned, vel_body, attitude, dt_ms);
	Vector2f Vxy = { vel_body.x_m_s, vel_body.y_m_s};
	Vector2f Vxy_sp = { vel_x ,0.0f };
	Vector2f Vxy_err = Vxy_sp - Vxy;
	Vector2f Kp_vel = { 0.1f,0.1f };
	
	//float Ki_brank = 0.01f;
	//float thrustx = Kp_brank * Vx_err + _int_vel_x;
	Vector2f thrust_xy = Kp_vel * Vxy_err;
	//_int_vel_x += Ki_brank * dt * Vx_err;

	float thrust_max_tilt = fabsf(alt_thrust) * tanf(tilt_max);//while in take_off or landing state i think the "cos(_pitch) * cos(_roll) = 1" => alt_thrust = thrust_z 
	float thrust_max_xy = sqrtf(thrust_max * thrust_max - alt_thrust * alt_thrust);
	thrust_max_xy = thrust_max_tilt < thrust_max_xy ? thrust_max_tilt : thrust_max_xy;
	// Saturate thrust in NE-direction.
    float mag = mag2f(thrust_xy);
	if (mag * mag > thrust_max_xy * thrust_max_xy) {
		thrust_xy = thrust_xy / (mag / thrust_max_xy);
	}
	//if (Vx_err > 0 && _int_vel_x < 0) _int_vel_x
	//kfy :float pitch = -atanf(thrust_xy.x / alt_thrust);
	//float roll = atanf(thrust_xy.y / alt_thrust);
    Vector3f thrust_xyz = {thrust_xy.x, thrust_xy.y, alt_thrust};
    roll_deg = rad2deg( atan2f(thrust_xyz.y, thrust_xyz.z) );
    pitch_deg = rad2deg( atan2f(-thrust_xyz.x, sqrtf(thrust_xyz.y*thrust_xyz.y+thrust_xyz.z*thrust_xyz.z)) );

//	pitch = limit_values(pitch, -P_I / 6.0f, P_I / 6.0f);
//	roll = limit_values(roll, -P_I / 6.0f, P_I / 6.0f);
//	roll_deg = rad2deg(roll);
//	pitch_deg = rad2deg(pitch);
    roll_deg=limit_values(roll_deg,-30.0f,30.0f);
    pitch_deg= limit_values(pitch_deg,-30.0f,30.0f);
//  thrust = alt_thrust;
	thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) );
}


float AltitudeThrustControl::landing()
{
    float thrust;
	if (times < 2000 / 20){// first 2 second
        thrust = mid_thrust - 0.05f;
	}
	else if (times < 4000 / 20) {
        thrust = mid_thrust - 0.05f;
	}
	else {
        thrust = mid_thrust - 0.05f;
	}
	times++;
    return thrust;
}

void AltitudeThrustControl::braking(float& roll_deg, float& pitch_deg, float& thrust, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms) {
	float alt_thrust = hold(pos_ned, vel_body, attitude, dt_ms);
	Vector2f Vxy = { vel_body.x_m_s, vel_body.y_m_s};
	Vector2f Vxy_sp = { min_vx_find_loop ,0.0f };
	Vector2f Vxy_err = Vxy_sp - Vxy;
	Vector2f Kp_brake = { 0.1f,0.1f };
	
	//float Ki_brank = 0.01f;
	//float thrustx = Kp_brank * Vx_err + _int_vel_x;
	Vector2f thrust_xy = Kp_brake * Vxy_err;
	//_int_vel_x += Ki_brank * dt * Vx_err;

	float thrust_max_tilt = fabsf(alt_thrust) * tanf(tilt_max);//while in take_off or landing state i think the "cos(_pitch) * cos(_roll) = 1" => alt_thrust = thrust_z 
	float thrust_max_xy = sqrtf(thrust_max * thrust_max - alt_thrust * alt_thrust);
	thrust_max_xy = thrust_max_tilt < thrust_max_xy ? thrust_max_tilt : thrust_max_xy;
	// Saturate thrust in NE-direction.
    float mag = mag2f(thrust_xy);
	if (mag * mag > thrust_max_xy * thrust_max_xy) {
		thrust_xy = thrust_xy / (mag / thrust_max_xy);
	}

    Vector3f thrust_xyz = {thrust_xy.x, thrust_xy.y, alt_thrust};
    roll_deg = rad2deg( atan2f(thrust_xyz.y, thrust_xyz.z) );
    pitch_deg = rad2deg( atan2f(-thrust_xyz.x, sqrtf(thrust_xyz.y*thrust_xyz.y+thrust_xyz.z*thrust_xyz.z)) );
    roll_deg=limit_values(roll_deg,-30.0f,30.0f);
    pitch_deg= limit_values(pitch_deg,-30.0f,30.0f);
	//thrust = alt_thrust;
    thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) );

}

float AltitudeThrustControl::downOffset( float err, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms )
{
    time_change = dt_ms / 1000.0f;
    pos_z = pos_ned.down_m;
    vel_z = vel_body.z_m_s;
    roll = attitude.roll_deg;
    pitch = attitude.pitch_deg;
    pos_sp_z = pos_z + err;
    return calcThrust();
}

float AltitudeThrustControl::down(float pos_sp_z, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms)
{
    time_change = dt_ms / 1000.0f;
    pos_z = pos_ned.down_m;
    vel_z = vel_body.z_m_s;
    roll = attitude.roll_deg;
    pitch = attitude.pitch_deg;
    this->pos_sp_z = pos_sp_z;
    return calcThrust();
}

float AltitudeThrustControl::hold(PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms)
{
    time_change = dt_ms / 1000.0f;
    pos_z = pos_ned.down_m;
    vel_z = vel_body.z_m_s;
    roll = attitude.roll_deg;
    pitch = attitude.pitch_deg;
    return calcThrust();
}

float AltitudeThrustControl::calcThrust()
{
    float thrust;
    update<float>(down_reference_topic, pos_sp_z, down_reference_mtx);
    err_pos_z = pos_sp_z - pos_z;
    //offset_thrust = mid_thrust / (cos(deg2rad(pitch))*cos(deg2rad(roll)));
    offset_thrust = mid_thrust;
    thrust_desired_D = Kp_z * err_pos_z + Kd_z * (-vel_z) + int_pos_z - offset_thrust;
    stop_integral_D = (thrust_desired_D >= uMax && err_pos_z >= 0.0f) ||
			(thrust_desired_D <= uMin && err_pos_z <= 0.0f);
    if( ! stop_integral_D )
    {
        int_pos_z += err_pos_z * Ki_z * time_change;
        sign_thr_int_z = int_pos_z > 0 ? 1 : -1;
		int_pos_z = std::min(fabsf(int_pos_z), 1.0f) * sign_thr_int_z;
    }
    thrust = limit_values(-thrust_desired_D, uMin, uMax);
    return thrust;
}

void healthCheck( shared_ptr<Telemetry> telemetry )
{
    while ( ! telemetry->health_all_ok() ) {
        cout << "[LOGGING]: Waiting for health_all_ok" << endl;
        this_thread::sleep_for(seconds(1));
    }
}

void waitForArmed( shared_ptr<Telemetry> telemetry )
{
    while( ! telemetry->armed() )
    {
        this_thread::sleep_for(seconds(1));
    }
    cout << "[LOGGING]: armed" << endl;
    return;
}

void quitOffboard( shared_ptr<Offboard> offboard )
{
    Offboard::Result offboard_result;
    if( offboard->is_active() ){
        offboard_result = offboard->stop();
        if ( offboard_result != Offboard::Result::SUCCESS ){
            cout << string("[ERROR]: ") + Offboard::result_str(offboard_result) << endl;
            cout << "[ERROR]: stop offboard error" << endl;
            return;
        }
    }
    return;
}

void offbCtrlAttitude(shared_ptr<Offboard> offboard, Offboard::Attitude attitude)
{
    InputAttitude input_attitude;
    Offboard::Result offboard_result;
    if( ! offboard->is_active() ){
        offboard->set_attitude(attitude);
        offboard_result = offboard->start();
        if ( offboard_result != Offboard::Result::SUCCESS ){
            cout << string("[ERROR]: ") + Offboard::result_str(offboard_result) << endl;
            cout << "[ERROR]: unable to start offboard" << endl;
            return;
        }
    }
    // Send it once before starting offboard, otherwise it will be rejected.
    offboard->set_attitude(attitude);
    input_attitude.roll_deg = attitude.roll_deg;
    input_attitude.pitch_deg = attitude.pitch_deg;
    input_attitude.yaw_deg = attitude.yaw_deg;
    input_attitude.thrust = attitude.thrust_value;
    update<InputAttitude>(input_attitude_topic, input_attitude, input_attitude_mtx);
    return;
}