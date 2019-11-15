#include "control_node.hpp"
#include "const.hpp"
#include <vector>
using namespace std;
int flag_loop = 1;
#if (TEST_LEVEL == LEVEL_FINAL0) || (TEST_LEVEL == LEVEL_FINAL1) || (TEST_LEVEL == LEVEL_FINAL2) || (TEST_LEVEL == LEVEL_FINAL3)
void missionLoop(shared_ptr<Telemetry> telemetry, shared_ptr<Offboard> offboard, FileNode altitude_pid, FileNode vision_pid, FileNode flow_pid, FileNode mission_param)
{
	int16_t missions_status = WAIT_COMMAND;
	int timeout_count = 0;
	int time_count = 0;
	float altitude_offset = 0;
	float altitude_set = 0;
	float vel_x_set = 0;
	Target target[3];
	bool flag_climb_init = false,
		 flag_flow_init = false,
#if TEST_LEVEL == LEVEL_FINAL0 || TEST_LEVEL == LEVEL_FINAL2
		 flag_search_target12_init = false,
		 flag_through_target12_init = false,
#endif
#if TEST_LEVEL == LEVEL_FINAL1 || TEST_LEVEL == LEVEL_FINAL3
		 flag_search_target1_init = false,
		 flag_search_target2_init = false,
		 flag_through_target1_init = false,
		 flag_through_target2_init = false,
#endif
#define TARGET1 0
#define TARGET2 1
#define TARGET3 2
		 flag_search_target3_init = false,
		 flag_through_target3_init = false,
		 flag_adjust_init = false,
		 flag_land_init = false;
	unsigned int current_target;
	int64_t command_timestamp, target_timestamp;
	double param;
	MissionCommand command;
	Telemetry::PositionVelocityNED position_velocity_ned;
	Telemetry::EulerAngle euler_angle;
	PositionNED position_ned;
	Vector2f position_checkpoint;
	VelocityNED velocity_ned;
	VelocityBody velocity_body;
	EulerAngle attitude;
	EulerAngle attitude_offset;
	EulerAngle attitude_buf[4];

	AltitudeThrustControl altitude_thrust_control;
	VisionRollThrustControl vision_roll_thrust_control;
	FlowPosThrustControl flow_pos_thrust_control;
	altitude_thrust_control.reset(altitude_pid);
	vision_roll_thrust_control.reset(vision_pid, altitude_pid);
	flow_pos_thrust_control.reset(flow_pid, altitude_pid);

	Offboard::Attitude input_attitude;
	float roll_deg, pitch_deg, yaw_deg, thrust;
	float pos_sp_z;
	float open_loop_distance;
	int period_ms;
	high_resolution_clock::time_point open_loop_t0;
	int64_t last_peroid = timestampf();
	int fail_cnt = 0;
	Vector3f offset_body;

	DetectionResult result;
	bool YAW_MODE_LOCK = false;
	bool stop_search_second_loop = false,
		 through12_lock = false;
	bool through1_lock = false;
	float roll_rad;

	float VEL_TARGET_1,
		TIME_START_SEARCH_TARGET_2,
    	TIME_THROUGH_TARGET_1,
		VEL_TARGET_2_FAR,
    	VEL_TARGET_2_NEAR,
    	TIME_THROUGH_TARGET_2;
	
	mission_param["VEL_TARGET_1"] >> VEL_TARGET_1;
	mission_param["TIME_START_SEARCH_TARGET_2"] >> TIME_START_SEARCH_TARGET_2;
	mission_param["TIME_THROUGH_TARGET_1"] >> TIME_THROUGH_TARGET_1;
	mission_param["VEL_TARGET_2_FAR"] >> VEL_TARGET_2_FAR;
	mission_param["VEL_TARGET_2_NEAR"] >> VEL_TARGET_2_NEAR;
	mission_param["TIME_THROUGH_TARGET_2"] >> TIME_THROUGH_TARGET_2;

	cout << "VEL_TARGET_1: " << VEL_TARGET_1;
	cout << "TIME_START_SEARCH_TARGET_2: " << TIME_START_SEARCH_TARGET_2;
	cout << "TIME_THROUGH_TARGET_1: " << TIME_THROUGH_TARGET_1;
	cout << "VEL_TARGET_2_FAR: " << VEL_TARGET_2_FAR;
	cout << "VEL_TARGET_2_NEAR: " << VEL_TARGET_2_NEAR;
	cout << "TIME_THROUGH_TARGET_2: " << TIME_THROUGH_TARGET_2;
	//****************************************************************************************************
	//****************************************************************************************************
	while (true)
	{
		//update everything every loop
		//update telem
		position_velocity_ned = telemetry->position_velocity_ned();
		euler_angle = telemetry->attitude_euler_angle();
		memcpy(&position_ned, &position_velocity_ned.position, sizeof position_ned);
		memcpy(&velocity_ned, &position_velocity_ned.velocity, sizeof velocity_ned);
		memcpy(&attitude, &euler_angle, sizeof attitude);
		velocity_body = velocityNED2Body(velocity_ned, attitude);
		//exception
		if (-position_ned.down_m > ALTITUDE_UPPER_BOUND)
		{
			if (missions_status != FORCE_QUIT_COMMAND)
				missions_status = SAFE_QUIT_COMMAND;
		}
		//update ground control
		if (latest<MissionCommand>(mission_command_topic, command_timestamp, command, mission_command_mtx))
		{
			clear<MissionCommand>(mission_command_topic, mission_command_mtx);
			cout << "COMMAND: " << command.index << endl;
			missions_status = command.index;
			param = command.argv[0];
		}
		else
		{
			param = 0.0;
		}

		//update target
		target[TARGET1].valid = false;
		target[TARGET2].valid = false;
		target[TARGET3].valid = false;
#if TEST_LEVEL == LEVEL_FINAL2 || TEST_LEVEL == LEVEL_FINAL3
		roll_rad = deg2rad(attitude.roll_deg);
		if (latest<DetectionResult>(first_target_topic, target_timestamp, result, first_target_mtx) && timestampf() - target_timestamp < 30)
		{
			target[TARGET1].valid = true;
			target[TARGET1].distance_m = result.distance_m;
			target[TARGET1].y_m = result.x_m * cos(roll_rad) - result.y_m * sin(roll_rad);
			target[TARGET1].z_m = result.x_m * sin(roll_rad) + result.y_m * cos(roll_rad);
			target[TARGET1].x_m = result.z_m;
		}
		if (latest<DetectionResult>(second_target_topic, target_timestamp, result, first_target_mtx) && timestampf() - target_timestamp < 30)
		{
			target[TARGET2].valid = true;
			target[TARGET2].distance_m = result.distance_m;
			target[TARGET2].y_m = result.x_m * cos(roll_rad) - result.y_m * sin(roll_rad);
			target[TARGET2].z_m = result.x_m * sin(roll_rad) + result.y_m * cos(roll_rad);
			target[TARGET2].x_m = result.z_m;
		}
		if (latest<DetectionResult>(third_target_topic, target_timestamp, result, first_target_mtx) && timestampf() - target_timestamp < 30)
		{
			target[TARGET3].valid = true;
			target[TARGET3].distance_m = result.distance_m;
			target[TARGET3].y_m = result.x_m * cos(roll_rad) - result.y_m * sin(roll_rad);
			target[TARGET3].z_m = result.x_m * sin(roll_rad) + result.y_m * cos(roll_rad);
			target[TARGET3].x_m = result.z_m;
		}
#endif
#if TEST_LEVEL == LEVEL_FINAL0 || TEST_LEVEL == LEVEL_FINAL1
		update<int>(camera_status_topic, 2, camera_status_mtx);
		bool found = latest<DetectionResult>(second_target_topic, target_timestamp, result, second_target_mtx);
		if (found)
		{
			int64_t interval = timestampf() - target_timestamp;
			//cout << "found:" << found << endl;
			//cout << "interval:" << interval << endl;
			if (interval < 30)
			{
				//cout << "VALID" << endl;
				target[TARGET2].valid = true;
				target[TARGET2].distance_m = result.distance_m;

				float roll_rad = deg2rad(attitude.roll_deg);
				target[TARGET2].y_m = result.x_m * cos(roll_rad) - result.y_m * sin(roll_rad);
				target[TARGET2].z_m = result.x_m * sin(roll_rad) + result.y_m * cos(roll_rad);
				target[TARGET2].x_m = result.z_m;

				target[TARGET2].y_deg = rad2deg(atan2f(target[TARGET2].y_m, target[TARGET2].x_m));
				target[TARGET2].z_deg = rad2deg(atan2f(target[TARGET2].z_m, target[TARGET2].x_m));
			}
		}
#endif
		update<int16_t>(control_status_topic, missions_status, control_status_mtx);
		switch (missions_status)
		{
		case WAIT_COMMAND:
			attitude_offset = (attitude + attitude_buf[0] + attitude_buf[1] + attitude_buf[2] + attitude_buf[3]) / 5;
			for (int i = 3; i > 0; i--)
				attitude_buf[i] = attitude_buf[i - 1];
			attitude_buf[0] = attitude;
			pitch_deg = attitude.pitch_deg;
			roll_deg = attitude.roll_deg;
			yaw_deg = attitude.yaw_deg;
			pos_sp_z = position_ned.down_m;
			//do nothing
			break;
		case SAFE_QUIT_COMMAND:
			//remotePrint("[WARNIGN]: LANDING!");
			thrust = altitude_thrust_control.landing();
			input_attitude = {0.0f, 0.0f, attitude.yaw_deg, thrust};
			offbCtrlAttitude(offboard, input_attitude);
			missions_status = LAND_MISSION;
			break;
		case FORCE_QUIT_COMMAND:
			remotePrint("[WARNIGN]: QUIT OFFBOARD!");
			quitOffboard(offboard);
			return;
		case INIT_MISSION:

			timeout_count = 0;
			flag_climb_init = false;

#if TEST_LEVEL == LEVEL_FINAL0 || TEST_LEVEL == LEVEL_FINAL2
			flag_search_target12_init = false;
			flag_through_target12_init = false;
			current_target = TARGET2;
			update<int>(camera_status_topic, 4, camera_status_mtx);
#endif
#if TEST_LEVEL == LEVEL_FINAL1 || TEST_LEVEL == LEVEL_FINAL3
			flag_search_target1_init = false;
			flag_search_target2_init = false;
			flag_through_target1_init = false;
			flag_through_target2_init = false;
			//current_target = TARGET2;
			//update<int>(camera_status_topic, 1, camera_status_mtx);
#endif
			flag_flow_init = false;
			flag_search_target3_init = false;
			flag_through_target3_init = false;
			flag_adjust_init = false;
			flag_land_init = false;
			altitude_offset = -position_ned.down_m;
			missions_status = TAKEOFF_MISSION;
			//sleep, waiting for system init
			break;
		case TAKEOFF_MISSION:
			cout << "TAKEOFF_MISSION" << endl;
			if (altitude_set < 1.1f)
				altitude_set += 0.5 / CONTROL_FREQUENCY;
			altitude_thrust_control.takeoff(-altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
			pitch_deg += 2;
			input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
			//cout << thrust << endl;
			offbCtrlAttitude(offboard, input_attitude);
			if (-position_ned.down_m > 1.0f)
			{
				altitude_set = 1.0;
				missions_status = SETPOINT_CLIMB_MISSION;
				break;
			}
			break;
		case SETPOINT_CLIMB_MISSION:
			cout << "SETPOINT_CLIMB_MISSION" << endl;
			if (!flag_climb_init)
			{
				altitude_set = 1.0;
				flag_climb_init = true;
			}
			if ((position_ned.east_m < 1e-4) && (position_ned.north_m < 1e-4))
			{
				cout << "FLOw CAN NOT THRUST" << endl;
				altitude_thrust_control.takeoff(-altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
				pitch_deg += 1.0;
				input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
				//cout << thrust << endl;
				offbCtrlAttitude(offboard, input_attitude);
				flag_flow_init = false;
			}
			else
			{
				if (!flag_flow_init)
				{
					altitude_set = -position_ned.down_m;
					offset_body = {-0.2f, 0.0f, 0.0f};
					flow_pos_thrust_control.positionBodyOffset(roll_deg, pitch_deg, thrust, offset_body, position_ned, velocity_body, attitude, period_ms);
					input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
					offbCtrlAttitude(offboard, input_attitude);
					remotePrint("Flow init!");
					flag_flow_init = true;
				}
				else
				{
					flow_pos_thrust_control.climb(-altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
					input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
					offbCtrlAttitude(offboard, input_attitude);
				}
			}
#if (TEST_LEVEL == LEVEL_FINAL0)
			if (-position_ned.down_m < 1.5 && altitude_set < 1.6)
				altitude_set += 0.8 / CONTROL_FREQUENCY;
			if (-position_ned.down_m > 1.5 && altitude_set > 1.5)
			{
				altitude_set = 1.5;
				timeout_count++;
			}
			//满足找环条件，提前找到环
			if (position_ned.down_m > 1.25 && target[TARGET1].valid == true && target[TARGET2].valid == true)
			{
				timeout_count = 0;
				missions_status = AJUSTPOSITION_MISSION;
				break;
			}
			//定位等待,超时且有一个目标没找到则进入搜索模式
			if (timeout_count >= 2 * CONTROL_FREQUENCY)
			{
				timeout_count = 0;
				missions_status = SEARCH_TARGET12_MISSION;
				break;
			}
#endif
#if (TEST_LEVEL == LEVEL_FINAL1)
			if (-position_ned.down_m < 1.5 && altitude_set < 1.6)
				altitude_set += 0.8 / CONTROL_FREQUENCY;
			if (-position_ned.down_m > 1.5 && altitude_set > 1.5)
			{
				altitude_set = 1.5;
				timeout_count++;
			}
			//满足找环条件，提前找到环
			if (-position_ned.down_m > 1.25 && target[TARGET2].valid == true)
			{
				timeout_count = 0;
				missions_status = THROUGH_TARGET2_MISSION;
				vision_roll_thrust_control.set_pos_sp_z(position_ned.down_m, position_ned, velocity_body, attitude, 20);
				break;
			}
			//定位等待,超时且有一个目标没找到则进入搜索模式
			if (timeout_count >= 2 * CONTROL_FREQUENCY)
			{
				timeout_count = 0;
				missions_status = LAND_MISSION;
				break;
			}
#endif
#if (TEST_LEVEL == LEVEL_FINAL2)
			if (-position_ned.down_m < 4.0 && altitude_set < 4.1)
				altitude_set += 0.8 / CONTROL_FREQUENCY;
			if (-position_ned.down_m > 4.0 && altitude_set > 4.0)
			{
				altitude_set = 4.0;
				timeout_count++;
			}
			//满足找环条件，提前找到环
			if (-position_ned.down_m > 3.8 && target[TARGET1].valid == true && target[TARGET2].valid == true)
			{
				timeout_count = 0;
				missions_status = AJUSTPOSITION_MISSION;
				vision_roll_thrust_control.set_pos_sp_z(position_ned.down_m, position_ned, velocity_body, attitude, 20);
				break;
			}
			//定位等待,超时且有一个目标没找到则进入搜索模式
			if (timeout_count >= 2 * CONTROL_FREQUENCY)
			{
				timeout_count = 0;
				missions_status = SEARCH_TARGET12_MISSION;
				break;
			}
#endif
#if (TEST_LEVEL == LEVEL_FINAL3)
			if (-position_ned.down_m < 4.0 && altitude_set < 4.1)
				altitude_set += 1.0f / CONTROL_FREQUENCY;
			if (-position_ned.down_m > 3.5 && altitude_set > 3.5)
			{
				altitude_set = 4.0;
				timeout_count++;
			}
			if (-position_ned.down_m > 3.8 && -position_ned.down_m < 4.2 && timeout_count > 1.5*CONTROL_FREQUENCY)
			{
				timeout_count = 0;
				missions_status = THROUGH_TARGET1_MISSION;
				vision_roll_thrust_control.set_pos_sp_z(position_ned.down_m, position_ned, velocity_body, attitude, 20);
				break;
			}
			//满足找环条件，提前找到环
			// if (-position_ned.down_m > 3.8 && target[TARGET1].valid == true)
			// {
			// 	timeout_count = 0;
			// 		missions_status = THROUGH_TARGET1_MISSION;

			// 	vision_roll_thrust_control.set_pos_sp_z(position_ned.down_m, position_ned, velocity_body, attitude, 20);
			// 	break;
			// }
			//定位等待,超时且有一个目标没找到则进入搜索模式
			// if (timeout_count >= 2 * CONTROL_FREQUENCY)
			// {
			// 	timeout_count = 0;

			// 	missions_status = SEARCH_TARGET1_MISSION;

			// 	break;
			// }
#endif
			break;

#if TEST_LEVEL == LEVEL_FINAL0 || TEST_LEVEL == LEVEL_FINAL2
		case SEARCH_TARGET12_MISSION:
			cout << "SEARCH TARGET12" << endl;
			if (!flag_search_target12_init)
			{
				altitude_set = TARGET1_HEIGHT;
				flag_search_target12_init = true;
			}
			if (timeout_count < 2 * CONTROL_FREQUENCY)
			{
				if ((position_ned.east_m < 1e-4) && (position_ned.north_m < 1e-4))
				{
					//没办法，加速进度
					timeout_count++;
					cout << "FLOw CAN NOT THRUST" << endl;
					altitude_thrust_control.takeoff(-altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
					pitch_deg += -5;
					input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
					//cout << thrust << endl;
					offbCtrlAttitude(offboard, input_attitude);
					flag_flow_init = false;
				}
				else
				{
					if (!flag_flow_init)
					{
						altitude_set = -position_ned.down_m;
						offset_body = {3.0f, 0.0f, 0.0f};
						flow_pos_thrust_control.positionBodyOffset(roll_deg, pitch_deg, thrust, offset_body, position_ned, velocity_body, attitude, period_ms);
						input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
						offbCtrlAttitude(offboard, input_attitude);
						remotePrint("Flow init!");
						flag_flow_init = true;
					}
					else
					{
						flow_pos_thrust_control.hold(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
						input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
						offbCtrlAttitude(offboard, input_attitude);
					}
				}
			}
			else if (timeout_count < 4 * CONTROL_FREQUENCY)
			{
				if ((position_ned.east_m < 1e-4) && (position_ned.north_m < 1e-4))
				{
					//没办法，加速降落
					timeout_count++;
					cout << "FLOw CAN NOT THRUST" << endl;
					altitude_thrust_control.takeoff(-altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
					roll_deg += 5 / CONTROL_FREQUENCY;
					input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
					//cout << thrust << endl;
					offbCtrlAttitude(offboard, input_attitude);
					flag_flow_init = false;
				}
				else
				{
					if (!flag_flow_init)
					{
						altitude_set = -position_ned.down_m;
						offset_body = {0.0f, 1.0f, 0.0f};
						flow_pos_thrust_control.positionBodyOffset(roll_deg, pitch_deg, thrust, offset_body, position_ned, velocity_body, attitude, period_ms);
						input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
						offbCtrlAttitude(offboard, input_attitude);
						remotePrint("Flow init!");
						flag_flow_init = true;
					}
					else
					{
						flow_pos_thrust_control.hold(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
						input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
						offbCtrlAttitude(offboard, input_attitude);
					}
				}
			}
			else if (timeout_count < 8 * CONTROL_FREQUENCY)
			{
				if ((position_ned.east_m < 1e-4) && (position_ned.north_m < 1e-4))
				{
					//没办法，加速降落
					timeout_count++;
					cout << "FLOw CAN NOT THRUST" << endl;
					altitude_thrust_control.takeoff(-altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
					roll_deg -= 5 / CONTROL_FREQUENCY;
					input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
					//cout << thrust << endl;
					offbCtrlAttitude(offboard, input_attitude);
					flag_flow_init = false;
				}
				else
				{
					if (!flag_flow_init)
					{
						altitude_set = -position_ned.down_m;
						offset_body = {0.0f, -2.0f, 0.0f};
						flow_pos_thrust_control.positionBodyOffset(roll_deg, pitch_deg, thrust, offset_body, position_ned, velocity_body, attitude, period_ms);
						input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
						offbCtrlAttitude(offboard, input_attitude);
						remotePrint("Flow init!");
						flag_flow_init = true;
					}
					else
					{
						flow_pos_thrust_control.hold(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
						input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
						offbCtrlAttitude(offboard, input_attitude);
					}
				}
			}
			else //超时且未找到，进入降落模式
			{
				cout << "TIME OUT" << endl;
				timeout_count = 0;
				missions_status = LAND_MISSION;
				break;
			}
			cout << target[TARGET2].valid << endl;
			//找到环，进入调整模式
			if (target[TARGET1].valid == true && target[TARGET2].valid == true)
			{
				timeout_count = 0;
				missions_status = AJUSTPOSITION_MISSION;
				break;
			}
			else
			{
				timeout_count++;
			}
			break;

		case AJUSTPOSITION_MISSION:

			if (!flag_adjust_init)
			{
				altitude_set = -position_ned.down_m;
				flag_adjust_init = true;
			}
			if (target[TARGET1].valid)
			{
				vision_roll_thrust_control.collimation1(yaw_deg, thrust, target[TARGET1], position_ned, velocity_body, attitude, period_ms);
				fail_cnt = 0;
				vision_roll_thrust_control.update_thr_ring_flag(target[TARGET1]);
				vision_roll_thrust_control.update_the_first_thr_flag(velocity_body);
				if (vision_roll_thrust_control.can_start_the_first_thr)
				{
					pitch_deg = -5.0f;
					stop_search_second_loop = true;
				}
				else
					pitch_deg = 0.0f;
			}
			else if (!stop_search_second_loop && target[TARGET2].valid)
			{
				//vision_roll_thrust_control.collimation(yaw_deg, roll_deg, thrust, first_target, second_target, position_ned, velocity_body, attitude, period_ms );
				vision_roll_thrust_control.collimation2(roll_deg, thrust, target[TARGET2], position_ned, velocity_body, attitude, period_ms);
			}
			else
			{
				//vision_roll_thrust_control.hold(roll_deg, thrust, position_ned, velocity_body, attitude, period_ms);
				vision_roll_thrust_control.hold_roll_yaw(thrust, position_ned, velocity_body, attitude, period_ms);
				fail_cnt++;
			}
			if (fail_cnt > VISION_FAIL_TOLERENCE) //1s?
			{
				//	vision_roll_thrust_control.can_through_ring_flag = false;
				if (vision_roll_thrust_control.can_through_ring_flag)
				{
					missions_status = THROUGH_TARGET12_MISSION;
					cout << "VISION OPEN LOOP!" << endl;
					break;
				}
				else
				{
					remotePrint("TIMEOUT!");
					missions_status = LAND_MISSION;
					break;
				}
			}
			input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
			cout << "roll_deg: " << roll_deg << endl;
			offbCtrlAttitude(offboard, input_attitude);
			break;

		case THROUGH_TARGET12_MISSION:
			cout << "THROUGH_TARGET12_MISSION" << endl;
			if (target[TARGET1].valid && !target[TARGET1].done)
			{
				{ //冲向第1个环
					timeout_count = 0;
					vision_roll_thrust_control.update_thr_ring_flag(target[TARGET1]);
					target[TARGET1].done = vision_roll_thrust_control.can_through_ring_flag && (target[TARGET1].x_m < 5.0);
					cout << "CAN THROUGH: " << vision_roll_thrust_control.can_through_ring_flag << endl;
					cout << "x_m < 5: " << (target[TARGET1].x_m < 5.0) << endl;
					if ((fabs(target[TARGET1].y_deg) > 1.0f && target[TARGET1].x_m > 7.0f) && !YAW_MODE_LOCK)
					{
						cout << "ROLL CONTROL" << endl;
						vel_x_set = 3;
						vision_roll_thrust_control.angleOffset_vel(vel_x_set, pitch_deg, roll_deg, thrust, target[TARGET1], position_ned, velocity_body, attitude, period_ms);
						input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
						offbCtrlAttitude(offboard, input_attitude);
					}
					else
					{
						cout << "ROLL YAW CONTROL" << endl;
						//近距离 视觉：速度+滚转+偏航
						YAW_MODE_LOCK = true;
						vel_x_set = 5;
						vision_roll_thrust_control.angleOffset_roll_yaw_vel(vel_x_set, pitch_deg, roll_deg, yaw_deg, thrust, target[TARGET1], position_ned, velocity_body, attitude, period_ms);
						input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
						offbCtrlAttitude(offboard, input_attitude);
					}
				}
				else
				{ //目标丢失，判断是否传环
					if (target[TARGET1].done)
					{ //穿越中
						if (!through12_lock)
						{
							if (time_count > 1.5 * CONTROL_FREQUENCY)
							{
								time_count = 0;
								timeout_count = 0;
								yaw_deg = attitude_offset.yaw_deg;
								vision_roll_thrust_control.hold_roll_yaw(thrust, position_ned, velocity_body, attitude, period_ms);
								input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
								offbCtrlAttitude(offboard, input_attitude);
								through12_lock = true;
								cout << "THROUGH1_OK " << endl;
								break;
							}
							cout << "OPEN LOOP" << endl;
							timeout_count = 0;
							vision_roll_thrust_control.hold_roll_yaw(thrust, position_ned, velocity_body, attitude, period_ms);
							input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
							offbCtrlAttitude(offboard, input_attitude);
							time_count++;
						}

						if (target[TARGET2].valid && !target[TARGET2].done)
						{ //冲向第2个环
							timeout_count = 0;
							vision_roll_thrust_control.update_thr_ring_flag(target[TARGET2]);
							target[TARGET2].done = vision_roll_thrust_control.can_through_ring_flag && (target[TARGET2].x_m < 5.0);
							cout << "CAN THROUGH: " << vision_roll_thrust_control.can_through_ring_flag << endl;
							cout << "x_m < 5: " << (target[TARGET2].x_m < 5.0) << endl;
							if ((fabs(target[TARGET2].y_deg) > 1.0f && target[TARGET2].x_m > 7.0f) && !YAW_MODE_LOCK)
							{
								cout << "ROLL CONTROL" << endl;
								vel_x_set = 3;
								vision_roll_thrust_control.angleOffset_vel(vel_x_set, pitch_deg, roll_deg, thrust, target[TARGET2], position_ned, velocity_body, attitude, period_ms);
								input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
								offbCtrlAttitude(offboard, input_attitude);
							}
							else
							{
								cout << "ROLL YAW CONTROL" << endl;
								//近距离 视觉：速度+滚转+偏航
								YAW_MODE_LOCK = true;
								vel_x_set = 5;
								vision_roll_thrust_control.angleOffset_roll_yaw_vel(vel_x_set, pitch_deg, roll_deg, yaw_deg, thrust, target[TARGET2], position_ned, velocity_body, attitude, period_ms);
								input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
								offbCtrlAttitude(offboard, input_attitude);
							}
						}
						else
						{ //目标丢失，判断是否传环
							if (target[TARGET2].done)
							{ //穿越中
								update<int>(camera_status_topic, 3, camera_status_mtx);
								if (time_count > 1.5 * CONTROL_FREQUENCY)
								{
									time_count = 0;
									timeout_count = 0;
									yaw_deg = attitude_offset.yaw_deg;
									vision_roll_thrust_control.hold_roll_yaw(thrust, position_ned, velocity_body, attitude, period_ms);
									input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
									offbCtrlAttitude(offboard, input_attitude);
									cout << "THROUGH2_OK " << endl;
									if (target[3].valid)
									{
										missions_status = THROUGH_TARGET3_MISSION;
									}
									else
									{
										missions_status = SEARCH_TARGET3_MISSION;
									}
									break;
								}
								cout << "OPEN LOOP" << endl;
								timeout_count = 0;
								vision_roll_thrust_control.hold_roll_yaw(thrust, position_ned, velocity_body, attitude, period_ms);
								input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
								offbCtrlAttitude(offboard, input_attitude);
								time_count++;
							}
							else
							{
								if (timeout_count < 0.5 * CONTROL_FREQUENCY)
								{
									cout << "HOLD" << endl;
									cout << "CAN THROUGH: " << vision_roll_thrust_control.can_through_ring_flag << endl;
									cout << "x_m < 5: " << (target[TARGET2].x_m < 5.0) << endl;
									vision_roll_thrust_control.hold_roll_yaw(thrust, position_ned, velocity_body, attitude, period_ms);
									input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
									offbCtrlAttitude(offboard, input_attitude);
								}
								else
								{ //丢失目标2秒
									yaw_deg = attitude_offset.yaw_deg;
									vision_roll_thrust_control.hold_roll_yaw(thrust, position_ned, velocity_body, attitude, period_ms);
									input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
									offbCtrlAttitude(offboard, input_attitude);
									cout << "target lost" << endl;
									timeout_count = 0;
									missions_status = LAND_MISSION;
									break;
								}
								timeout_count++;
							}
						}
					}
					else
					{
						if (timeout_count < 0.5 * CONTROL_FREQUENCY)
						{
							cout << "HOLD" << endl;
							cout << "CAN THROUGH: " << vision_roll_thrust_control.can_through_ring_flag << endl;
							cout << "x_m < 5: " << (target[TARGET1].x_m < 5.0) << endl;
							vision_roll_thrust_control.hold_roll_yaw(thrust, position_ned, velocity_body, attitude, period_ms);
							input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
							offbCtrlAttitude(offboard, input_attitude);
						}
						else
						{ //丢失目标2秒
							yaw_deg = attitude_offset.yaw_deg;
							vision_roll_thrust_control.hold_roll_yaw(thrust, position_ned, velocity_body, attitude, period_ms);
							input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
							offbCtrlAttitude(offboard, input_attitude);
							cout << "target lost" << endl;
							timeout_count = 0;
							missions_status = LAND_MISSION;
							break;
						}
						timeout_count++;
					}
				}
				break;
#endif
#if (TEST_LEVEL == LEVEL_FINAL1) || (TEST_LEVEL == LEVEL_FINAL3)
			// case SEARCH_TARGET1_MISSION:
			// 	cout << "SEARCH TARGET1" << endl;
			// 	if (!flag_search_target1_init)
			// 	{
			// 		altitude_set = TARGET1_HEIGHT;
			// 		flag_search_target1_init = true;
			// 	}
			// 	if (timeout_count < 2 * CONTROL_FREQUENCY)
			// 	{
			// 		if ((position_ned.east_m < 1e-4) && (position_ned.north_m < 1e-4))
			// 		{
			// 			//没办法，加速进度
			// 			timeout_count++;
			// 			cout << "FLOw CAN NOT THRUST" << endl;
			// 			altitude_thrust_control.takeoff(-altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
			// 			pitch_deg+=5;
			// 			input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
			// 			//cout << thrust << endl;
			// 			offbCtrlAttitude(offboard, input_attitude);
			// 			flag_flow_init = false;
			// 		}
			// 		else
			// 		{
			// 			if (!flag_flow_init)
			// 			{
			// 				altitude_set = -position_ned.down_m;
			// 				offset_body = {-1.0f, 0.0f, 0.0f};
			// 				flow_pos_thrust_control.positionBodyOffset(roll_deg, pitch_deg, thrust, offset_body, position_ned, velocity_body, attitude, period_ms);
			// 				input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
			// 				offbCtrlAttitude(offboard, input_attitude);
			// 				remotePrint("Flow init!");
			// 				flag_flow_init = true;
			// 			}
			// 			else
			// 			{
			// 				flow_pos_thrust_control.hold(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
			// 				input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
			// 				offbCtrlAttitude(offboard, input_attitude);
			// 			}
			// 		}
			// 	}
			// 	else if (timeout_count < 4 * CONTROL_FREQUENCY)
			// 	{
			// 		if ((position_ned.east_m < 1e-4) && (position_ned.north_m < 1e-4))
			// 		{
			// 			//没办法，加速降落
			// 			timeout_count++;
			// 			cout << "FLOw CAN NOT THRUST" << endl;
			// 			altitude_thrust_control.takeoff(-altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
			// 			roll_deg += 5 / CONTROL_FREQUENCY;
			// 			input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
			// 			//cout << thrust << endl;
			// 			offbCtrlAttitude(offboard, input_attitude);
			// 			flag_flow_init = false;
			// 		}
			// 		else
			// 		{
			// 			if (!flag_flow_init)
			// 			{
			// 				altitude_set = -position_ned.down_m;
			// 				offset_body = {0.0f, 1.0f, 0.0f};
			// 				flow_pos_thrust_control.positionBodyOffset(roll_deg, pitch_deg, thrust, offset_body, position_ned, velocity_body, attitude, period_ms);
			// 				input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
			// 				offbCtrlAttitude(offboard, input_attitude);
			// 				remotePrint("Flow init!");
			// 				flag_flow_init = true;
			// 			}
			// 			else
			// 			{
			// 				flow_pos_thrust_control.hold(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
			// 				input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
			// 				offbCtrlAttitude(offboard, input_attitude);
			// 			}
			// 		}
			// 	}
			// 	else if (timeout_count < 8 * CONTROL_FREQUENCY)
			// 	{
			// 		if ((position_ned.east_m < 1e-4) && (position_ned.north_m < 1e-4))
			// 		{
			// 			//没办法，加速降落
			// 			timeout_count++;
			// 			cout << "FLOw CAN NOT THRUST" << endl;
			// 			altitude_thrust_control.takeoff(-altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
			// 			roll_deg -= 5 / CONTROL_FREQUENCY;
			// 			input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
			// 			//cout << thrust << endl;
			// 			offbCtrlAttitude(offboard, input_attitude);
			// 			flag_flow_init = false;
			// 		}
			// 		else
			// 		{
			// 			if (!flag_flow_init)
			// 			{
			// 				altitude_set = -position_ned.down_m;
			// 				offset_body = {0.0f, -2.0f, 0.0f};
			// 				flow_pos_thrust_control.positionBodyOffset(roll_deg, pitch_deg, thrust, offset_body, position_ned, velocity_body, attitude, period_ms);
			// 				input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
			// 				offbCtrlAttitude(offboard, input_attitude);
			// 				remotePrint("Flow init!");
			// 				flag_flow_init = true;
			// 			}
			// 			else
			// 			{
			// 				flow_pos_thrust_control.hold(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
			// 				input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
			// 				offbCtrlAttitude(offboard, input_attitude);
			// 			}
			// 		}
			// 	}
			// 	else //超时且未找到，进入降落模式
			// 	{
			// 		cout << "TIME OUT" << endl;
			// 		timeout_count = 0;
			// 		missions_status = LAND_MISSION;
			// 		break;
			// 	}
			// 	cout << target[TARGET1].valid << endl;
			// 	//找到环，进入调整模式
			// 	if (target[TARGET1].valid == true)
			// 	{
			// 		timeout_count = 0;
			// 		cout << "FIND TARGET1" << endl;
			// 		missions_status = THROUGH_TARGET1_MISSION;
			// 		break;
			// 	}
			// 	else
			// 	{
			// 		timeout_count++;
			// 	}
			// 	break;
			case THROUGH_TARGET1_MISSION:
				cout << "THROUGH_TARGET1_MISSION" << endl;
				altitude_set = TARGET1_HEIGHT;
				vel_x_set = VEL_TARGET_1;
				altitude_thrust_control.takeoff_vx(vel_x_set, -altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
				input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
				offbCtrlAttitude(offboard, input_attitude);

				if (timeout_count > TIME_START_SEARCH_TARGET_2 * CONTROL_FREQUENCY)
				{ //穿越中
					update<int>(camera_status_topic, 2, camera_status_mtx);
				}
				else if (timeout_count > TIME_THROUGH_TARGET_1 * CONTROL_FREQUENCY)
				{
					timeout_count = 0;
					cout << "THROUGH1_OK " << endl;
					if (target[TARGET2].valid)
					{
						cout << "TARGET2 AVILABLE" << endl;
						missions_status = THROUGH_TARGET2_MISSION;
					}
					else
					{
						cout << "TARGET2 DISAVILABLE" << endl;
						missions_status = SEARCH_TARGET2_MISSION;
					}
					break;
				}
				timeout_count++;
				break;
			case SEARCH_TARGET2_MISSION:
				cout << "SEARCH TARGET2" << endl;
				if (!flag_search_target2_init)
				{
					altitude_set = TARGET2_HEIGHT;
					flag_search_target2_init = true;
				}
				if (timeout_count < 2 * CONTROL_FREQUENCY)
				{
					if ((position_ned.east_m < 1e-4) && (position_ned.north_m < 1e-4))
					{
						//没办法，加速进度
						timeout_count++;
						cout << "FLOw CAN NOT THRUST" << endl;
						altitude_thrust_control.takeoff(-altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
						pitch_deg += -10;
						input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
						//cout << thrust << endl;
						offbCtrlAttitude(offboard, input_attitude);
						flag_flow_init = false;
					}
					else
					{
						if (!flag_flow_init)
						{
							altitude_set = -position_ned.down_m;
							offset_body = {5.0f, 0.0f, 0.0f};
							flow_pos_thrust_control.positionBodyOffset(roll_deg, pitch_deg, thrust, offset_body, position_ned, velocity_body, attitude, period_ms);
							input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
							offbCtrlAttitude(offboard, input_attitude);
							remotePrint("Flow init!");
							flag_flow_init = true;
						}
						else
						{
							flow_pos_thrust_control.hold(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
							input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
							offbCtrlAttitude(offboard, input_attitude);
						}
					}
				}
				else if (timeout_count < 4 * CONTROL_FREQUENCY)
				{
					if ((position_ned.east_m < 1e-4) && (position_ned.north_m < 1e-4))
					{
						//没办法，加速降落
						timeout_count++;
						cout << "FLOw CAN NOT THRUST" << endl;
						altitude_thrust_control.takeoff(-altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
						roll_deg += 8 / CONTROL_FREQUENCY;
						input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
						//cout << thrust << endl;
						offbCtrlAttitude(offboard, input_attitude);
						flag_flow_init = false;
					}
					else
					{
						if (!flag_flow_init)
						{
							altitude_set = -position_ned.down_m;
							offset_body = {0.0f, 1.0f, 0.0f};
							flow_pos_thrust_control.positionBodyOffset(roll_deg, pitch_deg, thrust, offset_body, position_ned, velocity_body, attitude, period_ms);
							input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
							offbCtrlAttitude(offboard, input_attitude);
							remotePrint("Flow init!");
							flag_flow_init = true;
						}
						else
						{
							flow_pos_thrust_control.hold(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
							input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
							offbCtrlAttitude(offboard, input_attitude);
						}
					}
				}
				else if (timeout_count < 8 * CONTROL_FREQUENCY)
				{
					if ((position_ned.east_m < 1e-4) && (position_ned.north_m < 1e-4))
					{
						//没办法，加速降落
						timeout_count++;
						cout << "FLOw CAN NOT THRUST" << endl;
						altitude_thrust_control.takeoff(-altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
						roll_deg -= 8 / CONTROL_FREQUENCY;
						input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
						//cout << thrust << endl;
						offbCtrlAttitude(offboard, input_attitude);
						flag_flow_init = false;
					}
					else
					{
						if (!flag_flow_init)
						{
							altitude_set = -position_ned.down_m;
							offset_body = {0.0f, -2.0f, 0.0f};
							flow_pos_thrust_control.positionBodyOffset(roll_deg, pitch_deg, thrust, offset_body, position_ned, velocity_body, attitude, period_ms);
							input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
							offbCtrlAttitude(offboard, input_attitude);
							remotePrint("Flow init!");
							flag_flow_init = true;
						}
						else
						{
							flow_pos_thrust_control.hold(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
							input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
							offbCtrlAttitude(offboard, input_attitude);
						}
					}
				}
				else //超时且未找到，进入降落模式
				{
					cout << "TIME OUT" << endl;
					timeout_count = 0;
					missions_status = LAND_MISSION;
					break;
				}
				cout << target[TARGET2].valid << endl;
				//找到环，进入调整模式
				if (target[TARGET2].valid == true)
				{
					cout << "FIND TARGET2" << endl;
					timeout_count = 0;
					missions_status = THROUGH_TARGET2_MISSION;
					break;
				}
				else
				{
					timeout_count++;
				}
				break;
			case THROUGH_TARGET2_MISSION:
				cout << "THROUGH_TARGET2_MISSION" << endl;
				if (target[TARGET2].valid && !target[TARGET2].done)
				{ //冲向第2个环
					timeout_count = 0;
					vision_roll_thrust_control.update_thr_ring_flag(target[TARGET2]);
					target[TARGET2].done = vision_roll_thrust_control.can_through_ring_flag && (target[TARGET2].x_m < 5.0);
					cout << "CAN THROUGH: " << vision_roll_thrust_control.can_through_ring_flag << endl;
					cout << "x_m < 5: " << (target[TARGET2].x_m < 5.0) << endl;
					if ((fabs(target[TARGET2].y_deg) > 1.0f && target[TARGET2].x_m > 7.0f) && !YAW_MODE_LOCK)
					{
						cout << "ROLL CONTROL" << endl;
						altitude_set = TARGET2_HEIGHT;
						vel_x_set = VEL_TARGET_2_FAR;
						vision_roll_thrust_control.angleOffset_vel_z(vel_x_set, -altitude_set, pitch_deg, roll_deg, thrust, target[TARGET2], position_ned, velocity_body, attitude, period_ms);
						input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
						offbCtrlAttitude(offboard, input_attitude);
					}
					else
					{
						cout << "ROLL YAW CONTROL" << endl;
						//近距离 视觉：速度+滚转+偏航
						YAW_MODE_LOCK = true;
						altitude_set = TARGET2_HEIGHT;
						vel_x_set = VEL_TARGET_2_NEAR;
						vision_roll_thrust_control.angleOffset_roll_yaw_vel_z(vel_x_set, -altitude_set, pitch_deg, roll_deg, yaw_deg, thrust, target[TARGET2], position_ned, velocity_body, attitude, period_ms);
						input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
						offbCtrlAttitude(offboard, input_attitude);
					}
				}
				else
				{ //目标丢失，判断是否传环
					if (target[TARGET2].done)
					{ //穿越中
						update<int>(camera_status_topic, 3, camera_status_mtx);
						if (time_count > TIME_THROUGH_TARGET_2 * CONTROL_FREQUENCY)
						{
							time_count = 0;
							timeout_count = 0;
							yaw_deg = attitude_offset.yaw_deg;
							vision_roll_thrust_control.hold_roll_yaw(thrust, position_ned, velocity_body, attitude, period_ms);
							input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
							offbCtrlAttitude(offboard, input_attitude);
							cout << "THROUGH_OK " << endl;
							if (target[TARGET3].valid)
							{
								YAW_MODE_LOCK = false;
								vision_roll_thrust_control.reset(vision_pid,altitude_pid);
								missions_status = THROUGH_TARGET3_MISSION;
							}
							else
							{
								YAW_MODE_LOCK = false;
								vision_roll_thrust_control.reset(vision_pid,altitude_pid);
								missions_status = SEARCH_TARGET3_MISSION;
							}
							break;
						}
						cout << "OPEN LOOP" << endl;
						timeout_count = 0;
						vision_roll_thrust_control.hold_roll_yaw(thrust, position_ned, velocity_body, attitude, period_ms);
						input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
						offbCtrlAttitude(offboard, input_attitude);
						time_count++;
					}
					else
					{
						if (timeout_count < 0.5 * CONTROL_FREQUENCY)
						{
							cout << "HOLD" << endl;
							cout << "CAN THROUGH: " << vision_roll_thrust_control.can_through_ring_flag << endl;
							cout << "x_m < 5: " << (target[TARGET2].x_m < 5.0) << endl;
							vision_roll_thrust_control.hold_roll_yaw(thrust, position_ned, velocity_body, attitude, period_ms);
							input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
							offbCtrlAttitude(offboard, input_attitude);
						}
						else
						{ //丢失目标2秒
							yaw_deg = attitude_offset.yaw_deg;
							vision_roll_thrust_control.hold_roll_yaw(thrust, position_ned, velocity_body, attitude, period_ms);
							input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
							offbCtrlAttitude(offboard, input_attitude);
							cout << "target lost" << endl;
							timeout_count = 0;
							missions_status = LAND_MISSION;
							break;
						}
						timeout_count++;
					}
				}

				break;
#endif
			case SEARCH_TARGET3_MISSION:
				cout << "SEARCH TARGET3" << endl;
				if (!flag_search_target3_init)
				{
					altitude_set = TARGET3_HEIGHT;
					flag_search_target3_init = true;
				}
				if (timeout_count < 2 * CONTROL_FREQUENCY)
				{
					if ((position_ned.east_m < 1e-4) && (position_ned.north_m < 1e-4))
					{
						//没办法，加速进度
						timeout_count++;
						cout << "FLOw CAN NOT THRUST" << endl;
						altitude_thrust_control.takeoff(-altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
						pitch_deg += -10;
						input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
						//cout << thrust << endl;
						offbCtrlAttitude(offboard, input_attitude);
						flag_flow_init = false;
					}
					else
					{
						if (!flag_flow_init)
						{
							altitude_set = -position_ned.down_m;
							offset_body = {5.0f, 0.0f, 0.0f};
							flow_pos_thrust_control.positionBodyOffset(roll_deg, pitch_deg, thrust, offset_body, position_ned, velocity_body, attitude, period_ms);
							input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
							offbCtrlAttitude(offboard, input_attitude);
							remotePrint("Flow init!");
							flag_flow_init = true;
						}
						else
						{
							flow_pos_thrust_control.hold(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
							input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
							offbCtrlAttitude(offboard, input_attitude);
						}
					}
				}
				else if (timeout_count < 4 * CONTROL_FREQUENCY)
				{
					if ((position_ned.east_m < 1e-4) && (position_ned.north_m < 1e-4))
					{
						//没办法，加速降落
						timeout_count++;
						cout << "FLOw CAN NOT THRUST" << endl;
						altitude_thrust_control.takeoff(-altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
						roll_deg += 8 / CONTROL_FREQUENCY;
						input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
						//cout << thrust << endl;
						offbCtrlAttitude(offboard, input_attitude);
						flag_flow_init = false;
					}
					else
					{
						if (!flag_flow_init)
						{
							altitude_set = -position_ned.down_m;
							offset_body = {0.0f, 1.0f, 0.0f};
							flow_pos_thrust_control.positionBodyOffset(roll_deg, pitch_deg, thrust, offset_body, position_ned, velocity_body, attitude, period_ms);
							input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
							offbCtrlAttitude(offboard, input_attitude);
							remotePrint("Flow init!");
							flag_flow_init = true;
						}
						else
						{
							flow_pos_thrust_control.hold(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
							input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
							offbCtrlAttitude(offboard, input_attitude);
						}
					}
				}
				else if (timeout_count < 8 * CONTROL_FREQUENCY)
				{
					if ((position_ned.east_m < 1e-4) && (position_ned.north_m < 1e-4))
					{
						//没办法，加速降落
						timeout_count++;
						cout << "FLOw CAN NOT THRUST" << endl;
						altitude_thrust_control.takeoff(-altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
						roll_deg -= 8 / CONTROL_FREQUENCY;
						input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
						//cout << thrust << endl;
						offbCtrlAttitude(offboard, input_attitude);
						flag_flow_init = false;
					}
					else
					{
						if (!flag_flow_init)
						{
							altitude_set = -position_ned.down_m;
							offset_body = {0.0f, -2.0f, 0.0f};
							flow_pos_thrust_control.positionBodyOffset(roll_deg, pitch_deg, thrust, offset_body, position_ned, velocity_body, attitude, period_ms);
							input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
							offbCtrlAttitude(offboard, input_attitude);
							remotePrint("Flow init!");
							flag_flow_init = true;
						}
						else
						{
							flow_pos_thrust_control.hold(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
							input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
							offbCtrlAttitude(offboard, input_attitude);
						}
					}
				}
				else //超时且未找到，进入降落模式
				{
					cout << "TIME OUT" << endl;
					timeout_count = 0;
					missions_status = LAND_MISSION;
					break;
				}
				cout << target[TARGET3].valid << endl;
				//找到环，进入调整模式
				if (target[TARGET3].valid == true)
				{
					cout << "FIND TARGET3" << endl;
					timeout_count = 0;
					missions_status = THROUGH_TARGET3_MISSION;
					break;
				}
				else
				{
					timeout_count++;
				}
				break;
			case THROUGH_TARGET3_MISSION:
				cout << "THROUGH_TARGET3_MISSION" << endl;
				if (target[TARGET3].valid && !target[TARGET3].done)
				{ //冲向第2个环
					timeout_count = 0;
					vision_roll_thrust_control.update_thr_ring_flag(target[TARGET3]);
					target[TARGET3].done = vision_roll_thrust_control.can_through_ring_flag && (target[TARGET3].x_m < 5.0);
					cout << "CAN THROUGH: " << vision_roll_thrust_control.can_through_ring_flag << endl;
					cout << "x_m < 5: " << (target[TARGET3].x_m < 5.0) << endl;
					if ((fabs(target[TARGET3].y_deg) > 1.0f && target[TARGET3].x_m > 7.0f) && !YAW_MODE_LOCK)
					{
						cout << "ROLL CONTROL" << endl;
						altitude_set = TARGET3_HEIGHT;
						vel_x_set = 3;
						vision_roll_thrust_control.angleOffset_vel_z(vel_x_set, -altitude_set, pitch_deg, roll_deg, thrust, target[TARGET3], position_ned, velocity_body, attitude, period_ms);
						input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
						offbCtrlAttitude(offboard, input_attitude);
					}
					else
					{
						cout << "ROLL YAW CONTROL" << endl;
						//近距离 视觉：速度+滚转+偏航
						altitude_set = TARGET3_HEIGHT;
						YAW_MODE_LOCK = true;
						vel_x_set = 6;
						vision_roll_thrust_control.angleOffset_roll_yaw_vel_z(vel_x_set, -altitude_set, pitch_deg, roll_deg, yaw_deg, thrust, target[TARGET3], position_ned, velocity_body, attitude, period_ms);
						input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
						offbCtrlAttitude(offboard, input_attitude);
					}
				}
				else
				{ //目标丢失，判断是否传环
					if (target[TARGET3].done)
					{ //穿越中
						if (time_count > 1.8 * CONTROL_FREQUENCY)
						{
							time_count = 0;
							timeout_count = 0;
							yaw_deg = attitude_offset.yaw_deg;
							vision_roll_thrust_control.hold_roll_yaw(thrust, position_ned, velocity_body, attitude, period_ms);
							input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
							offbCtrlAttitude(offboard, input_attitude);
							cout << "THROUGH3_OK " << endl;
							missions_status = LAND_MISSION;
							break;
						}
						cout << "OPEN LOOP" << endl;
						timeout_count = 0;
						vision_roll_thrust_control.hold_roll_yaw(thrust, position_ned, velocity_body, attitude, period_ms);
						input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
						offbCtrlAttitude(offboard, input_attitude);
						time_count++;
					}
					else
					{
						if (timeout_count < 0.5 * CONTROL_FREQUENCY)
						{
							cout << "HOLD" << endl;
							cout << "CAN THROUGH: " << vision_roll_thrust_control.can_through_ring_flag << endl;
							cout << "x_m < 5: " << (target[TARGET3].x_m < 5.0) << endl;
							vision_roll_thrust_control.hold_roll_yaw(thrust, position_ned, velocity_body, attitude, period_ms);
							input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
							offbCtrlAttitude(offboard, input_attitude);
						}
						else
						{ //丢失目标2秒
							yaw_deg = attitude_offset.yaw_deg;
							vision_roll_thrust_control.hold_roll_yaw(thrust, position_ned, velocity_body, attitude, period_ms);
							input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
							offbCtrlAttitude(offboard, input_attitude);
							cout << "target lost" << endl;
							timeout_count = 0;
							missions_status = LAND_MISSION;
							break;
						}
						timeout_count++;
					}
				}
				break;
			case LAND_MISSION:
				if (!flag_land_init)
				{
					altitude_set = -position_ned.down_m;
					flag_land_init = true;
				}
				if (timeout_count < 1.0 * CONTROL_FREQUENCY)
				{
					altitude_thrust_control.takeoff(-altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
					input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
					//cout << thrust << endl;
					offbCtrlAttitude(offboard, input_attitude);
				}else{
					//cout << "LAND_MISSION" << endl;
					if (-position_ned.down_m > 1.0)
					{
						if ((position_ned.east_m < 1e-4) && (position_ned.north_m < 1e-4))
						{
							altitude_thrust_control.takeoff(-altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
							input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
							//cout << thrust << endl;
							offbCtrlAttitude(offboard, input_attitude);
							flag_land_init = false;
						}
						else
						{
							if (!flag_land_init)
							{
								altitude_set = -position_ned.down_m;
								offset_body = {0.0f, 0.0f, 0.0f};
								flow_pos_thrust_control.positionBodyOffset(roll_deg, pitch_deg, thrust, offset_body, position_ned, velocity_body, attitude, period_ms);
								input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
								offbCtrlAttitude(offboard, input_attitude);
								remotePrint("land init!");
								flag_land_init = true;
							}
							else
							{
								flow_pos_thrust_control.climb(-altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
								input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
								offbCtrlAttitude(offboard, input_attitude);
							}
						}
						altitude_set -= 1.0 / CONTROL_FREQUENCY;
					}
					else
					{
						altitude_set -= 0.5 / CONTROL_FREQUENCY;
						altitude_thrust_control.takeoff(-altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
						input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
						//cout << thrust << endl;
						offbCtrlAttitude(offboard, input_attitude);
					}
					if (altitude_set <= 0 && -position_ned.down_m < 0.2)
					{
						missions_status = FINISH;
						timeout_count=0;
						break;
					}
				}
				timeout_count++;
				break;
			case FINISH:
				input_attitude = {roll_deg, pitch_deg, yaw_deg, 0.0f};
				//cout << thrust << endl;
				//offbCtrlAttitude(offboard, input_attitude);
				break;
			}
			this_thread::sleep_for(milliseconds(20));
			period_ms = timestampf() - last_peroid;
			last_peroid = timestampf();
		}
		return;
	}
#else
void testLoop(shared_ptr<Telemetry> telemetry, shared_ptr<Offboard> offboard, FileNode altitude_pid, FileNode vision_pid, FileNode flow_pid)
{
	bool YAW_MODE_LOCK = false;
	int16_t status = WAIT_COMMAND;
	bool stop_search_second_loop = false;
	int64_t command_timestamp, first_target_timestamp, second_target_timestamp, third_target_timestamp;
	double param;
	MissionCommand command;
	Telemetry::PositionVelocityNED position_velocity_ned;
	Telemetry::EulerAngle euler_angle;
	PositionNED position_ned;
	VelocityNED velocity_ned;
	VelocityBody velocity_body;
	EulerAngle attitude;
	DetectionResult first_target, second_target, third_target;

	AltitudeThrustControl altitude_thrust_control;
	VisionRollThrustControl vision_roll_thrust_control;
	FlowPosThrustControl flow_pos_thrust_control;
	altitude_thrust_control.reset(altitude_pid);
	vision_roll_thrust_control.reset(vision_pid, altitude_pid);
	flow_pos_thrust_control.reset(flow_pid, altitude_pid);

	Offboard::Attitude input_attitude;
	float yaw_mid;
	float roll_deg, pitch_deg, yaw_deg, thrust, thrust_n;
	float pos_sp_z;
	//float open_loop_distance;
	int period_ms;
	high_resolution_clock::time_point open_loop_t0;
	int64_t last_peroid = timestampf();
	int fail_cnt = 0;
	Vector3f offset_body;
	while (true)
	{
		//only read telemetry data once every iteration
		position_velocity_ned = telemetry->position_velocity_ned();
		euler_angle = telemetry->attitude_euler_angle();
		memcpy(&position_ned, &position_velocity_ned.position, sizeof position_ned);
		memcpy(&velocity_ned, &position_velocity_ned.velocity, sizeof velocity_ned);
		memcpy(&attitude, &euler_angle, sizeof attitude);
		velocity_body = velocityNED2Body(velocity_ned, attitude);

		//receive ground control command
		if (latest<MissionCommand>(mission_command_topic, command_timestamp, command, mission_command_mtx))
		{
			clear<MissionCommand>(mission_command_topic, mission_command_mtx);
			status = command.index;
			param = command.argv[0];
		}
		else
		{
			param = 0.0;
		}

		if (-position_ned.down_m > ALTITUDE_UPPER_BOUND)
		{
			if (status != FORCE_QUIT_COMMAND)
				status = SAFE_QUIT_COMMAND;
		}

		//ignore compute time, this loop update every 20ms
		update<int16_t>(control_status_topic, status, control_status_mtx);
		switch (status)
		{
		case WAIT_COMMAND:
			roll_deg = attitude.roll_deg;
			pitch_deg = attitude.pitch_deg;
			yaw_deg = attitude.yaw_deg;
			pos_sp_z = position_ned.down_m;
			//do nothing
			break;
		case SAFE_QUIT_COMMAND:
			//remotePrint("[WARNIGN]: LANDING!");
			thrust = altitude_thrust_control.landing();
			input_attitude = {0.0f, 5.0f, attitude.yaw_deg, thrust};
			offbCtrlAttitude(offboard, input_attitude);
			break;
		case FORCE_QUIT_COMMAND:
			remotePrint("[WARNIGN]: QUIT OFFBOARD!");
			quitOffboard(offboard);
			return;
		case ALTITUDE_STEP_COMMAND:
			remotePrint("[LOGGING]: ALTITUDE STEP COMMAND!");
			status = ALTITUDE_STAY_MODE;
			cout << param << endl;
			thrust = altitude_thrust_control.downOffset(-param, position_ned, velocity_body, attitude, period_ms);
			input_attitude = {0.0f, 0.0f, yaw_deg, thrust};
			offbCtrlAttitude(offboard, input_attitude);
			break;
		case ALTITUDE_STAY_MODE:
			//thrust = altitude_thrust_control.hold(position_ned, velocity_body, attitude, period_ms);
			altitude_thrust_control.braking(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
			input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
			//cout << thrust << endl;
			offbCtrlAttitude(offboard, input_attitude);
			break;
		case FLOW_HOLD_COMMAND:
			offset_body = {0.0f, 0.0f, 0.0f};
			flow_pos_thrust_control.positionBodyOffset(roll_deg, pitch_deg, thrust, offset_body, position_ned, velocity_body, attitude, period_ms);
			input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
			offbCtrlAttitude(offboard, input_attitude);
			status = FLOW_HOLD_MODE;
			remotePrint("Flow Hold!");
			break;
		case FLOW_HOLD_MODE:
			flow_pos_thrust_control.hold(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
			input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
			offbCtrlAttitude(offboard, input_attitude);
			break;
		case VISION_CONTROL_COMMAND:
			if (latest<DetectionResult>(first_target_topic, first_target_timestamp, first_target, first_target_mtx) && timestampf() - first_target_timestamp < 30)
			{
				remotePrint("VISION CONTROL!");
				//status = VISION_CONTROL_MODE;
				vision_roll_thrust_control.angleOffset(roll_deg, thrust, first_target, position_ned, velocity_body, attitude, period_ms);
				status = VISION_CONTROL_MODE_YAW;
				//vision_roll_thrust_control.angleOffset_Yaw(yaw_deg, thrust, first_target, position_ned, velocity_body, attitude, period_ms);
				// vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust_n, position_ned, velocity_body, attitude, period_ms);
				input_attitude = {roll_deg, -0.0f, yaw_deg, thrust};
				offbCtrlAttitude(offboard, input_attitude);
				fail_cnt = 0;
			}
			else
			{
				thrust = altitude_thrust_control.down(pos_sp_z, position_ned, velocity_body, attitude, period_ms);
				input_attitude = {0.0f, 0.0f, yaw_deg, thrust};
				offbCtrlAttitude(offboard, input_attitude);
			}
			break;
			// case VISION_CONTROL_MODE_YAW:
			// 	if (latest<DetectionResult>(first_target_topic, target_timestamp, target, first_target_mtx) && timestampf() - target_timestamp < 30)
			// 	{
			// 		//vision_roll_thrust_control.angleOffset(roll_deg, thrust, target, position_ned, velocity_body, attitude, period_ms);
			// 		vision_roll_thrust_control.angleOffset_Yaw(yaw_deg, thrust, target, position_ned, velocity_body, attitude, period_ms);
			// 		vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust_n, position_ned, velocity_body, attitude, period_ms);
			// 		fail_cnt = 0;
			// 		vision_roll_thrust_control.update_thr_ring_flag(target);
			// 	}
			// 	else
			// 	{
			// 		//vision_roll_thrust_control.hold(roll_deg, thrust, position_ned, velocity_body, attitude, period_ms);
			// 		vision_roll_thrust_control.hold_yaw(thrust, position_ned, velocity_body, attitude, period_ms);
			// 		vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust_n, position_ned, velocity_body, attitude, period_ms);
			// 		fail_cnt++;
			// 	}
			// 	if (fail_cnt > VISION_FAIL_TOLERENCE) //1s?
			// 	{
			// 		vision_roll_thrust_control.can_through_ring_flag = false;
			// 		if (vision_roll_thrust_control.can_through_ring_flag)
			// 		{
			// 			status = VISION_OPEN_LOOP_MODE;
			// 			cout << "VISION OPEN LOOP!" << endl;
			// 			break;
			// 		}
			// 		else
			// 		{
			// 			remotePrint("TIMEOUT!");
			// 			//vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust, pos_ned, vel_body, attitude, period_ms);
			// 			//status = BRAKING;
			// 			status = SAFE_QUIT_COMMAND;
			// 			//status = SAFE_QUIT_COMMAND;
			// 			break;
			// 		}
			// 	}
			// 	cout << "yaw_deg_sp: " << yaw_deg << endl;
			// 	input_attitude = {roll_deg, -10.0f, yaw_deg, thrust};
			// 	offbCtrlAttitude(offboard, input_attitude);
			// 	break;

			// case VISION_CONTROL_COMMAND:
			// 	if (latest<DetectionResult>(first_target_topic, first_target_timestamp, first_target, first_target_mtx) && timestampf() - first_target_timestamp < 30)
			// 	{
			// 		// remotePrint("VISION CONTROL!");
			// 		// status = VISION_CONTROL_MODE_YAW;
			// 		// vision_roll_thrust_control.angleOffset_roll_yaw(roll_deg,yaw_deg,thrust,target,position_ned, velocity_body, attitude, period_ms);
			// 		// input_attitude = {roll_deg, -5.0f, yaw_deg, thrust};
			// 		// offbCtrlAttitude(offboard, input_attitude);
			// 		// fail_cnt = 0;
			// 		// remotePrint("VISION CONTROL!");
			// 		// status = VISION_COLLIMATION;
			// 		// yaw_mid = attitude.yaw_deg;
			// 		// vision_roll_thrust_control.collimation1(yaw_deg, thrust, first_target, position_ned, velocity_body, attitude, period_ms );
			// 		// input_attitude = {0.0f, -5.0f, yaw_deg, thrust};
			// 		// offbCtrlAttitude(offboard, input_attitude);
			// 		// fail_cnt = 0;
			// 	}
			// 	else if (latest<DetectionResult>(second_target_topic, second_target_timestamp, second_target, second_target_mtx) && timestampf() - second_target_timestamp < 30){
			// 		vision_roll_thrust_control.collimation2( roll_deg, thrust, second_target, position_ned, velocity_body, attitude, period_ms );
			// 		input_attitude = {roll_deg, -5.0f, yaw_deg, thrust};
			// 		offbCtrlAttitude(offboard, input_attitude);
			// 		fail_cnt = 0;
			// 	}
			// 	else
			// 	{
			// 		thrust = altitude_thrust_control.down(pos_sp_z, position_ned, velocity_body, attitude, period_ms);
			// 		input_attitude = {0.0f, 0.0f, yaw_deg, thrust};
			// 		offbCtrlAttitude(offboard, input_attitude);
			// 	}
			// 	break;
			// case VISION_COLLIMATION:
			// 	if (latest<DetectionResult>(first_target_topic, first_target_timestamp, first_target, first_target_mtx) && timestampf() - first_target_timestamp < 120){
			// 		vision_roll_thrust_control.collimation1(yaw_deg, thrust, first_target, position_ned, velocity_body, attitude, period_ms );
			// 		fail_cnt = 0;
			// 		vision_roll_thrust_control.update_thr_ring_flag(first_target);
			// 		vision_roll_thrust_control.update_the_first_thr_flag(velocity_body);
			// 		if(vision_roll_thrust_control.can_start_the_first_thr){
			// 			pitch_deg = -5.0f;
			// 			stop_search_second_loop = true;
			// 		}
			// 		else pitch_deg = 0.0f;
			// 	}
			// 	else if(!stop_search_second_loop && latest<DetectionResult>(second_target_topic, second_target_timestamp, second_target, second_target_mtx) && timestampf() - second_target_timestamp < 120)
			// 	{
			// 		//vision_roll_thrust_control.collimation(yaw_deg, roll_deg, thrust, first_target, second_target, position_ned, velocity_body, attitude, period_ms );
			// 		vision_roll_thrust_control.collimation2( roll_deg, thrust, second_target, position_ned, velocity_body, attitude, period_ms );

			// 	}
			// 	else
			// 	{
			// 		//vision_roll_thrust_control.hold(roll_deg, thrust, position_ned, velocity_body, attitude, period_ms);
			// 		vision_roll_thrust_control.hold_roll_yaw(thrust, position_ned, velocity_body, attitude, period_ms);
			// 		fail_cnt++;
			// 	}
			// 	if (fail_cnt > VISION_FAIL_TOLERENCE) //1s?
			// 	{
			// 		vision_roll_thrust_control.can_through_ring_flag = false;
			// 		if (vision_roll_thrust_control.can_through_ring_flag)
			// 		{
			// 			status = VISION_OPEN_LOOP_MODE;
			// 			cout << "VISION OPEN LOOP!" << endl;
			// 			break;
			// 		}
			// 		else
			// 		{
			// 			remotePrint("TIMEOUT!");
			// 			//vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust, pos_ned, vel_body, attitude, period_ms);
			// 			//status = BRAKING;
			// 			status = SAFE_QUIT_COMMAND;
			// 			//status = SAFE_QUIT_COMMAND;
			// 			break;
			// 		}
			// 	}
			// 	input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
			// 	cout << "roll_deg: " << roll_deg << endl;
			// 	offbCtrlAttitude(offboard, input_attitude);
			// 	break;
			// case VISION_CONTROL_MODE_YAW:
			// 	if(flag_loop == 2){
			// 		if (latest<DetectionResult>(second_target_topic, second_target_timestamp, second_target, second_target_mtx) && timestampf() - second_target_timestamp < 30)
			// 		{
			// 			//vision_roll_thrust_control.angleOffset(roll_deg, thrust, target, position_ned, velocity_body, attitude, period_ms);
			// 			if(second_target.z_m > 10.0f) {
			// 				vision_roll_thrust_control.angleOffset(roll_deg, thrust, second_target, position_ned, velocity_body, attitude, period_ms);
			// 				pitch_deg = -5.0f;
			// 			}
			// 			else{
			// 				vision_roll_thrust_control.angleOffset_roll_yaw(roll_deg,yaw_deg,thrust,second_target,position_ned, velocity_body, attitude, period_ms);
			// 				pitch_deg = -10.0f;
			// 			}
			// 			//vision_roll_thrust_control.angleOffset_roll_yaw(roll_deg, yaw_deg, thrust, target, position_ned, velocity_body, attitude, period_ms);
			// 			fail_cnt = 0;
			// 			vision_roll_thrust_control.update_thr_ring_flag(second_target);
			// 		}
			// 		else
			// 		{
			// 			//vision_roll_thrust_control.hold(roll_deg, thrust, position_ned, velocity_body, attitude, period_ms);
			// 			vision_roll_thrust_control.hold_roll_yaw(thrust, position_ned, velocity_body, attitude, period_ms);
			// 			fail_cnt++;
			// 		}
			// 		if (fail_cnt > VISION_FAIL_TOLERENCE) //1s?
			// 		{
			// 			vision_roll_thrust_control.can_through_ring_flag = false;
			// 			if (vision_roll_thrust_control.can_through_ring_flag)
			// 			{
			// 				status = VISION_OPEN_LOOP_MODE;
			// 				cout << "VISION OPEN LOOP!" << endl;
			// 				break;
			// 			}
			// 			else
			// 			{
			// 				remotePrint("TIMEOUT!");
			// 				//vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust, pos_ned, vel_body, attitude, period_ms);
			// 				//status = BRAKING;
			// 				status = SAFE_QUIT_COMMAND;
			// 				//status = SAFE_QUIT_COMMAND;
			// 				break;
			// 			}
			// 		}
			// 	}
			// 	else if(flag_loop == 3){
			// 		if (latest<DetectionResult>(third_target_topic, third_target_timestamp, third_target, third_target_mtx) && timestampf() - third_target_timestamp < 30)
			// 		{
			// 			//vision_roll_thrust_control.angleOffset(roll_deg, thrust, target, position_ned, velocity_body, attitude, period_ms);
			// 			if(second_target.z_m > 10.0f) {
			// 				vision_roll_thrust_control.angleOffset(roll_deg, thrust, third_target, position_ned, velocity_body, attitude, period_ms);
			// 				pitch_deg = -5.0f;
			// 			}
			// 			else{
			// 				vision_roll_thrust_control.angleOffset_roll_yaw(roll_deg,yaw_deg,thrust,third_target,position_ned, velocity_body, attitude, period_ms);
			// 				pitch_deg = -10.0f;
			// 			}
			// 			//vision_roll_thrust_control.angleOffset_roll_yaw(roll_deg, yaw_deg, thrust, target, position_ned, velocity_body, attitude, period_ms);
			// 			fail_cnt = 0;
			// 			vision_roll_thrust_control.update_thr_ring_flag(third_target);
			// 		}
			// 		else
			// 		{
			// 			//vision_roll_thrust_control.hold(roll_deg, thrust, position_ned, velocity_body, attitude, period_ms);
			// 			vision_roll_thrust_control.hold_roll_yaw(thrust, position_ned, velocity_body, attitude, period_ms);
			// 			fail_cnt++;
			// 		}
			// 		if (fail_cnt > VISION_FAIL_TOLERENCE) //1s?
			// 		{
			// 			vision_roll_thrust_control.can_through_ring_flag = false;
			// 			if (vision_roll_thrust_control.can_through_ring_flag)
			// 			{
			// 				status = VISION_OPEN_LOOP_MODE;
			// 				cout << "VISION OPEN LOOP!" << endl;
			// 				break;
			// 			}
			// 			else
			// 			{
			// 				remotePrint("TIMEOUT!");
			// 				//vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust, pos_ned, vel_body, attitude, period_ms);
			// 				//status = BRAKING;
			// 				status = SAFE_QUIT_COMMAND;
			// 				//status = SAFE_QUIT_COMMAND;
			// 				break;
			// 			}
			// 		}
			// 	}
			// 	input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
			// 	offbCtrlAttitude(offboard, input_attitude);
			// 	break;

			// case VISION_CONTROL_MODE:
			// 	if (latest<DetectionResult>(first_target_topic, first_target_timestamp, first_target, first_target_mtx) && timestampf() - first_target_timestamp < 30)
			// 	{
			// 		vision_roll_thrust_control.angleOffset(roll_deg, thrust, first_target, position_ned, velocity_body, attitude, period_ms);
			// 		fail_cnt = 0;
			// 		vision_roll_thrust_control.update_thr_ring_flag(first_target);
			// 	}
			// 	else
			// 	{
			// 		vision_roll_thrust_control.hold(roll_deg, thrust, position_ned, velocity_body, attitude, period_ms);
			// 		fail_cnt++;
			// 	}
			// 	if (fail_cnt > VISION_FAIL_TOLERENCE) //1s?
			// 	{
			// 		vision_roll_thrust_control.can_through_ring_flag = false;
			// 		if (vision_roll_thrust_control.can_through_ring_flag)
			// 		{
			// 			status = VISION_OPEN_LOOP_MODE;
			// 			cout << "VISION OPEN LOOP!" << endl;
			// 			break;
			// 		}
			// 		else
			// 		{
			// 			remotePrint("TIMEOUT!");
			// 			//vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust, pos_ned, vel_body, attitude, period_ms);
			// 			//status = BRAKING;
			// 			status = SAFE_QUIT_COMMAND;
			// 			//status = SAFE_QUIT_COMMAND;
			// 			break;
			// 		}
			// 	}
			// 	input_attitude = {roll_deg, -0.0f, yaw_deg, thrust};
			// 	offbCtrlAttitude(offboard, input_attitude);
			// 	break;
		case VISION_CONTROL_MODE_YAW:

			if (latest<DetectionResult>(first_target_topic, first_target_timestamp, first_target, first_target_mtx) && timestampf() - first_target_timestamp < 30)
			{
				//vision_roll_thrust_control.angleOffset(roll_deg, thrust, target, position_ned, velocity_body, attitude, period_ms);
				if (fabs(atan2f(first_target.x_m, first_target.z_m)) > 0.02f && !YAW_MODE_LOCK)
				{
					vision_roll_thrust_control.angleOffset(roll_deg, thrust, first_target, position_ned, velocity_body, attitude, period_ms);
					pitch_deg = -5.0f;
				}
				else
				{
					cout << "control yaw & roll";
					YAW_MODE_LOCK = true;
					vision_roll_thrust_control.angleOffset_roll_yaw(roll_deg, yaw_deg, thrust, first_target, position_ned, velocity_body, attitude, period_ms);
					pitch_deg = OPEN_LOOP_PITCH_DEG;
				}
				// vision_roll_thrust_control.angleOffset_roll_yaw(roll_deg,yaw_deg,thrust,first_target,position_ned, velocity_body, attitude, period_ms);
				// pitch_deg = OPEN_LOOP_PITCH_DEG;
				//vision_roll_thrust_control.angleOffset_roll_yaw(roll_deg, yaw_deg, thrust, target, position_ned, velocity_body, attitude, period_ms);
				fail_cnt = 0;
				vision_roll_thrust_control.update_thr_ring_flag(first_target);
			}
			else
			{
				//vision_roll_thrust_control.hold(roll_deg, thrust, position_ned, velocity_body, attitude, period_ms);
				vision_roll_thrust_control.hold_roll_yaw(thrust, position_ned, velocity_body, attitude, period_ms);
				fail_cnt++;
			}
			if (first_target.z_m < 5.0f && fail_cnt > 0.2 * VISION_FAIL_TOLERENCE && vision_roll_thrust_control.can_through_ring_flag)
			{
				fail_cnt = 0;
				status = VISION_OPEN_LOOP_MODE;
				cout << "VISION OPEN LOOP!" << endl;
			}
			if (fail_cnt > 1 * VISION_FAIL_TOLERENCE) //1s?
			{
				//vision_roll_thrust_control.can_through_ring_flag = false;
				remotePrint("TIMEOUT!");
				//vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust, pos_ned, vel_body, attitude, period_ms);
				status = BRAKING;
				//status = SAFE_QUIT_COMMAND;
				//status = SAFE_QUIT_COMMAND;
				break;
			}
			input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
			offbCtrlAttitude(offboard, input_attitude);
			break;
		case VISION_OPEN_LOOP_MODE:
			fail_cnt++;
			if (fail_cnt < 0.6 * OPEN_LOOP_TOLERENCE)
			{
				vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
				input_attitude = {roll_deg, OPEN_LOOP_PITCH_DEG, yaw_deg, thrust};
				offbCtrlAttitude(offboard, input_attitude);
			}
			else //finish through
			{
				if (flag_loop == 1)
				{
					cout << "FLAG_LOOP: 1" << endl;
					status = BRAKING;
					cout << "BRAKING" << endl;
					input_attitude = {roll_deg, -OPEN_LOOP_PITCH_DEG, yaw_deg, thrust};
					offbCtrlAttitude(offboard, input_attitude);
					break;
					// if(latest<DetectionResult>(second_target_topic, second_target_timestamp, second_target, second_target_mtx) && timestampf() - second_target_timestamp < 30){//we think we can see the second loop directly
					// 	status = VISION_CONTROL_MODE_YAW;
					// 	flag_loop = 2;
					// 	vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
					// 	input_attitude = {roll_deg, -10.0f, yaw_deg, thrust};
					// 	offbCtrlAttitude(offboard, input_attitude);
					// }
					// else {//braking and try to find loop , we think we can see the second loop directly
					// 	vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
					// 	input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
					// 	offbCtrlAttitude(offboard, input_attitude);
					// }
				}
				else if (flag_loop == 2)
				{
					if (latest<DetectionResult>(third_target_topic, third_target_timestamp, third_target, third_target_mtx) && timestampf() - third_target_timestamp < 30)
					{
						status = VISION_CONTROL_MODE_YAW;
						flag_loop = 3;
						vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
						input_attitude = {roll_deg, -10.0f, yaw_deg, thrust};
						offbCtrlAttitude(offboard, input_attitude);
					}
					else
					{ //don't find the loop,change yaw to yaw_mid
						yaw_deg = yaw_mid;
						vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
						input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
						offbCtrlAttitude(offboard, input_attitude);
					}
				}
			}
			break;
		case BRAKING:
			vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
			input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
			offbCtrlAttitude(offboard, input_attitude);
			break;
		case SEARCH_RING:
			vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
			input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
			offbCtrlAttitude(offboard, input_attitude);
			break;
		}
		this_thread::sleep_for(milliseconds(20));
		period_ms = timestampf() - last_peroid;
		last_peroid = timestampf();
	}
	return;
}
#endif