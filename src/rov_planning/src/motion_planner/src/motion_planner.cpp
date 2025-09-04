/**
 * @file motion_planner.cpp
 * @brief
 * @author IAT Digital
 * @version 1.0
 * @date 2025-8-19
 *
 * @copyright Copyright (c) 2025-  阿尔特汽车技术股份有限公司
 *
 * Unpublished copyright. All rights reserved. This material contains
 * proprietary information that should be used or copied only within
 * IAT, except with written permission of IAT.
 *
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2025-8-19 <td>1.0     <td>IAT Digital     <td>Initialize create
 * </table>
 */
 #include "motion_planner.h"
 
namespace rov_planning {
	 
MotionPlanner::MotionPlanner(std::shared_ptr<RosCom>& ros_com)
	: closed_(true),
	  yaw_switch_(true),
	  depth_switch_(true),
	  ros_com_(ros_com)
{
	
}

bool MotionPlanner::Init()
{
	
	ros_com_->GetPIDParams(pid_param_vec_);
	const int pid_vec_size = 4;
	for (int i = 0; i < pid_vec_size; ++i) {
		std::shared_ptr<PDController> pid = std::make_shared<PDController>();
		pid->UpdateParams(pid_param_vec_[i]);
		pid_vec_.emplace_back(pid);
	}
	pid_switch_ = ros_com_->GetPIDSwitch();
}

void MotionPlanner::Start()
{
	if (!pid_switch_) {
		return;
	}
	closed_.store(false);
    thread_ = std::thread(&MotionPlanner::Process, this);
}

void MotionPlanner::Stop()
{
	if (!pid_switch_) {
		return;
	}
	closed_.store(true);
    try {
        if (thread_.joinable()) {
            thread_.join();
        }
    
    } catch (const std::exception &e) {
        std::cout << "catched motion planner thread_ exception (shutdown):" << e.what() << std::endl;
    }
}

void  MotionPlanner::Process()
{
	const int sleep_time_at_beginning = 2;
	std::this_thread::sleep_for(std::chrono::seconds(sleep_time_at_beginning));
	std::chrono::steady_clock::time_point next_run(std::chrono::steady_clock::now());
    std::chrono::steady_clock::duration duration(std::chrono::milliseconds(MotionPlanner::kPeriodTime));

	const double neutral_duty = 1500.0;
	const double ms_to_s = 1000.0;
	final_target_.imu_info.roll = 0.0;
	final_target_.imu_info.pitch = 0.0;		
	double pid_period = kPeriodTime / ms_to_s;
	
	ModeStatus status;
	ImuInfo imu;
	ros_com_->GetState(imu, status);
	ROS_INFO("-----first_yaw: %f", imu.yaw);
	final_target_.imu_info.yaw = imu.yaw;
	final_target_.depth = status.depth;		// status.depth
	ROS_INFO("init_target_yaw : %f, init_target_depth : %f-----", final_target_.imu_info.yaw, final_target_.depth);
	
	const double yaw_gradient = pid_param_vec_.at(YAW_PID).gradient;
	const double depth_gradient = pid_param_vec_.at(DEPTH_PID).gradient;
	
    while (!closed_.load()) {		
	//ROS_INFO("---------enter while------------");
		if (status.motion_status == MODE_MANUAL) {	  // MODE_ALT_HOLD
			//ROS_INFO("fianl---target_pitch:%f, target_yaw:%f, target_roll:%f, target_depth:%f",final_target_.imu_info.pitch, final_target_.imu_info.yaw, final_target_.imu_info.roll, final_target_.depth);
		    ros_com_->GetState(imu, status);
			current_value_.imu_info = imu;
			current_value_.depth = status.depth;
			{
				std::lock_guard<std::mutex> lock(mutex_);
				RunRollControlPID(pid_period);
				RunPitchControlPID(pid_period);	
				RunYawControlPID(pid_period, yaw_gradient);	
				RunDepthControlPID(pid_period, depth_gradient);					
			}		
		}
        next_run += duration;
        /* Blocks the execution of the current thread until specified point of time (next_run) has been reached */
        std::this_thread::sleep_until(next_run);
    }
	
	ROS_WARN("exit process");
}

void MotionPlanner::RunRollControlPID(double pid_period)
{
	static bool last_roll_controlling = false;
	RCControl send;
	const double neutral_duty = 1500.0;			
	bool current_roll_controlling = fabs(final_target_.imu_info.roll - current_value_.imu_info.roll) > pid_param_vec_.at(ROLL_PID).start_offset;
	if (current_roll_controlling) {
		ROS_INFO("--------------roll pid-------------");
		ROS_INFO("current_roll :%f ", current_value_.imu_info.roll);	// ROS_INFO
		ROS_INFO("start_offset_roll :%f ", pid_param_vec_.at(ROLL_PID).start_offset);	// ROS_INFO
		send.rc_control_value = pid_vec_.at(ROLL_PID)->Calculate(final_target_.imu_info.roll,  current_value_.imu_info.roll, pid_period)
			+ neutral_duty;
		ROS_INFO("roll_rc_in : %f", send.rc_control_value);	// ROS_INFO
		send.rc_in = ROLL_CONTROL;
		ros_com_->RCControlCallBack(send);
	} else if (last_roll_controlling && !current_roll_controlling) {
		// 从控制状态切换到非控制状态，发送一次中性值
		send.rc_control_value = neutral_duty;
		send.rc_in = ROLL_CONTROL;
		ros_com_->RCControlCallBack(send);
		ROS_INFO("Roll control stopped, sending neutral once");
		pid_vec_.at(ROLL_PID)->Reset();
	}
	last_roll_controlling = current_roll_controlling;
}

void MotionPlanner::RunPitchControlPID(double pid_period)
{
	static bool last_pitch_controlling = false;
	RCControl send;
	const double neutral_duty = 1500.0;	
	bool current_pitch_controlling = fabs(final_target_.imu_info.pitch - current_value_.imu_info.pitch) > pid_param_vec_.at(PITCH_PID).start_offset;
	if (current_pitch_controlling) {
		ROS_INFO("-------------pitch pid--------------");
		ROS_INFO("current_pitch :%f ", current_value_.imu_info.pitch);	// ROS_INFO
		ROS_INFO("start_offset_pitch :%f ", pid_param_vec_.at(PITCH_PID).start_offset);	// ROS_INFO
		send.rc_control_value = pid_vec_.at(PITCH_PID)->Calculate(final_target_.imu_info.pitch,  current_value_.imu_info.pitch, pid_period)
			+ neutral_duty;
		ROS_INFO("roll_rc_in : %f", send.rc_control_value);	// ROS_INFO
		send.rc_in = PITCH_CONTROL;
		ros_com_->RCControlCallBack(send);
	} else if (last_pitch_controlling && !current_pitch_controlling) {
		// 从控制状态切换到非控制状态，发送一次中性值
		send.rc_control_value = neutral_duty;
		send.rc_in = PITCH_CONTROL;
		ros_com_->RCControlCallBack(send);
		ROS_INFO("Pitch control stopped, sending neutral once");
		pid_vec_.at(PITCH_PID)->Reset();
	}
	last_pitch_controlling = current_pitch_controlling;
}

void MotionPlanner::RunYawControlPID(double pid_period, double yaw_gradient)
{
	static bool last_yaw_controlling = false;
	double yaw_error = 0.0;
	RCControl send;
	const double neutral_duty = 1500.0;	
	if (yaw_switch_.load()) {
		yaw_error = final_target_.imu_info.yaw - current_value_.imu_info.yaw;
		ResetByLimit(yaw_error);
		
		bool current_yaw_controlling = fabs(yaw_error) > pid_param_vec_.at(YAW_PID).start_offset;
		if (current_yaw_controlling) {
			double temp_target = 0.0;
			ROS_INFO("--------------yaw pid-------------");
			ROS_INFO("current_yaw :%f ", current_value_.imu_info.yaw);	// ROS_INFO
			ROS_INFO("start_offset_yaw :%f ", pid_param_vec_.at(YAW_PID).start_offset);
			if (yaw_error > 0) {
				temp_target = (yaw_error > yaw_gradient) ? 
					(current_value_.imu_info.yaw + yaw_gradient) : final_target_.imu_info.yaw;
			} else {
				temp_target = (yaw_error < -yaw_gradient) ? 
					(current_value_.imu_info.yaw - yaw_gradient) : final_target_.imu_info.yaw;
			}
			ResetByLimit(temp_target);
			ROS_INFO("temp_target_yaw :%f, final_target_yaw: %f", temp_target, final_target_.imu_info.yaw);	// ROS_INFO
			send.rc_control_value = pid_vec_.at(YAW_PID)->Calculate(temp_target,  current_value_.imu_info.yaw, pid_period)
				+ neutral_duty;
			ROS_INFO("yaw_rc_in : %f", send.rc_control_value);	// ROS_INFO
			send.rc_in = YAW_CONTROL;
			ros_com_->RCControlCallBack(send);
		} else if (last_yaw_controlling && !current_yaw_controlling) {
			// 从控制状态切换到非控制状态，发送一次中性值
			send.rc_control_value = neutral_duty;
			send.rc_in = YAW_CONTROL;
			ros_com_->RCControlCallBack(send);
			ROS_INFO("Yaw control stopped, sending neutral once");
			pid_vec_.at(YAW_PID)->Reset();
		}
		last_yaw_controlling = current_yaw_controlling;
	} 
	/*else {
		if (last_yaw_controlling) {
			// Yaw开关关闭，如果之前正在控制，发送一次中性值
			send.rc_control_value = neutral_duty;
			send.rc_in = YAW_CONTROL;
			ros_com_->RCControlCallBack(send);
			ROS_INFO("Yaw control disabled, sending neutral once");
		}
		pid_vec_.at(YAW_PID)->Reset();
		last_yaw_controlling = false;
	}*/
			
}

void MotionPlanner::RunDepthControlPID(double pid_period, double depth_gradient)
{
	RCControl send;
	static bool last_depth_controlling = false;	
	const double neutral_duty = 1500.0;	
	if (depth_switch_.load()) {
		bool current_depth_controlling = fabs(final_target_.depth - current_value_.depth) > pid_param_vec_.at(DEPTH_PID).start_offset;
		if (current_depth_controlling) {
			double temp_target = 0.0;
			ROS_INFO("--------------depth pid-------------");
			ROS_INFO("current_depth :%f ", current_value_.depth);	// ROS_INFO
			ROS_INFO("start_offset_depth :%f ", pid_param_vec_.at(DEPTH_PID).start_offset);	// ROS_INFO
			if (final_target_.depth > current_value_.depth) {
				temp_target = (final_target_.depth - current_value_.depth > depth_gradient) ? 
					current_value_.depth + depth_gradient : final_target_.depth;
			} else {
				temp_target = (final_target_.depth - current_value_.depth < -depth_gradient) ? 
					current_value_.depth - depth_gradient : final_target_.depth;
			}
			ROS_INFO("temp_target_depth :%f ", temp_target);	// ROS_INFO
			send.rc_control_value = pid_vec_.at(DEPTH_PID)->Calculate(temp_target,  current_value_.depth, pid_period)
				+ neutral_duty;
			ROS_INFO("depth_rc_in : %f", send.rc_control_value);	// ROS_INFO
			send.rc_in = THROTTLE_CONTROL;
			ros_com_->RCControlCallBack(send);
		} else if (last_depth_controlling && !current_depth_controlling) {
			// 从控制状态切换到非控制状态，发送一次中性值
			send.rc_control_value = neutral_duty;
			send.rc_in = THROTTLE_CONTROL;
			ros_com_->RCControlCallBack(send);
			ROS_INFO("Depth control stopped, sending neutral once");
			pid_vec_.at(DEPTH_PID)->Reset();
		}
		last_depth_controlling = current_depth_controlling;
	} 
	/*else {
		if (last_depth_controlling) {
			// Depth开关关闭，如果之前正在控制，发送一次中性值
			send.rc_control_value = neutral_duty;
			send.rc_in = THROTTLE_CONTROL;
			ros_com_->RCControlCallBack(send);
			ROS_INFO("Depth control disabled, sending neutral once");
		}
		pid_vec_.at(DEPTH_PID)->Reset();
		last_depth_controlling = false;
	}*/
}

void MotionPlanner::SetTargetValue(int type, double value)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (type == YAW_PID) {
		final_target_.imu_info.yaw = value;
		yaw_switch_.store(true);
		ROS_ERROR("set_final_target_.imu_info: %f",final_target_.imu_info.yaw);
	} else if (type == DEPTH_PID) {
		final_target_.depth = value;
		depth_switch_.store(true);
		ROS_ERROR("set_final_target_.depth: %f",final_target_.depth);
	} else {
	}
}

void MotionPlanner::SetManualControlValue(const RCControl& manual_control)
{
	const double neutral_duty = 1500.0;
	const double offset_duty = 20.0;
	std::lock_guard<std::mutex> lock(mutex_);
	switch (manual_control.rc_in) {
		case CLOSE_CONTROL :
		{			
			final_target_.imu_info.yaw = current_value_.imu_info.yaw;
			final_target_.depth = current_value_.depth;
			yaw_switch_.store(true);
			depth_switch_.store(true);
			//ROS_WARN("yaw_switch_---true, depth_switch_---true");
			break;
		}
		case YAW_CONTROL :
		{
			if (fabs(manual_control.rc_control_value - neutral_duty) < offset_duty) {
				yaw_switch_.store(true);
				//ROS_WARN("yaw_switch_---true");
				final_target_.imu_info.yaw = current_value_.imu_info.yaw;
			} else {
				yaw_switch_.store(false);
				//ROS_WARN("yaw_switch_---false");
			}
			break;
		}
		case THROTTLE_CONTROL :
		{
			if (fabs(manual_control.rc_control_value - neutral_duty) < offset_duty) {
				depth_switch_.store(true);
				//ROS_WARN("depth_switch_---true");
				final_target_.depth = current_value_.depth;
			} else {
				depth_switch_.store(false);
				//ROS_WARN("depth_switch_---false");
			}
			break;
		}
		default : 
			break;
	}	
}

}