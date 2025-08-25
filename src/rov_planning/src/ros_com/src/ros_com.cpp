#include "ros_com.h"
#include <sstream>

namespace rov_planning {

RosCom::RosCom() 
{
    // 初始化ROS节点句柄
}

void RosCom::Init() 
{
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    armed_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    state_sub_ = nh_.subscribe("/mavros/state", 1000, &RosCom::StateCallBack, this);
    rc_out_sub_  = nh_.subscribe("/mavros/rc/out", 1000, &RosCom::RCOutCallBack, this);
    altitude_sub_ = nh_.subscribe("/mavros/global_position/rel_alt", 1000, &RosCom::AltitudeCallBack, this);
    imu_sub_ = nh_.subscribe("/mavros/imu/data", 1000, &RosCom::ImuInfoCallBack, this);
    ultrasonic_sensor_sub_ = nh_.subscribe("mavros/sensor/distance", 1000, &RosCom::UltrasonicCallBack, this);
    battery_status_sub_ = nh_.subscribe("/mavros/battery", 1000, &RosCom::BatteryStatusCallBack, this);
    twist_sub_ = nh_.subscribe("/mavros/local_position/velocity_body", 1000, &RosCom::TwistCallBack, this);
	GetRosParam();
    
    const int neutral_duty = 1500;
    for (int i = 0 ; i < RC_OUT_VEC_SIZE; i++) {
        rc_out_vec_.emplace_back(neutral_duty);
    }
    RCControl surface_depth_msg{g_surface_depth_, SURFACE_DEPTH};
    RCControlCallBack(surface_depth_msg); 

    RCControl bottom_offset_msg{g_bottom_offset_, BOTTOM_OFFSET};
    RCControlCallBack(bottom_offset_msg);

    RCControl front_pitch_fix_msg{g_front_pitch_fix_,FRONT_PITCH_FIX};
    RCControlCallBack(front_pitch_fix_msg);

    RCControl rear_pitch_fix_msg{g_rear_pitch_fix_,REAR_PITCH_FIX};
    RCControlCallBack(rear_pitch_fix_msg);
}

void RosCom::StateCallBack(const mavros_msgs::State::ConstPtr& msg) 
{
    
    mode_status_.armed_status = msg->armed;
    if (msg->mode == "MANUAL") {
        mode_status_.motion_status = MODE_MANUAL;
    } else if (msg->mode == "ALT_HOLD") {
        mode_status_.motion_status = MODE_ALT_HOLD;
    } else {

    }
    
    {
		std::lock_guard<std::mutex> lock(mutex_);
		mode_status_.depth = current_depth_;
    }
    MQTT::Mqtt_imp::get_single().SetState(mode_status_);

}

void RosCom::RCOutCallBack(const mavros_msgs::RCOut::ConstPtr& msg) 
{
    for (int i = 0 ; i < RC_OUT_VEC_SIZE; i++) {
        rc_out_vec_[i] = msg->channels[i];
    }

   MQTT::Mqtt_imp::get_single().SetRCOut(rc_out_vec_);
}

std::string RosCom::SetChannels(const RCControl& send)
{
    int duty = static_cast<int>(send.rc_control_value);
    // ROS_INFO("rc_control_value:%f,duty:%d",send.rc_control_value,duty);
    // duty = std::max(duty, MIN_DUTY);
    // duty = std::min(duty, MAX_DUTY);
    
    return std::to_string(duty);
}

void RosCom::RCControlCallBack(const RCControl& send) 
{
	mavros_msgs::SetMode srv; 
    const std::string neutral_duty = "1500";  
    if (send.rc_in == CLOSE_CONTROL) {  
		srv.request.custom_mode = neutral_duty;
		ROS_INFO("type:%d, duty:%f",send.rc_in,send.rc_control_value);
		for (int i = FORWARD_CONTROL; i <= PITCH_CONTROL; i++) {
			srv.request.base_mode = CONTROL_TYPE + i;
			set_mode_client_.call(srv);	
		}  
		// set_mode_client_.call(srv);		
    } else if (send.rc_in >= FORWARD_CONTROL && send.rc_in <= REAR_PITCH_FIX) {
        srv.request.base_mode = CONTROL_TYPE + send.rc_in;
        ROS_INFO("type:%d, duty:%f",send.rc_in,send.rc_control_value);
        srv.request.custom_mode = SetChannels(send);
		set_mode_client_.call(srv);
    } else {

    }
	
}

void RosCom::AltitudeCallBack(const std_msgs::Float64::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    current_depth_ = -msg->data;
}

void RosCom::ImuInfoCallBack(const sensor_msgs::Imu::ConstPtr& msg) 
{
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );
    tf2::Matrix3x3 m(q);
	{
		std::lock_guard<std::mutex> lock(mutex_);
		m.getRPY(imu_.roll, imu_.pitch, imu_.yaw);
		imu_.roll *= RADIAN_TO_ANGLE;
		imu_.pitch *= RADIAN_TO_ANGLE;
		imu_.yaw *= RADIAN_TO_ANGLE;
		MQTT::Mqtt_imp::get_single().SetImuIfo(imu_);
	}
}

void RosCom::BatteryStatusCallBack(const mavros_msgs::BatteryStatus::ConstPtr& msg) const
{
    BatteryStatus battery_status;
    battery_status.voltage = msg->voltage;      // V
    battery_status.current = msg->current;      // A
    battery_status.remaining = msg->remaining;  // 0..1
    // ROS_INFO("battery_status: %f V, %f A, %f", battery_status.voltage, battery_status.current, battery_status.remaining);
    MQTT::Mqtt_imp::get_single().SetBatteryStatus(battery_status);
}

void RosCom::TwistCallBack(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    ROS_INFO("Linear:x: %f", msg->twist.linear.x);
}

void RosCom::UltrasonicCallBack(const std_msgs::UInt16::ConstPtr& msg)
{
    uint16_t distance = msg->data;
    // ROS_INFO("received distance: %d mm", distance);
}

void RosCom::ModeControlCallBack(const ModeControl& send) 
{
	mavros_msgs::SetMode srv; 
    srv.request.base_mode = 0;
    ROS_INFO("-----mode");
    switch (send.mode_control)
    {
        case MODE_MANUAL:
            srv.request.custom_mode = "MANUAL";
            break;

        case MODE_ALT_HOLD:
            srv.request.custom_mode = "ALT_HOLD";
            break;
        
        default:
            break;
    }
    set_mode_client_.call(srv);
}

void RosCom::ArmedControlCallBack(const ArmedControl& send)
{
	mavros_msgs::CommandBool srv;
    
    switch (send.armed_control)
    {
        case ARMED_DISARMED:
            srv.request.value = false;
            break;

        case ARMED_ARMED:
            srv.request.value = true;
            break;
        
        default:
            break;
    }
    armed_client_.call(srv);
}

int RosCom::GetLeakageInputIO() const 
{
    return leakage_input_io_;

}

void RosCom::GetPIDParams(std::vector<PIDParams>& vec) const
{
	vec = pid_params_;
}

void RosCom::GetState(ImuInfo& imu, ModeStatus& status) const
{
	std::lock_guard<std::mutex> lock(mutex_);
	imu = imu_;
	status = mode_status_;
}

void RosCom::GetRosParam()
{
	nh_.getParam("g_surface_depth", g_surface_depth_);
    nh_.getParam("g_bottom_offset", g_bottom_offset_);
    nh_.getParam("g_front_pitch_fix", g_front_pitch_fix_);
    nh_.getParam("g_rear_pitch_fix", g_rear_pitch_fix_);
    nh_.getParam("leakage_input_io", leakage_input_io_);

	PIDParams temp;
	nh_.getParam("roll_pid_kp", temp.kp);
	nh_.getParam("roll_pid_kd", temp.kd);
	nh_.getParam("roll_pid_gradient", temp.gradient);
	nh_.getParam("roll_start_offset", temp.start_offset);
	nh_.getParam("roll_stop_offset", temp.stop_offset);
	pid_params_.emplace_back(temp);
	
	nh_.getParam("pitch_pid_kp", temp.kp);
	nh_.getParam("pitch_pid_kd", temp.kd);
	nh_.getParam("pitch_pid_gradient", temp.gradient);
	nh_.getParam("pitch_start_offset", temp.start_offset);
	nh_.getParam("pitch_stop_offset", temp.stop_offset);
	pid_params_.emplace_back(temp);
	
	nh_.getParam("yaw_pid_kp", temp.kp);
	nh_.getParam("yaw_pid_kd", temp.kd);
	nh_.getParam("yaw_pid_gradient", temp.gradient);
	nh_.getParam("yaw_start_offset", temp.start_offset);
	nh_.getParam("yaw_stop_offset", temp.stop_offset);
	pid_params_.emplace_back(temp);
	
	nh_.getParam("depth_pid_kp", temp.kp);
	nh_.getParam("depth_pid_kd", temp.kd);
	nh_.getParam("depth_pid_gradient", temp.gradient);
	nh_.getParam("depth_start_offset", temp.start_offset);
	nh_.getParam("depth_stop_offset", temp.stop_offset);
	pid_params_.emplace_back(temp);
}

bool RosCom::GetPIDSwitch()
{
	bool pid_switch = false;
	nh_.getParam("pid_switch", pid_switch);
	return pid_switch;
}

}  // namespace rov_planning