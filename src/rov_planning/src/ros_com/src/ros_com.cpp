#include "ros_com.h"
#include <sstream>
#include "pd_controller.h"

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
    battery_state_sub_ = nh_.subscribe<sensor_msgs::BatteryState>("/mavros/battery", 1000, &RosCom::BatteryStateCallBack, this);
    twist_sub_ = nh_.subscribe("/mavros/local_position/velocity_body", 1000, &RosCom::TwistStampedCallBack, this);
    // /mavros/local_position/velocity_local
    compass_sub_ = nh_.subscribe("/mavros/imu/mag", 1000, &RosCom::CompassCallBack, this);
    nh_.getParam("g_surface_depth", g_surface_depth_);
    nh_.getParam("g_bottom_offset", g_bottom_offset_);
    nh_.getParam("g_front_pitch_fix", g_front_pitch_fix_);
    nh_.getParam("g_rear_pitch_fix", g_rear_pitch_fix_);
    nh_.getParam("leakage_input_io", leakage_input_io_);

    // PD Controller parameters
    nh_.param("pd_control_enabled", pd_controller_enabled_, false);
    nh_.param("target_depth", target_depth_, 0.0);
    nh_.param("target_pitch", target_pitch_, 0.0);
    nh_.param("target_roll", target_roll_, 0.0);
    nh_.param("target_yaw", target_yaw_, 0.0);
    nh_.param("pd_threshold", pd_threshold_, 0.05);

    // 工厂模式创建Controller实例
    depth_controller_ = ControllerFactory::CreateController("PD");
    pitch_controller_ = ControllerFactory::CreateController("PD");
    roll_controller_ = ControllerFactory::CreateController("PD");
    yaw_controller_ = ControllerFactory::CreateController("PD");

    // Init
    depth_controller_->Init(nh_);
    pitch_controller_->Init(nh_);
    roll_controller_->Init(nh_);
    yaw_controller_->Init(nh_);

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

    // last_update_time
    last_update_time_ = ros::Time::now();
}

void RosCom::ApplyPDController() {
    if (!pd_controller_enabled_ || mode_status_.motion_status != MODE_ALT_HOLD) {
        return;
    }
    
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_update_time_).toSec();
    
    if (dt <= 0.0) {
        return;
    }
    
    // 当前状态
    double current_depth, current_pitch, current_roll,  current_yaw;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_depth = current_depth_;
        current_pitch = current_imu_.pitch;
        current_roll = current_imu_.roll;
        current_yaw = current_imu_.yaw;
    }

    double depth_output = 0.0;
    double pitch_output = 0.0;
    double roll_output = 0.0;
    double yaw_output = 0.0;

    // 是否超过阈值
    double depth_error = std::abs(target_depth_ - current_depth);
    double pitch_error = std::abs(target_pitch_ - current_pitch);
    double roll_error = std::abs(target_roll_ - current_roll);
    double yaw_error = std::abs(target_yaw_ - current_yaw);

    bool depth_control_needed = depth_error >= pd_threshold_;
    bool pitch_control_needed = pitch_error >= pd_threshold_;
    bool roll_control_needed = roll_error >= pd_threshold_;
    bool yaw_control_needed = yaw_error >= pd_threshold_;

    // 超过阈值 => 执行PD控制
    // if(std::abs(target_depth_ - current_depth) >= pd_threshold_)
    if (depth_control_needed) {
        depth_output = depth_controller_->Calculate(target_depth_, current_depth, dt);
    }
    
    if (pitch_control_needed) {
        pitch_output = pitch_controller_->Calculate(target_pitch_, current_pitch, dt);
    }
    
    if (roll_control_needed) {
        roll_output = roll_controller_->Calculate(target_roll_, current_roll, dt);
    }
    
    if (yaw_control_needed) {
        yaw_output = yaw_controller_->Calculate(target_yaw_, current_yaw, dt);
    }

    // 不需要控制
    if (!depth_control_needed && !pitch_control_needed && 
        !roll_control_needed && !yaw_control_needed) {
        return;
    }
    
    // 输出值
    depth_pd_output_ = depth_output;
    pitch_pd_output_ = pitch_output;
    roll_pd_output_ = roll_output;
    yaw_pd_output_ = yaw_output;

    // if (std::abs(pitch_output) > 0.01)
    ApplyControlOutput(depth_output, pitch_output, roll_output, yaw_output);

    if (depth_control_needed) {
        ROS_INFO("PD Control: target_depth=%.2f, current_depth=%.2f, error=%.2f, output=%.2f", 
                 target_depth_, current_depth, depth_error, depth_output);
    }
    
    if (pitch_control_needed) {
        ROS_INFO("PD Control: target_pitch=%.2f, current_pitch=%.2f, error=%.2f, output=%.2f", 
                 target_pitch_, current_pitch, pitch_error, pitch_output);
    }
    
    if (roll_control_needed) {
        ROS_INFO("PD Control: target_roll=%.2f, current_roll=%.2f, error=%.2f, output=%.2f", 
                 target_roll_, current_roll, roll_error, roll_output);
    }
    
    if (yaw_control_needed) {
        ROS_INFO("PD Control: target_yaw=%.2f, current_yaw=%.2f, error=%.2f, output=%.2f", 
                 target_yaw_, current_yaw, yaw_error, yaw_output);
    }
    
    last_update_time_ = current_time;
}

void RosCom::ApplyControlOutput(double depth_output, double pitch_output, double roll_output, double yaw_output) {    
    // 深度
    RCControl depth_control;
    depth_control.rc_control_value = 1500 + depth_output * 200; // => PWM
    depth_control.rc_in = DEPTH_CONTROL;

    // 俯仰
    RCControl pitch_control;
    pitch_control.rc_control_value = 1500 + pitch_output * 200; // => PWM
    pitch_control.rc_in = PITCH_CONTROL;

    // 横滚
    RCControl roll_control;
    roll_control.rc_control_value = 1500 + roll_output * 200; // => PWM
    roll_control.rc_in = ROLL_CONTROL;
    
    // 偏航
    RCControl yaw_control;
    yaw_control.rc_control_value = 1500 + yaw_output * 200; // => PWM
    yaw_control.rc_in = YAW_CONTROL;

    // 发送控制命令
    {
        std::lock_guard<std::mutex> lock(mutex_);
        RCControlCallBack(depth_control);
        RCControlCallBack(pitch_control);
        RCControlCallBack(roll_control);
        RCControlCallBack(yaw_control);
    }
}

void RosCom::StateCallBack(const mavros_msgs::State::ConstPtr& msg) 
{
    std::lock_guard<std::mutex> lock(mutex_);
    mode_status_.armed_status = msg->armed;
    if (msg->mode == "MANUAL") {
        mode_status_.motion_status = MODE_MANUAL;
    } else if (msg->mode == "ALT_HOLD") {
        mode_status_.motion_status = MODE_ALT_HOLD;
    } else {

    }
    
    mode_status_.depth = current_depth_;
    ROS_INFO("current_depth: %f", mode_status_.depth);
    
    MQTT::Mqtt_imp::get_single().SetState(mode_status_);

}

void RosCom::RCOutCallBack(const mavros_msgs::RCOut::ConstPtr& msg) 
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0 ; i < RC_OUT_VEC_SIZE; i++) {
        rc_out_vec_[i] = msg->channels[i];
    }

   MQTT::Mqtt_imp::get_single().SetRCOut(rc_out_vec_);
}

std::string RosCom::SetChannels(const RCControl& send)
{
    int duty = static_cast<int>(send.rc_control_value);
    ROS_INFO("rc_control_value:%f,duty:%d",send.rc_control_value,duty);
    // duty = std::max(duty, MIN_DUTY);
    // duty = std::min(duty, MAX_DUTY);
    
    return std::to_string(duty);
}

void RosCom::RCControlCallBack(const RCControl& send) 
{
    mavros_msgs::SetMode srv; 
    const std::string neutral_duty = "1500";  
    if (send.rc_in == CLOSE_CONTROL) {       
        std::lock_guard<std::mutex> lock(mutex_);
        if (mode_status_.motion_status == MODE_ALT_HOLD) {
            mavros_msgs::SetMode srv; 
            srv.request.base_mode = 0;
            srv.request.custom_mode = "MANUAL";
            set_mode_client_.call(srv);
        } else {
            srv.request.custom_mode = neutral_duty;
            for (int i = FORWARD_CONTROL; i <= PITCH_CONTROL; i++) {
                srv.request.base_mode = CONTROL_TYPE + i;
                set_mode_client_.call(srv);
            }     
        }          
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

    // 应用PD控制
    ApplyPDController();
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
    ImuInfo imu;
    m.getRPY(imu.roll, imu.pitch, imu.yaw);
    imu.roll *= RADIAN_TO_ANGLE;
    imu.pitch *= RADIAN_TO_ANGLE;
    imu.yaw *= RADIAN_TO_ANGLE;

    // current_imu_
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_imu_ = imu;
    }

    MQTT::Mqtt_imp::get_single().SetImuIfo(imu);

    // 应用PD控制
    ApplyPDController();
}

void RosCom::UltrasonicCallBack(const std_msgs::UInt16::ConstPtr& msg)
{
    uint16_t distance = msg->data;
    // ROS_INFO("received distance: %d mm", distance);
}

void RosCom::BatteryStateCallBack(const sensor_msgs::BatteryState::ConstPtr& msg) const
{
    BatteryState battery_state;
    battery_state.voltage = msg->voltage;      // V
    battery_state.current = msg->current;      // A
    battery_state.remaining = msg->percentage;  // 0..1
    ROS_INFO("battery_state: %f V, %f A, %f", battery_state.voltage, battery_state.current, battery_state.remaining);
    MQTT::Mqtt_imp::get_single().SetBatteryState(battery_state);
}

void RosCom::TwistStampedCallBack(const geometry_msgs::TwistStamped::ConstPtr& msg) const
{
    Twist twist;
    twist.linear.x = msg->twist.linear.x;
    ROS_INFO("received linear x: %f", twist.linear.x);
}

void RosCom::CompassCallBack(const sensor_msgs::MagneticField::ConstPtr& msg) const
{
    Compass compass;
    compass.x = msg->magnetic_field.x;
    compass.y = msg->magnetic_field.y;
    compass.z = msg->magnetic_field.z;
    ROS_INFO("Mag: X=%f, Y=%f, Z=%f", compass.x, compass.y, compass.z); 
}

void RosCom::ModeControlCallBack(const ModeControl& send) 
{
    mavros_msgs::SetMode srv; 
    srv.request.base_mode = 0;
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

}  // namespace rov_planning