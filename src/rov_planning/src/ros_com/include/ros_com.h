#ifndef ROS_COM_H
#define ROS_COM_H

#include <ros/ros.h>
#include <mutex>
#include <std_msgs/String.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCOut.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/BatteryStatus.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mavros_msgs/Altitude.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <nav_msgs/Odometry.h>
#include "mqtt.h"
#include "planning_common.h"

namespace rov_planning {

class RosCom {
public:
    RosCom();
    ~RosCom() = default;

    void Init();
    void RCControlCallBack(const RCControl&);
    void ModeControlCallBack(const ModeControl&);
    void ArmedControlCallBack(const ArmedControl&);
    int GetLeakageInputIO() const;
	void GetPIDParams(std::vector<PIDParams>&) const;
	void GetState(ImuInfo&, ModeStatus&) const;
	void GetRobotState(RobotState&) const;
	bool GetPIDSwitch();
	void GetCurrentPos(CartesianPoint&) const;
 double GetCompassData() const;

private:
    void StateCallBack(const mavros_msgs::State::ConstPtr& msg) ;
    void RCOutCallBack(const mavros_msgs::RCOut::ConstPtr& msg) ;
    void AltitudeCallBack(const std_msgs::Float64::ConstPtr& msg) ;
    void ImuInfoCallBack(const sensor_msgs::Imu::ConstPtr& msg) ;
    void UltrasonicCallBack(const std_msgs::UInt16::ConstPtr& msg);
    void BatteryStatusCallBack(const mavros_msgs::BatteryStatus::ConstPtr& msg) const;
    void TwistCallBack(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void CompassHdgCallBack(const std_msgs::Float64::ConstPtr& msg);
	void ImuOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
	
	void GetRosParam();

    std::string SetChannels(const RCControl& send);

    ros::NodeHandle nh_;

    ros::Subscriber state_sub_;
    ros::Subscriber rc_out_sub_;
    ros::Subscriber altitude_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber ultrasonic_sensor_sub_;
    ros::Subscriber battery_status_sub_;
    ros::Subscriber twist_sub_;
	ros::Subscriber compass_hdg_sub_;
	ros::Subscriber imu_odom_sub_;

    ros::ServiceClient set_mode_client_;
    ros::ServiceClient armed_client_;
	
	

    mutable std::mutex mutex_;
	std::vector<PIDParams> pid_params_;
    ModeStatus mode_status_{0};
    Twist twist_{0};
	ImuInfo imu_{0};
    std::vector<int> rc_out_vec_;
	
    double g_surface_depth_= -10.0;
    double g_bottom_offset_ = 10.0;
    double g_front_pitch_fix_ = 100.0;
    double g_rear_pitch_fix_ = 100.0;
    int leakage_input_io_ = 12;
    double current_depth_ = 0.00;
    double compass_data_ = 0.00;
	RobotState current_state_;
	
	CartesianPoint current_pos_;
    mutable std::mutex pos_mutex_;
 

    constexpr static int CONTROL_TYPE = 100;
    const int MAX_DUTY = 1900;
    const int MIN_DUTY = 1100;
    const int RC_OUT_VEC_SIZE = 8;
    const double RADIAN_TO_ANGLE = 180.0 / 3.14159;
};

}  // namespace rov_planning

#endif  // ROS_COM_H