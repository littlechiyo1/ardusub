#ifndef ROS_COM_H
#define ROS_COM_H

#include <ros/ros.h>
#include <mutex>
#include <std_msgs/String.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCOut.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mavros_msgs/Altitude.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include "mqtt.h"
#include "common.h"

namespace rov_planning {

class RosCom {
public:
    RosCom();
    ~RosCom() = default;

    void Init();
    void RCControlCallBack(const RCControl&);
    void ModeControlCallBack(const ModeControl&);
    void ArmedControlCallBack(const ArmedControl&);

private:
    void StateCallBack(const mavros_msgs::State::ConstPtr& msg) ;
    void RCOutCallBack(const mavros_msgs::RCOut::ConstPtr& msg) ;
    void AltitudeCallBack(const std_msgs::Float64::ConstPtr& msg) const;
    void ImuInfoCallBack(const sensor_msgs::Imu::ConstPtr& msg) const;
    void UltrasonicCallBack(const std_msgs::UInt16::ConstPtr& msg);

    std::string SetChannels(const RCControl& send);

    ros::NodeHandle nh_;

    ros::Subscriber state_sub_;
    ros::Subscriber rc_out_sub_;
    ros::Subscriber altitude_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber ultrasonic_sub_;

    ros::ServiceClient set_mode_client_;
    ros::ServiceClient armed_client_;

    std::mutex mutex_;
    ModeStatus mode_status_{0};
    std::vector<int> rc_out_vec_;
    double g_surface_depth_= -10.0;
    double g_bottom_offset_ = 10.0;
    double g_front_pitch_fix_ = 100.0;
    double g_rear_pitch_fix_ = 100.0;
 

    constexpr static int CONTROL_TYPE = 100;
    const int MAX_DUTY = 1900;
    const int MIN_DUTY = 1100;
    const int RC_OUT_VEC_SIZE = 6;
    const double RADIAN_TO_ANGLE = 180.0 / 3.14159;
};

}  // namespace rov_planning

#endif  // ROS_COM_H