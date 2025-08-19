#ifndef ROS_COM_H
#define ROS_COM_H

#include <ros/ros.h>
#include <mutex>
#include <std_msgs/String.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCOut.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <sensor_msgs/BatteryState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mavros_msgs/Altitude.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/MagneticField.h>
// #include <uORB/topics/vehicle_magnetometer.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include "mqtt.h"
#include "planning_common.h"
#include "pd_controller.h"

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
    // GetState

private:
    void StateCallBack(const mavros_msgs::State::ConstPtr& msg) ;
    void RCOutCallBack(const mavros_msgs::RCOut::ConstPtr& msg) ;
    void AltitudeCallBack(const std_msgs::Float64::ConstPtr& msg) ;
    void ImuInfoCallBack(const sensor_msgs::Imu::ConstPtr& msg);
    void UltrasonicCallBack(const std_msgs::UInt16::ConstPtr& msg);
    void BatteryStateCallBack(const sensor_msgs::BatteryState::ConstPtr& msg) const;
    void TwistStampedCallBack(const geometry_msgs::TwistStamped::ConstPtr& msg) const;
    void CompassCallBack(const sensor_msgs::MagneticField::ConstPtr& msg) const;

    std::string SetChannels(const RCControl& send);

    // PD controller
    void ApplyPDController();
    void ApplyControlOutput(double depth_output, double pitch_output, double roll_output, double yaw_output);

    ros::NodeHandle nh_;

    ros::Subscriber state_sub_;
    ros::Subscriber rc_out_sub_;
    ros::Subscriber altitude_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber ultrasonic_sensor_sub_;
    ros::Subscriber battery_state_sub_;
    ros::Subscriber twist_sub_;
    ros::Subscriber compass_sub_;

    ros::ServiceClient set_mode_client_;
    ros::ServiceClient armed_client_;

    mutable std::mutex mutex_;
    ModeStatus mode_status_{0};
    Twist twist_{0};
    Compass compass{0};
    std::vector<int> rc_out_vec_;
    double g_surface_depth_= -10.0;
    double g_bottom_offset_ = 10.0;
    double g_front_pitch_fix_ = 100.0;
    double g_rear_pitch_fix_ = 100.0;
    int leakage_input_io_ = 12;
    double current_depth_ = 0.00;
 
    // IMU data
    ImuInfo current_imu_;

    constexpr static int CONTROL_TYPE = 100;
    const int MAX_DUTY = 1900;
    const int MIN_DUTY = 1100;
    const int RC_OUT_VEC_SIZE = 8;
    const double RADIAN_TO_ANGLE = 180.0 / 3.14159;

    // PD controller parameters
    bool pd_controller_enabled_; // 开关
    double target_depth_;
    double target_pitch_;
    double target_roll_;
    double target_yaw_;
    double depth_pd_output_;
    double pitch_pd_output_;
    double roll_pd_output_;
    double yaw_pd_output_;
    double pd_threshold_;   // 阈值

    // 工厂模式controller实例
    std::unique_ptr<BaseController> depth_controller_;
    std::unique_ptr<BaseController> pitch_controller_;
    std::unique_ptr<BaseController> roll_controller_;
    std::unique_ptr<BaseController> yaw_controller_;

    // last update time
    ros::Time last_update_time_;
};

}  // namespace rov_planning

#endif  // ROS_COM_H