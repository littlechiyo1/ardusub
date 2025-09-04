/*!
 * \file imu_navigator.cpp
 * \brief
 * \author IAT Digital
 * \version 1.0
 * \date 2025-8-19
 *
 * \copyright Copyright (c) 2014-  阿尔特(北京)汽车数字科技有限公司
 *
 * Unpublished copyright. All rights reserved. This material contains
 * proprietary information that should be used or copied only within
 * IAT (Beijing) Automotive Digital Technology Co., Ltd., except with
 * written permission of IAT (Beijing) Automotive Digital Technology Co., Ltd.
 *
 * \par 修改日志:
 * <table>
 * <tr><th>Date      <th>Version <th>Author     <th>Description
 * <tr><td>2024-9-2 <td>1.0     <td>kan tao    <td>Initialize create
 * </table>
 */
#ifndef LOCATION_NODE_H
#define LOCATION_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <memory>
#include <atomic>
#include <geometry_msgs/TwistStamped.h>
#include "geofence_checker.h"
#include "gps_postion.h"  
#include "imu_navigator.h"
#include "planning_common.h"

namespace location_node {

class LocationNode {
public:
    explicit LocationNode(ros::NodeHandle& nh);

    ~LocationNode() = default;

private:
    void PublishImuOdometry(const ros::TimerEvent& event);

    void GpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void GpsVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

    enum class GpsStatus {
        VALID,    
        INVALID  
    };
    GpsStatus gps_status_ = GpsStatus::INVALID;

    ros::NodeHandle nh_;

    ros::Publisher imu_odom_pub_;

    
    double gps_body_vel_x_ = 0.0;
    double gps_body_vel_y_ = 0.0;
    double gps_body_vel_z_ = 0.0;

    ros::Subscriber gps_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber gps_vel_sub_;

    ros::Timer imu_timer_;
    ros::Time last_valid_gps_time_;
    const double GPS_TIMEOUT = 1.0;

    double publish_rate_;

    CartesianPoint current_cart_ = {0.0};


    // 围栏文件路径
    std::string fence_file_;

    std::atomic<bool> origin_set_{false};

    std::unique_ptr<gps::GPSPosition> gps_pos_;
    std::unique_ptr<imu::IMUNavigator> imu_nav_;
    std::unique_ptr<fence::GeofenceChecker> fence_checker_;
};

}  // namespace location_node

#endif  // LOCATION_NODE_H