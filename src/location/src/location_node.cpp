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
#include "location_node.h"
#include <ros/console.h>
#include <memory> 
#include <nav_msgs/Odometry.h>

namespace location_node {

LocationNode::LocationNode(ros::NodeHandle& nh) : nh_(nh) {
    nh_.param<double>("publish_rate", publish_rate_, 10.0);
    //nh_.param<std::string>("fence_file", fence_file_, "geofence.txt");

    gps_pos_ = std::make_unique<gps::GPSPosition>();
    imu_nav_ = std::make_unique<imu::IMUNavigator>();
    //fence_checker_ = std::make_unique<fence::GeofenceChecker>(fence_file_);
    
    gps_sub_ = nh_.subscribe("/mavros/global_position/global", 1000,  &LocationNode::GpsCallback, this);
    imu_sub_ = nh_.subscribe("/mavros/imu/data", 1000, &LocationNode::ImuCallback, this);
    gps_vel_sub_ = nh_.subscribe("/mavros/local_position/velocity_body", 1000, &LocationNode::GpsVelCallback, this);
 
    imu_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("location/imu_odometry", 10);  

    imu_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), &LocationNode::PublishImuOdometry, this);

    ROS_INFO("Location node initialized. Publish rate: %.1f Hz", publish_rate_);
}

void LocationNode::PublishImuOdometry(const ros::TimerEvent &event){
    if (!origin_set_.load()) {
        //ROS_WARN_THROTTLE(1, "Origin not set, skip publishing");
        return;
    }


    nav_msgs::Odometry imu_odom;
    imu_odom.header.stamp = event.current_real;
    imu_odom.header.frame_id = "imu";

    RobotState imu_state = imu_nav_->GetCurrentState();

    imu_odom.pose.pose.position.x = imu_state.position.x;
    imu_odom.pose.pose.position.y = imu_state.position.y;
    imu_odom.pose.pose.position.z = imu_state.position.z;

    imu_odom.pose.pose.orientation.x = imu_state.orientation.x();
    imu_odom.pose.pose.orientation.y = imu_state.orientation.y();
    imu_odom.pose.pose.orientation.z = imu_state.orientation.z();
    imu_odom.pose.pose.orientation.w = imu_state.orientation.w();

    if (gps_status_ == GpsStatus::VALID) {
        imu_odom.twist.twist.linear.x = gps_body_vel_x_;
        imu_odom.twist.twist.linear.y = gps_body_vel_y_;
        imu_odom.twist.twist.linear.z = gps_body_vel_z_;
    } else {
        ROS_INFO("-----GPS not Valid");        
        imu_odom.twist.twist.linear.x = imu_state.vel_x;
        imu_odom.twist.twist.linear.y = imu_state.vel_y;
        imu_odom.twist.twist.linear.z = imu_state.vel_z;
    }

    imu_odom.child_frame_id = (gps_status_ == GpsStatus::VALID) ? "imu_with_gps" : "imu_only";
    //ROS_INFO("imu_odom.twist.twist.linear.x:%f,y:%f,z:%f",imu_odom.twist.twist.linear.x,imu_odom.twist.twist.linear.y,imu_odom.twist.twist.linear.z);
    imu_odom_pub_.publish(imu_odom);
}

void LocationNode::GpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    if(msg->status.status == 0) {
            gps_status_ = GpsStatus::VALID;
            last_valid_gps_time_ = msg->header.stamp;

            GeoPoint geo_pos_;
            geo_pos_.lat = msg->latitude;
            geo_pos_.lon = msg->longitude;
            geo_pos_.alt = msg->altitude;

            if(origin_set_.load()) {
                current_cart_ = gps_pos_->GetCartFromPos(geo_pos_);
                if (imu_nav_->IsInitialized()) {  
                    imu_nav_->CalibratePosition(current_cart_);
                }
                return;
            }
            else {
                gps_pos_->SetOrigin(geo_pos_.lat, geo_pos_.lon, geo_pos_.alt);
                origin_set_.store(true);
                CartesianPoint cart_pos_ = gps_pos_->GetCartFromPos(geo_pos_);
                imu_nav_->SetInitialPosition(cart_pos_);
                ROS_INFO("Origin set to first valid GPS: lat=%.6f, lon=%.6f, alt=%.2f",
                        geo_pos_.lat, geo_pos_.lon, geo_pos_.alt);
            }
    }
    else{
        //ROS_INFO("Invaild GPS data");
    }
}

void LocationNode::ImuCallback(const sensor_msgs::Imu::ConstPtr &msg){
    if (!origin_set_.load()) {
        //ROS_WARN_THROTTLE(1, "Origin not set, skip IMU processing");
        return;
    }

    static ros::Time last_imu_time;
    double dt = (msg->header.stamp - last_imu_time).toSec();
    last_imu_time = msg->header.stamp;

    ImuData imu_data;
    imu_data.gyro_x = msg->angular_velocity.x;  // 角速度
    imu_data.gyro_y = msg->angular_velocity.y;
    imu_data.gyro_z = msg->angular_velocity.z;
    imu_data.acc_x = msg->linear_acceleration.x; // 线加速度
    imu_data.acc_y = msg->linear_acceleration.y;
    imu_data.acc_z = msg->linear_acceleration.z;

    if (gps_status_ == GpsStatus::VALID && 
        (msg->header.stamp - last_valid_gps_time_).toSec() > GPS_TIMEOUT) {
        gps_status_ = GpsStatus::INVALID;
        ROS_WARN("GPS timeout, switch to IMU-only mode");
    }

    imu_nav_->ProcessImuData(imu_data, dt);
}

void LocationNode::GpsVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg){
        if (gps_status_ == GpsStatus::VALID) {  
            gps_body_vel_x_ = msg->twist.linear.x;
            gps_body_vel_y_ = msg->twist.linear.y;
            gps_body_vel_z_ = msg->twist.linear.z;
    }
}

}  // namespace location_node

