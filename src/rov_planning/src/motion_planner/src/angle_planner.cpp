/**
 * @file angel_planner.cpp
 * @brief
 * @author IAT Digital
 * @version 1.0
 * @date 2025-9-3
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
 * <tr><td>2025-9-3 <td>1.0     <td>IAT Digital     <td>Initialize create
 * </table>
 */
#include "angle_planner.h"

namespace rov_planning {

AnglePlanner::AnglePlanner(std::shared_ptr<RosCom>& ros_com,
                        std::shared_ptr<MotionPlanner>& motion_planner)
    : motion_planner_(motion_planner),
    ros_com_(ros_com) {
    // 订阅ENU位置话题
	SubscribeENUPosition();
    }

// 当前位置订阅
void AnglePlanner::SubscribeENUPosition() {
    ros::NodeHandle nh;
    enu_pos_sub_ = nh.subscribe("location/imu_odometry", 1000, &AnglePlanner::EnuPositionCallback, this);
}

// ENU位置回调函数实现
void AnglePlanner::EnuPositionCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_enu_position_.x = msg->pose.pose.position.x;
    current_enu_position_.y = msg->pose.pose.position.y;
    current_enu_position_.z = msg->pose.pose.position.z;
}

void AnglePlanner::FirstCorrention(const CartesianPoint& home_position) {
    double target_yaw = CalculateTargetDirection(home_position);
    motion_planner_->SetTargetValue(YAW_PID, target_yaw);

    turn_completed_.store(false);
    while (!turn_completed_.load()) {
        ImuInfo imu;
        ModeStatus status;
        ros_com_->GetState(imu, status);    // 当前状态

        double yaw_error = target_yaw - imu.yaw;
        ResetByLimit(yaw_error);
        // 误差大于阈值，等待
        if (fabs(yaw_error) >= yaw_tolerance) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            // 转向完成
            turn_completed_.store(true);
            ROS_INFO("Turn to home direction completed!");
            break;
        }
    }
}

void AnglePlanner::StartCorrection(const CartesianPoint& home_position) {
    double target_yaw = CalculateTargetDirection(home_position);

    ImuInfo imu;
    ModeStatus status;
    ros_com_->GetState(imu, status);    // 当前状态
    double yaw_error = target_yaw - imu.yaw;
    ResetByLimit(yaw_error);

    if (fabs(yaw_error) >= yaw_tolerance) {
        motion_planner_->SetTargetValue(YAW_PID, target_yaw);
        ROS_WARN("Angle correction completed once");
    } else {

    }
}

void AnglePlanner::StopCorrection() {
    ROS_WARN("Angle correction stopped");
    ImuInfo imu;
    ModeStatus status;
    ros_com_->GetState(imu, status);
    motion_planner_->SetTargetValue(YAW_PID, imu.yaw);
}

double AnglePlanner::CalculateTargetDirection(const CartesianPoint& target_position) {
    // 当前位置
    CartesianPoint current_pos = {current_enu_position_.x, 
                                  current_enu_position_.y, 
                                  current_enu_position_.z};
    
    // 相对坐标
    double dx = target_position.x - current_pos.x;
    double dy = target_position.y - current_pos.y;
    
    // 目标弧度
    double target_rad = atan2(dx, dy);
    // 角度
    double target_deg = target_rad * 180.0 / M_PI;
    
    ResetByLimit(target_deg);

    double compass_yaw = 0.0;
    compass_yaw = ros_com_->GetCompassData(); 
    ROS_INFO("Received compass yaw: %f", compass_yaw);  // 当前compass角度
    ResetByLimit(compass_yaw);
    ROS_INFO("ResetByLimit compass yaw: %f", compass_yaw);  // 处理后compass角度

    ImuInfo imu;
    ModeStatus status;
    ros_com_->GetState(imu, status);    // 当前imu.yaw
    ROS_INFO("Received imu yaw: %f", imu.yaw);

    double final_target_deg = target_deg + (compass_yaw - imu.yaw);

    ResetByLimit(final_target_deg);
    
    return final_target_deg;
}

}