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
 * <tr><td>2024-8-19 <td>1.0     <td>kan tao    <td>Initialize create
 * </table>
 */
#include "imu_navigator.h"
#include <ros/console.h>

namespace imu{
IMUNavigator::IMUNavigator() {

}

void IMUNavigator::SetInitialPosition(const CartesianPoint& initial_pos) {
    if(!is_initialized_){
    current_state_.position = initial_pos;
    current_state_.orientation.setRPY(0, 0, 0);

    current_state_.vel_east = 0.0;
    current_state_.vel_north = 0.0;
    current_state_.vel_up = 0.0;
    is_initialized_ = true;
    ROS_INFO("Initial position set: x=%.2f, y=%.2f, z=%.2f",
             initial_pos.x, initial_pos.y, initial_pos.z);
    }
    else{
    ROS_ERROR("Initial position already set.");
    }
}

void IMUNavigator::ProcessImuData(const ImuData& imu_data, double dt){
    if (!is_initialized_) {
        //ROS_WARN_THROTTLE(1, "未初始化，请先设置初始位置！");
        return;
    }
    
    if (dt <= 0 || dt > 0.1) {
        ROS_WARN("无效的时间差: %.6f s", dt);
        return;
    }
    
    double wx = imu_data.gyro_x - gyro_bias_x_;
    double wy = imu_data.gyro_y - gyro_bias_y_;
    double wz = imu_data.gyro_z - gyro_bias_z_;
    
    double ax = imu_data.acc_x - accel_bias_x_;
    double ay = imu_data.acc_y - accel_bias_y_;
    double az = imu_data.acc_z - accel_bias_z_;
    
    UpdateOrientation(dt, wx, wy, wz);
    UpdatePosition(dt, ax, ay, az);
}

RobotState IMUNavigator::GetCurrentState() const {
    return current_state_;
}

void IMUNavigator::SetCalibrationParams(
        double accel_bias_x1, double accel_bias_y1, double accel_bias_z1,
        double gyro_bias_x1, double gyro_bias_y1, double gyro_bias_z1
    ){
            accel_bias_x_ = accel_bias_x1;
            accel_bias_y_ = accel_bias_y1;
            accel_bias_z_ = accel_bias_z1;
            gyro_bias_x_ = gyro_bias_x1;
            gyro_bias_y_ = gyro_bias_y1;
            gyro_bias_z_ = gyro_bias_z1;
    }

 void IMUNavigator::CalibratePosition(const CartesianPoint& gps_pos){
    current_state_.position = gps_pos;
 }

 void IMUNavigator::ResetDisplacement(){
    accumulated_displacement_ = 0.0;
 }

 double IMUNavigator::GetAccumulatedDisplacement() const {
    return accumulated_displacement_;
}

bool IMUNavigator::IsReachDisplacementThreshold() const{
    return accumulated_displacement_ >= displacement_threshold_;
}

bool IMUNavigator::IsInitialized() const{
    return is_initialized_;
}

void IMUNavigator::UpdateOrientation(double dt, double wx, double wy, double wz) {
    // 计算旋转增量
    double rx = wx * dt;  // X轴旋转增量（横滚）
    double ry = wy * dt;  // Y轴旋转增量（俯仰）
    double rz = wz * dt;  // Z轴旋转增量（偏航）
    
    // 创建旋转增量四元数
    tf2::Quaternion delta_rot;
    delta_rot.setRPY(rx, ry, rz);  // 从欧拉角创建四元数
    
    // 更新姿态（当前姿态 * 旋转增量）
    current_state_.orientation *= delta_rot;
    current_state_.orientation.normalize();  // 归一化四元数（避免漂移）
}

void IMUNavigator::UpdatePosition(double dt, double ax, double ay, double az) {
    // 将加速度从机体坐标系转换到大地坐标系（ENU）
    // 1. 获取当前姿态的旋转矩阵
    tf2::Matrix3x3 rot_mat(current_state_.orientation);
    // 2. 机体坐标系加速度 → 大地坐标系加速度
    tf2::Vector3 acc_body(ax, ay, az);
    tf2::Vector3 accel_enu = rot_mat * acc_body;
    
    // 3. 去除重力影响（水下机器人Z轴向上为正，重力加速度为-9.81）
    accel_enu.setZ(accel_enu.z() - 9.81);
    
    // 4. 积分计算速度（速度 = 上一时刻速度 + 加速度 * 时间）
    current_state_.vel_east += accel_enu.x() * dt;
    current_state_.vel_north += accel_enu.y() * dt;
    current_state_.vel_up += accel_enu.z() * dt;

    // 旋转矩阵的转置 = 逆矩阵（正交矩阵性质），用于ENU→机体的转换
    tf2::Matrix3x3 rot_mat_inv = rot_mat.transpose();

    // 构造ENU速度向量
    tf2::Vector3 vel_enu(current_state_.vel_east, current_state_.vel_north, current_state_.vel_up);
    // 转换为机体坐标系速度
    tf2::Vector3 vel_body = rot_mat_inv * vel_enu;

    // 存储到current_state_中
    current_state_.vel_x = vel_body.x();  // 机体x轴（向前）速度
    current_state_.vel_y = vel_body.y();  // 机体y轴（向左）速度
    current_state_.vel_z = vel_body.z();  // 机体z轴（向上）速度
    
    CartesianPoint prev_position = current_state_.position;

    // 5. 积分计算位置（位置 = 上一时刻位置 + 速度 * 时间）
    current_state_.position.x += current_state_.vel_east * dt;
    current_state_.position.y += current_state_.vel_north * dt;
    current_state_.position.z += current_state_.vel_up * dt;

    // 计算本次位置更新的位移增量（三维空间中的距离）
    double dx = current_state_.position.x - prev_position.x;
    double dy = current_state_.position.y - prev_position.y;
    double dz = current_state_.position.z - prev_position.z;
    double delta_displacement = sqrt(dx*dx + dy*dy + dz*dz);

    // 累加位移（仅统计上次校准后的位移）
    accumulated_displacement_ += delta_displacement;

    if(accumulated_displacement_ >= displacement_threshold_){
        // 上浮
        // 重新获取gps定位
        // 清零accumulated_displacement_

    }
}
}
