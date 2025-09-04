/*!
 * \file imu_navigator.h
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
#ifndef IMU_NAVIGATOR_H
#define IMU_NAVIGATOR_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "planning_common.h"

namespace imu{

class IMUNavigator {
public:
    IMUNavigator();
    
    // 设置初始位置
    void SetInitialPosition(const CartesianPoint& initial_pos);

    void ProcessImuData(const ImuData& imu_data, double dt);

    // 获取当前推算位置
    RobotState GetCurrentState() const;

    // 设置IMU校准参数
    void SetCalibrationParams(
        double accel_bias_x1, double accel_bias_y1, double accel_bias_z1,
        double gyro_bias_x1, double gyro_bias_y1, double gyro_bias_z1
    );

    // GPS校准位置
    void CalibratePosition(const CartesianPoint& gps_pos);

    // 重置位移计算器
    void ResetDisplacement();

    // 获取自上次校准后的累计位移
    double GetAccumulatedDisplacement() const;

    // 判断是否达到位移阈值
    bool IsReachDisplacementThreshold() const;

    bool IsInitialized() const;
    
private:
    
    // 计算姿态
    void UpdateOrientation(double dt, double wx, double wy, double wz);
    
    // 计算速度和位置
    void UpdatePosition(double dt, double ax, double ay, double az);
    
    RobotState current_state_;    
    bool is_initialized_ = false;  // 是否完成初始化
    
    double accel_bias_x_ = 0.0;    
    double accel_bias_y_ = 0.0;   
    double accel_bias_z_ = 0.0; 
    double gyro_bias_x_ = 0.0; 
    double gyro_bias_y_ = 0.0;
    double gyro_bias_z_ = 0.0;

    CartesianPoint last_position_;  // 上一时刻位置
    double accumulated_displacement_ = 0.0;  // 累计位移
    double displacement_threshold_ = 100.0;  // 位移阈值（默认100米）
};
}

#endif  // IMU_NAVIGATOR_H
