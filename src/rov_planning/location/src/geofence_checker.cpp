/*!
 * \file geofence_checker.cpp
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
#include "geofence_checker.h"
#include <fstream>
#include <sstream>
#include <ros/ros.h>
#include <tf2/LinearMath/Vector3.h>

namespace fence {

GeofenceChecker::GeofenceChecker(const std::string& fence_file_path) {
    if (!LoadGeofenceFromFile(fence_file_path)) {
        ROS_ERROR("Failed to initialize geofence checker!");
        is_valid_ = false;
    } else {
        is_valid_ = true;
        ROS_INFO("Geofence loaded successfully with %zu points", fence_points_.size());
    }
}

bool GeofenceChecker::LoadGeofenceFromFile(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        ROS_ERROR("Cannot open geofence file: %s", file_path.c_str());
        return false;
    }

    fence_points_.clear();
    std::string line;
    int line_num = 0;

    // 读取文件中的点（格式：纬度 经度 或 x y）
    while (std::getline(file, line)) {
        line_num++;
        // 跳过空行和注释
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        double x, y;
        if (!(iss >> x >> y)) {
            ROS_WARN("Invalid format in line %d of geofence file", line_num);
            continue;
        }

        // 存储为3D点（忽略高度，地理围栏通常是2D区域）
        fence_points_.emplace_back(x, y, 0.0);
    }

    file.close();

    // 检查围栏是否有效（至少需要3个点才能形成多边形）
    if (fence_points_.size() < 3) {
        ROS_ERROR("Geofence requires at least 3 points, got %zu", fence_points_.size());
        return false;
    }

    // 确保围栏是闭合的（首尾点相同）
    if (fence_points_.front() != fence_points_.back()) {
        fence_points_.push_back(fence_points_.front());
        ROS_INFO("Added closing point to geofence");
    }

    return true;
}

bool GeofenceChecker::IsInsideFence(const CartesianPoint& robot_pos) {
    if (!is_valid_) return false;

    // 将机器人3D位置转换为2D点（使用x和y坐标）
    tf2::Vector3 robot_point(robot_pos.x, robot_pos.y, 0.0);
    int crossings = 0;

    // 点-in-多边形算法（射线法）
    for (size_t i = 0; i < fence_points_.size() - 1; ++i) {
        tf2::Vector3 p1 = fence_points_[i];
        tf2::Vector3 p2 = fence_points_[i + 1];

        // 检查射线是否与边相交
        if (((p1.y() > robot_point.y()) != (p2.y() > robot_point.y())) &&
            (robot_point.x() < p1.x() + (p2.x() - p1.x()) * (robot_point.y() - p1.y()) / (p2.y() - p1.y()))) {
            crossings++;
        }
    }

    // 交叉次数为奇数则在多边形内
    return (crossings % 2) == 1;
}

bool GeofenceChecker::IsFenceValid(){ 
    return is_valid_; 
}

}  // namespace fence
    