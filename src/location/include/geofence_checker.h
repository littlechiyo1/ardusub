/*!
 * \file geofence_checker.h
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
#ifndef ROV_NAVIGATION_GEOFENCE_CHECKER_H
#define ROV_NAVIGATION_GEOFENCE_CHECKER_H

#include <tf2/LinearMath/Vector3.h>
#include <vector>
#include <string>
#include "planning_common.h"

namespace fence {
class GeofenceChecker {

public:
    // 构造函数：从文件加载地理围栏
    explicit GeofenceChecker(const std::string& fence_file_path);

    // 检查机器人是否在围栏内
    bool IsInsideFence(const CartesianPoint& robot_pos);

    // 检查围栏是否有效加载
    bool IsFenceValid();

private:
    std::vector<tf2::Vector3> fence_points_;  // 地理围栏多边形的点
    bool is_valid_ = false;                    // 围栏是否有效
    // 从文件加载围栏点
    bool LoadGeofenceFromFile(const std::string& file_path);
};

}  // namespace fence

#endif  // ROV_NAVIGATION_GEOFENCE_CHECKER_H
    