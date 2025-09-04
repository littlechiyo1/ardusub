/*!
 * \file gps_position.h
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
#ifndef GPS_POSITION_H
#define GPS_POSITION_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "planning_common.h"

namespace gps{

class GPSPosition {
public:
    GPSPosition();
    ~GPSPosition() = default;       

    // 设置位置
    void Setposition(const double lat, const double lon, const double alt);

    // 设置原点
    void SetOrigin(const double lat, const double lon, const double alt);

    // 获取当前位置
    GeoPoint Getposition() const;
    
    // 坐标转换
    GeoPoint GetPosFromCartesian(const CartesianPoint& cart_point);
    CartesianPoint GetCartFromPos(const GeoPoint& pos);
    CartesianPoint GetRelativeCartesian(const CartesianPoint& robot_cart,  const CartesianPoint& target_cart);

    GeoPoint current_pos_;

private:       
    GeoPoint reference_origin_;
    bool is_origin_set_ = false;               

};
}

#endif // GPS_POSITION_H