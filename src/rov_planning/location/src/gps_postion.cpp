/*!
 * \file gps_position.cpp
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
#include "gps_postion.h"
#include <GeographicLib/LocalCartesian.hpp>  
#include <ros/console.h>

namespace gps{

    // 初始化订阅者
GPSPosition::GPSPosition() {
    
}

// GPS消息回调：更新位置数据


// 手动设置位置
void GPSPosition::Setposition(const double lat, const double lon, const double alt) {
    current_pos_.lat = lat;
    current_pos_.lon = lon;
    current_pos_.alt = alt;
    ROS_INFO("Position manually updated");
}

void GPSPosition::SetOrigin(const double lat, const double lon, const double alt){
    if(!is_origin_set_){
        reference_origin_.lat = lat;
        reference_origin_.lon = lon;
        reference_origin_.alt = alt;
        is_origin_set_ = true;
        ROS_INFO("Origin set");
    }
    else{
        ROS_INFO("Origin already set");
    }
}

// 获取当前位置
GeoPoint GPSPosition::Getposition() const {
    return current_pos_;
}

GeoPoint GPSPosition::GetPosFromCartesian(const CartesianPoint& cart_point){
    GeoPoint geo_point;
    geo_point.alt = 0;
    geo_point.lat = 0;
    geo_point.lon = 0;

    try {
        // 使用相同的参考原点进行反向转换
        GeographicLib::LocalCartesian proj;
        proj.Reset(reference_origin_.lat, reference_origin_.lon, reference_origin_.alt);
        
        // 将ENU坐标转换为经纬度
        proj.Reverse(cart_point.x, cart_point.y, cart_point.z, geo_point.lat, geo_point.lon, geo_point.alt);
        return geo_point;
    } catch (const std::exception& e) {
        ROS_ERROR("Reverse coordinate conversion failed: %s", e.what());
        return geo_point;
    }
}

CartesianPoint GPSPosition::GetCartFromPos(const GeoPoint& pos){
    CartesianPoint cart_point;
    cart_point.x = 0;
    cart_point.y = 0;
    cart_point.z = 0;

    try {
        GeographicLib::LocalCartesian proj;

        proj.Reset(reference_origin_.lat, reference_origin_.lon, reference_origin_.alt);
        
        proj.Forward(pos.lat, pos.lon, pos.alt, cart_point.x, cart_point.y, cart_point.z);
        return cart_point;
    } catch (const std::exception& e) {
        ROS_ERROR("Coordinate conversion failed: %s", e.what());
        return cart_point;
    }
}

CartesianPoint GPSPosition::GetRelativeCartesian(
    const CartesianPoint& robot_cart, 
    const CartesianPoint& target_cart) {
    
    CartesianPoint relative_cart;
    
    try {
        relative_cart.x = target_cart.x - robot_cart.x;  
        relative_cart.y = target_cart.y - robot_cart.y;  
        relative_cart.z = target_cart.z - robot_cart.z;  
        
        return relative_cart;
    } catch (const std::exception& e) {
        ROS_ERROR("相对坐标计算失败：%s", e.what());
        relative_cart.x = 0.0;
        relative_cart.y = 0.0;
        relative_cart.z = 0.0;
        return relative_cart;
    }
}

}