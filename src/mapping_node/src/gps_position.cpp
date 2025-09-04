#include <gps_position.h>
#include <GeographicLib/LocalCartesian.hpp>  
#include <ros/console.h>

namespace GPS{

GeoPoint GPS::GPS_Position::current_pos_ = {0.0, 0.0, 0.0};
    // 初始化订阅者
GPS_Position::GPS_Position() {
    reference_origin_ = {0.0, 0.0, 0.0};
}

// GPS消息回调：更新位置数据


// 手动设置位置
void GPS_Position::SetPosition(const double lat, const double lon, const double alt) {
    current_pos_.lat = lat;
    current_pos_.lon = lon;
    current_pos_.alt = alt;
    reference_origin_ = current_pos_;
    ROS_INFO("Position manually updated");
}

// 获取当前位置
GeoPoint GPS_Position::GetPosition() const {
    return current_pos_;
}

GeoPoint GPS_Position::GetPosFromCartesian(const CartesianPoint& cart_point){
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

CartesianPoint GPS_Position::GetCartFromPos(const GeoPoint& pos){
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

CartesianPoint GPS_Position::GetRelativeCartesian(
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