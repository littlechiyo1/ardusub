#ifndef GPS_POSITION_H
#define GPS_POSITION_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "planning_common.h"

namespace GPS{

class GPS_Position {
public:
    GPS_Position();
    ~GPS_Position() = default;       

    // 设置位置
    void SetPosition(const double lat, const double lon, const double alt);

    // 获取当前位置
    GeoPoint GetPosition() const;
    
    // 坐标转换
    GeoPoint GetPosFromCartesian(const CartesianPoint& cart_point);
    CartesianPoint GetCartFromPos(const GeoPoint& pos);
    CartesianPoint GetRelativeCartesian(const CartesianPoint& robot_cart,  const CartesianPoint& target_cart);

    static GeoPoint current_pos_;
    GeoPoint reference_origin_;          
           

};
}

#endif // GPS_POSITION_H