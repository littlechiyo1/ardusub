/*!
 * \file common.h
 * \brief
 * \author IAT
 * \version 1.0
 * \date 2024-12-26
 *
 * \copyright Copyright (c) 2014-2024  阿尔特(北京)汽车数字科技有限公司
 *
 * Unpublished copyright. All rights reserved. This material contains
 * proprietary information that should be used or copied only within
 * IAT (Beijing) Automotive Digital Technology Co., Ltd., except with
 * written permission of IAT (Beijing) Automotive Digital Technology Co., Ltd.
 *
 * \par 修改日志:
 * <table>
 * <tr><th>Date      <th>Version <th>Author     <th>Description
 * <tr><td>2024-12-26 <td>1.0     <td>IAT       <td>Initialize create
 * </table>
 */

#ifndef COMMON_H
#define COMMON_H
#include <cstdint>

typedef struct {
    double pitch;
    double roll;
    double yaw;
} ImuInfo;

typedef struct {
    double rc_control_value;
    int rc_in;
} RCControl;

typedef struct {
    int mode_control;
} ModeControl;

typedef struct {
    int armed_control;
} ArmedControl;

typedef struct {
    int motion_status;
    int armed_status;
    double depth;
} ModeStatus;

// 地理坐标(经纬度)
typedef struct {
    double lat;
    double lon;
    double alt;
} GeoPoint;

//大地坐标
typedef struct {
    double x;
    double y;
    double z;
} CartesianPoint;

//IMU数据
typedef struct {
    double acc_x;    //x轴加速度
    double acc_y;    //y轴加速度
    double acc_z;    //z轴加速度
    double gyro_x;   //x轴角速度
    double gyro_y;   //y轴角速度
    double gyro_z;   //z轴角速度
} ImuData;

//轨迹控制
typedef struct {
    GeoPoint geo_point;
    CartesianPoint cartesian_point;
} Waypoint;

typedef enum {
    FORWARD_CONTROL = 1,
    YAW_CONTROL,
    THROTTLE_CONTROL,
    LATERAL_CONTROL,
    ROLL_CONTROL,
    PITCH_CONTROL,
    CLOSE_CONTROL,
    LIGHT_CONTROL,
    SURFACE_DEPTH = 11,
    BOTTOM_OFFSET,
    FRONT_PITCH_FIX,
    REAR_PITCH_FIX
} ControlTypeEnum;

typedef enum {
    MODE_MANUAL = 0,
    MODE_ALT_HOLD = 1
} ModeControlEnum;
typedef enum {
    ARMED_DISARMED = 0,
    ARMED_ARMED = 1
} ArmedControlEnum;

typedef enum {
    INTERNAL_MODE_MANUAL = 19,
    INTERNAL_MODE_ALT_HOLD = 2
} InternalModeControlEnum;



#endif  