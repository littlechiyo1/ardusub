/*!
 * \file common.h
 * \brief
 * \author IAT
 * \version 1.0
 * \date 2024-12-26
 *
 * \copyright Copyright (c) 2014-2024  阿尔�?北京)汽车数字科技有限公司
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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

typedef struct {
    double pitch;
    double roll;
    double yaw;
} ImuInfo;

typedef struct {
	ImuInfo imu_info;
	double depth;
	double speed;
} MotionStatus;

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

typedef struct {
    double voltage;
    double current;
    double remaining;
} BatteryStatus;


typedef struct {
    struct {
        double x, y, z;
    } linear;  // 线速度分量

    struct {
        double x, y, z;
    } angular; // 角速度分量
} Twist;

typedef struct {
	double kp;
	double ki;
	double kd;
	double gradient;
	double start_offset;
	double stop_offset;
} PIDParams;

typedef struct{
    float gather_total_voltage;     /// 采集总压 (V)
    float cumulative_total_voltag;  // 累计电压
    float current;   
    float state_of_charge;   ///< 剩余电量 (SOC %)
    bool has_fault;          
    uint32_t fault_bits[2];  ///< 故障位码, fault_bits[0]为Byte0-3, fault_bits[1]为Byte4-7
}BmsBasicInfo;

// 地理坐标(经纬�?
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

typedef struct  {
	CartesianPoint position;       
    tf2::Quaternion orientation;   // 姿态（四元数）
    double vel_east = 0.0;            
    double vel_north = 0.0;  
    double vel_up = 0.0;  
	
    double vel_x = 0.0;            
    double vel_y = 0.0;  
    double vel_z = 0.0;          
} RobotState;

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
	ROLL_PID = 0,
	PITCH_PID,
	YAW_PID,
	DEPTH_PID,
	SPEED_PID
} PidTypeEnum;

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