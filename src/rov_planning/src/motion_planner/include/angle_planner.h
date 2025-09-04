/**
 * @file angle_planner.h
 * @brief
 * @author IAT Digital
 * @version 1.0
 * @date 2025-9-3
 *
 * @copyright Copyright (c) 2025-  阿尔特汽车技术股份有限公司
 *
 * Unpublished copyright. All rights reserved. This material contains
 * proprietary information that should be used or copied only within
 * IAT, except with written permission of IAT.
 *
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2025-9-3 <td>1.0     <td>IAT Digital     <td>Initialize create
 * </table>
 */
#ifndef ANGLE_PLANNER_H
#define ANGLE_PLANNER_H 

#include <memory>
#include <atomic>
#include <mutex>
#include <cmath>
#include "motion_planner.h"
#include "pd_controller.h"
#include "ros_com.h"

namespace rov_planning {

class AnglePlanner {
public:
    explicit AnglePlanner(std::shared_ptr<RosCom>& ros_com,
                          std::shared_ptr<MotionPlanner>& motion_planner);
    ~AnglePlanner() = default;

    // 转向接口
    void FirstCorrention(const CartesianPoint& home_position);
    // 纠正接口
    void StartCorrection(const CartesianPoint& home_position);
    void StopCorrection();

    // 朝向home点的目标角度
    double CalculateTargetDirection(const CartesianPoint& home_position);

private:
    void SubscribeENUPosition();      // 订阅location/enu_position话题
	void EnuPositionCallback(const nav_msgs::Odometry::ConstPtr& msg); // ENU位置回调函数
    static inline void ResetByLimit(double& in)
    {
        const double lim1 = 180.0;
        const double lim2 = lim1 * 2.0;
        if (in > lim1) {
            in -= lim2;
        } else if (in < -lim1) {
            in += lim2;
        } else {

        }
    }

    std::shared_ptr<RosCom> ros_com_;
    std::shared_ptr<MotionPlanner> motion_planner_;

    // CartesianPoint home_position_{0.0, 0.0, 0.0};
    CartesianPoint current_enu_position_{0.0, 0.0, 0.0};	// 当前ENU坐标
    const double yaw_tolerance = 5.0;                   // 转向完成阈值
	ros::Subscriber enu_pos_sub_;							// ENU位置订阅者
    std::atomic<bool> turn_completed_{false};
};
}

#endif