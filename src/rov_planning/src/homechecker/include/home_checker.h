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
 * <tr><td>2025-9-3 <td>1.0     <td>kan tao    <td>Initialize create
 * </table>
 */
#ifndef HOME_CHECKER_H
#define HOME_CHECKER_H 

#include <mutex>
#include "planning_common.h"
#include "ros_com.h"

namespace homechecker {
class HomeChecker {
public:
    HomeChecker();
    // 用坐标判断是否到达home点
    bool IsHomeReached(const CartesianPoint& home_pos);

private:
    std::shared_ptr<rov_planning::RosCom> ros_com_;
    CartesianPoint robot_pos_;
};
}


#endif  // HOME_CHECKER_H