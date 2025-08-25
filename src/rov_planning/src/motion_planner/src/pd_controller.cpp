/**
 * @file pd_controller.cpp
 * @brief
 * @author IAT Digital
 * @version 1.0
 * @date 2025-8-19
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
 * <tr><td>2025-8-19 <td>1.0     <td>IAT Digital     <td>Initialize create
 * </table>
 */
#include "pd_controller.h"

namespace rov_planning {



double PDController::Calculate(double target, double current, double dt) {
	
    if (!initialized_) {
        ROS_WARN("PD Controller not initialized!");
        return 0.0;
    }

    
    // 误差
    double error = target - current;
    
    // 微分
    double derivative = 0.0;
    if (dt > 0.0) {
        derivative = (error - previous_error_) / dt;
    }
    
    // 输出
    double output = kp_ * error + kd_ * derivative;
    
    if (output > MAX_OUTPUT) {
        output = MAX_OUTPUT;
    } else if (output < MIN_OUTPUT) {
        output = MIN_OUTPUT;
    } else {
	}
    
    // 保存当前误差 => 下次计算微分项
    previous_error_ = error;
    
    return output;
}



void PDController::Reset() {
    previous_error_ = 0.0;
}



} // namespace rov_planning