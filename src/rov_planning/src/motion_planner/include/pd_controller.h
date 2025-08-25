/**
 * @file pd_controller.h
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
#ifndef PD_CONTROLLER_H
#define PD_CONTROLLER_H

#include <ros/ros.h>
#include <mutex>
#include <cmath>
#include <memory>
#include <string>
#include "planning_common.h"

namespace rov_planning {

class PDController {
public:
	PDController(){}
    ~PDController() = default;

    void UpdateParams(const PIDParams& param)
	{
		kp_ = param.kp;
		kd_ = param.kd;
		gradient_ = param.gradient;
		initialized_ = true;
	}
    double Calculate(double target, double current, double dt);
    void Reset(); 
	inline double GetGradient() const
	{
		return gradient_;
	}		
protected:
	double kp_{0.0};  // 比例系数
    double kd_{0.0};  // 微分
	double gradient_{0.0};
	// 状态
    double previous_error_{0.0};
    bool initialized_{false};
    
	const double MAX_OUTPUT = 300.0;
	const double MIN_OUTPUT = -300.0;
};

} // namespace rov_planning

#endif // PD_CONTROLLER_H