/**
 * @file motion_planner.h
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
#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <memory>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include "pd_controller.h"
#include "ros_com.h"

namespace rov_planning {
class MotionPlanner {
public:
	explicit MotionPlanner(std::shared_ptr<RosCom>&);
	~MotionPlanner() = default;
	bool Init();
	void Start();
	void Stop();
	void SetTargetValue(int, double);
	void SetManualControlValue(const RCControl&);
	
private:
	void Process();
	std::atomic<bool> closed_;
	std::atomic<bool> yaw_switch_;
	std::atomic<bool> depth_switch_;
	std::vector<std::shared_ptr<PDController>> pid_vec_;
	std::vector<PIDParams> pid_param_vec_;
	std::thread thread_;
	std::mutex mutex_;
	std::shared_ptr<RosCom> ros_com_{nullptr};
	MotionStatus final_target_{0};
	MotionStatus current_value_{0};
	bool pid_switch_{false};
	static constexpr int kPeriodTime = 200;
};
}


#endif 