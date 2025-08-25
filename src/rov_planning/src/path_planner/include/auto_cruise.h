/**
 * @file auto_cruise.h
 * @brief
 * @author IAT Digital
 * @version 1.0
 * @date 2025-8-18
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
 * <tr><td>2025-8-18 <td>1.0     <td>IAT Digital     <td>Initialize create
 * </table>
 */
 
#ifndef AUTO_CRUISE_H
#define AUTO_CRUISE_H


#include <atomic>
#include <chrono>
#include "thread_pool.h"
#include <fstream>
#include <sstream>
#include "ros_com.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"

namespace rov_planning {
	
class RosCom;

struct Action {
    std::string type;  // 动作类型：forward, throttle, yaw等
    int duty;          // 占空比
};


struct PathSegment {
    std::vector<Action> actions;  // 动作列表
    int time;                     // 执行时间(秒)
};

class AutoCruise {
public:
    AutoCruise(std::shared_ptr<ThreadPool>, std::shared_ptr<RosCom>&);
    ~AutoCruise() = default;  

	// load config
    void Init();
    void StartAutoCruiseThread();
    
	inline void CloseAutoCruiseThread()
	{
		closed_.store(true);
	}

private:
	void RunAutoCruise();
	int  ActionTypeToControlCode(const std::string& type);
	std::atomic<bool> closed_;
    std::shared_ptr<ThreadPool> pool_;
	std::shared_ptr<RosCom> ros_com_;
	std::vector<PathSegment> path_segments_;
	const std::string config_file_path_ = "/home/nvidia/catkin_ws2/src/rov_planning/etc/auto_path.json";
	static constexpr int kPeriodTime = 200;
};
}  // namespace rov_planning

#endif