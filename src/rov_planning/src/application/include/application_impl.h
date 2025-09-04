/*!
 * \file application_impl.h
 * \brief
 * \author IAT Digital
 * \version 1.0
 * \date 2023-12-25
 *
 * \copyright Copyright (c) 2014-2023  闃垮皵鐗?鍖椾含)姹借溅鏁板瓧绉戞妧鏈夐檺鍏徃
 *
 * Unpublished copyright. All rights reserved. This material contains
 * proprietary information that should be used or copied only within
 * IAT (Beijing) Automotive Digital Technology Co., Ltd., except with
 * written permission of IAT (Beijing) Automotive Digital Technology Co., Ltd.
 *
 * \par 淇敼鏃ュ織:
 * <table>
 * <tr><th>Date      <th>Version <th>Author       <th>Description
 * <tr><td>2024-1-15 <td>1.0     <td>kan tao      <td>Initialize create
 * </table>
 */
#ifndef _ADAPTIVEAPP_IMP_H_INCLUDE
#define _ADAPTIVEAPP_IMP_H_INCLUDE

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>
#include "std_msgs/String.h"       
#include <sstream>

#include "application_base.h"
#include "mqtt.h"
#include "ros_com.h"
#include "leakage_parser.h"
#include "auto_cruise.h"
#include "motion_planner.h"
#include "bms_reader.h"
#include "state_machine.h"
#include "sethome.h"//
#include "gps_postion.h"   // 新增：包含GPSPosition类头文件



namespace rov_planning {

/* Main class representing the application */
class MyApplicationImpl : public MyApplication {
    /*service alias*/

public:
    /**
     * \brief constructor
     */
    explicit MyApplicationImpl();
    /**
     * \brief deconstructor
     */
    ~MyApplicationImpl() = default;

protected:
    /**
     * \brief  Actual workload to be executed periodically. The parameter contains the
     *  state to be preserved across iterations.
     */
    void PeriodicWork(void) override;
    /**
     * \brief  for signal handle
     * \param  sig   signal id
     */
    void SignalHandler(int sig) override;
    /**
     * \brief  start server
     */
    void ComManagementInitialize() override;
    /**
     * \brief stop server
     */
    void ComManagementShutdown() override;

private:

	std::shared_ptr<ThreadPool> pool_;
    std::shared_ptr<RosCom> roscom_{nullptr};
    std::shared_ptr<LeakageParser> leakage_parser_{nullptr};
    std::shared_ptr<AutoCruise> auto_cruise_{nullptr};
	std::shared_ptr<MotionPlanner> motion_planner_{nullptr};
	std::shared_ptr<bms::DalyBmsReader> bms_reader_{nullptr};
	std::shared_ptr<RovStateMachine> state_machine_;
  std::unique_ptr<gps::SetHome> set_home_{nullptr};  // SetHome实例
  std::shared_ptr<gps::GPSPosition> gps_position_{nullptr};  // GPS位置实例（供SetHome使用）
};

}  // namespace rov_planning

#endif
