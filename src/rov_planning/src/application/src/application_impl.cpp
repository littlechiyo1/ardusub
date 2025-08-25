/*!
 * \file application_impl.cpp
 * \brief
 * \author IAT Digital
 * \version 1.0
 * \date 2023-12-25
 *
 * \copyright Copyright (c) 2014-2023  阿尔特(北京)汽车数字科技有限公司
 *
 * Unpublished copyright. All rights reserved. This material contains
 * proprietary information that should be used or copied only within
 * IAT (Beijing) Automotive Digital Technology Co., Ltd., except with
 * written permission of IAT (Beijing) Automotive Digital Technology Co., Ltd.
 *
 * \par 修改日志:
 * <table>
 * <tr><th>Date      <th>Version <th>Author      <th>Description
 * <tr><td>2024-1-15 <td>1.0     <td>kan tao     <td>Initialize create
 * </table>
 */

#include "application_impl.h"

namespace rov_planning {

const size_t MAX_TASK_NUM = 100U;

/* Initialize the exit flag on creation of the variable */
MyApplicationImpl::MyApplicationImpl() 
    : pool_{std::make_shared<ThreadPool>(MAX_TASK_NUM)},
	  roscom_{std::make_shared<RosCom>()},
      leakage_parser_{std::make_shared<LeakageParser>()},
	  auto_cruise_{std::make_shared<AutoCruise>(pool_, roscom_)},
	  motion_planner_{std::make_shared<MotionPlanner>(roscom_)}
{
    
}

void MyApplicationImpl::PeriodicWork(void) 
{
    const int cycle_time = 5;
    ros::Rate loop_rate(cycle_time);                
    int count = 0;
    while (!exit_requested_ && ros::ok()) {       
        
        ros::spinOnce();                    
        loop_rate.sleep();
        ++count;
    }

}

void MyApplicationImpl::SignalHandler(int sig) 
{
    /* Do the actual work */
}

void MyApplicationImpl::ComManagementInitialize() 
{
    mosqpp::lib_init(); 

    MQTT::Mqtt_imp::get_single().init();
	
	auto_cruise_->Init();
	
	MQTT::Mqtt_imp::get_single().SetAutoCruisePtr(auto_cruise_);

    roscom_->Init();
	motion_planner_->Init();

    leakage_parser_->SetGPIOConfig(roscom_->GetLeakageInputIO());
    
    auto mode_control_cb = std::bind(&RosCom::ModeControlCallBack, roscom_, std::placeholders::_1);
    MQTT::Mqtt_imp::get_single().SetModeControlCallBack(mode_control_cb);

    auto armed_control_cb = std::bind(&RosCom::ArmedControlCallBack, roscom_, std::placeholders::_1);
    MQTT::Mqtt_imp::get_single().SetArmedControlCallBack(armed_control_cb);
	
	auto rc_control_cb = std::bind(&RosCom::RCControlCallBack, roscom_, std::placeholders::_1);
    MQTT::Mqtt_imp::get_single().SetRCControlCallBack(rc_control_cb);
	
	auto target_value_cb = std::bind(&MotionPlanner::SetTargetValue, motion_planner_, std::placeholders::_1, std::placeholders::_2);
    MQTT::Mqtt_imp::get_single().SetDebugTargetValueCallBack(target_value_cb);
	
	auto manual_control_value_cb = std::bind(&MotionPlanner::SetManualControlValue, motion_planner_, std::placeholders::_1);
    MQTT::Mqtt_imp::get_single().SetMotionPlannerCallBack(manual_control_value_cb);
	
	
    leakage_parser_->Start();

    MQTT::Mqtt_imp::get_single().start();
	motion_planner_->Start();
}
void MyApplicationImpl::ComManagementShutdown() 
{
    MQTT::Mqtt_imp::get_single().stop();
    leakage_parser_->Stop();
	motion_planner_->Stop();
    mosqpp::lib_cleanup();
}





}  // namespace rov_planning
