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
	  motion_planner_{std::make_shared<MotionPlanner>(roscom_)},
	  bms_reader_{std::make_shared<bms::DalyBmsReader>()},
    gps_position_{std::make_shared<gps::GPSPosition>()},  // 新增：初始化GPSPosition实例
    set_home_{std::make_unique<gps::SetHome>(gps_position_.get())}  // 新增：初始化SetHome实例
{
    
}

void MyApplicationImpl::PeriodicWork(void) 
{
    const int cycle_time = 10;
    ros::Rate loop_rate(cycle_time);                
    int count = 0;

    // 定义存储状态的局部变量（新增 connect_counter）
    bool mqtt_connected = false;       // MQTT连接状态
    int connect_counter = 0;      // MQTT连接计数器（新增）
    bool leakage_detected = false;     // 漏水状态（true表示检测到漏水）
    double battery_voltage = 0.0;      // 电池电压
    double battery_soc = 0.0;          // 电池SOC

    while (!exit_requested_ && ros::ok()) {       
        // 获取MQTT连接状态和连接计数器（新增计数器获取）
        auto& mqtt_singleton = MQTT::Mqtt_imp::get_single(); // 获取MQTT单例
        mqtt_connected = mqtt_singleton.IsConnected();
        connect_counter = mqtt_singleton.GetConnectCounter(); // 获取计数器

        // 获取漏水状态
        if (leakage_parser_) {
            leakage_detected = leakage_parser_->GetLeakageStatus();
        }

        // 获取电池状态
        if (bms_reader_) {
            BmsBasicInfo bms_info_;
            bms_reader_->GetBmsInfo(bms_info_);
            battery_voltage = bms_info_.gather_total_voltage;
            battery_soc = bms_info_.state_of_charge;
        }

        // 传递所有状态给状态机（新增 mqtt_connect_counter 参数）
        if (state_machine_) {
            state_machine_->ProcessSystemStatus(
                mqtt_connected, 
                connect_counter,  // 新增：传递连接计数器
                leakage_detected, 
                battery_voltage, 
                battery_soc
            );
        }

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
	MQTT::Mqtt_imp::get_single().SetBmsReaderPtr(bms_reader_);

    roscom_->Init();
		
	motion_planner_->Init();
    leakage_parser_->SetGPIOConfig(roscom_->GetLeakageInputIO());
	MQTT::Mqtt_imp::get_single().SetLeakageParserPtr(leakage_parser_);
	
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
    
  auto set_home_cb = std::bind(&gps::SetHome::SetCurrentAsHome,set_home_.get());
  MQTT::Mqtt_imp::get_single().SetHomeCallBack(set_home_cb);
  
     state_machine_ = std::make_shared<rov_planning::RovStateMachine>(
    roscom_,                      // 1. RosCom 智能指针
    std::shared_ptr<MQTT::Mqtt_imp>(&MQTT::Mqtt_imp::get_single()),  // 2. MQTT 智能指针
    leakage_parser_               // 3. LeakageParser 智能指针
);
	
    state_machine_->Start();
    leakage_parser_->Start();

    MQTT::Mqtt_imp::get_single().start();
	motion_planner_->Start();
	bms_reader_->Start();
}
void MyApplicationImpl::ComManagementShutdown() 
{
    MQTT::Mqtt_imp::get_single().stop();
    leakage_parser_->Stop();
	motion_planner_->Stop();
	bms_reader_->Stop();
	state_machine_->Stop();
    mosqpp::lib_cleanup();
}





}  // namespace rov_planning
