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


/* Initialize the exit flag on creation of the variable */
MyApplicationImpl::MyApplicationImpl() 
    : roscom_{std::make_shared<RosCom>()}
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
    roscom_->Init();

    MQTT::Mqtt_imp::get_single().start();

    auto rc_control_cb = std::bind(&RosCom::RCControlCallBack, roscom_, std::placeholders::_1);
    MQTT::Mqtt_imp::get_single().SetRCControlCallBack(rc_control_cb);
    
    auto mode_control_cb = std::bind(&RosCom::ModeControlCallBack, roscom_, std::placeholders::_1);
    MQTT::Mqtt_imp::get_single().SetModeControlCallBack(mode_control_cb);

    auto armed_control_cb = std::bind(&RosCom::ArmedControlCallBack, roscom_, std::placeholders::_1);
    MQTT::Mqtt_imp::get_single().SetArmedControlCallBack(armed_control_cb);
}
void MyApplicationImpl::ComManagementShutdown() 
{
    MQTT::Mqtt_imp::get_single().stop();
    mosqpp::lib_cleanup();
}





}  // namespace rov_planning
