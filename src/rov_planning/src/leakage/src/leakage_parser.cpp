/**
 * @file leakage_parser.cpp
 * @brief
 * @author IAT Digital
 * @version 1.0
 * @date 2025-7-31
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
 * <tr><td>2025-7-31 <td>1.0     <td>IAT Digital     <td>Initialize create
 * </table>
 */
#include "leakage_parser.h"
#include <ros/ros.h>

namespace rov_planning {

LeakageParser::LeakageParser()
    : leakage_status_(false),  
      gpio_input_pin_(0),
      exit_flag_(false) {}

void LeakageParser::SetGPIOConfig(int input_pin) {
    gpio_input_pin_ = input_pin;
}

LeakageParser::~LeakageParser() {
   
}

void LeakageParser::WorkThread() {
    
    while (!exit_flag_.load()) {
        if (GPIO::input(gpio_input_pin_) == GPIO::LOW) {
            leakage_status_.store(false);
        } else {
            leakage_status_.store(true);
        }
        // ROS_INFO("leakage_status:%d",leakage_status_.load());
        std::this_thread::sleep_for(std::chrono::milliseconds(CYCLE_DURATION));
    }
}

void LeakageParser::Start() {
    if (work_thread_.joinable()) {
        return;
    }
    exit_flag_.store(false, std::memory_order_release);
    GPIO::setmode(GPIO::BOARD);
    GPIO::setwarnings(false);
    GPIO::setup(gpio_input_pin_, GPIO::IN);
    work_thread_ = std::thread(&LeakageParser::WorkThread, this);
}

void LeakageParser::Stop() {
    exit_flag_.store(true);
    if (work_thread_.joinable()) {
        work_thread_.join();
    }
    GPIO::cleanup();
}

bool LeakageParser::GetLeakageStatus() const {
    return leakage_status_.load();
}

}  // namespace rov_planning

