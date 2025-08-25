/**
 * @file leakage_parser.h
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
#ifndef LEAKAGE_PARSER_H
#define LEAKAGE_PARSER_H
#include <atomic>
#include <thread>
#include "JetsonGPIO.h"


namespace rov_planning {


class LeakageParser {
public:
    LeakageParser();
    ~LeakageParser();  
    LeakageParser(const LeakageParser&) = delete;
    LeakageParser(LeakageParser&&) = delete;
    LeakageParser& operator=(const LeakageParser&) = delete;
    LeakageParser& operator=(LeakageParser&&) = delete;

    void Start();
    void Stop();

    void SetGPIOConfig(int input_pin);
    bool GetLeakageStatus() const;

private:
    void WorkThread();
    std::thread work_thread_;
    std::atomic_bool leakage_status_;
    int gpio_input_pin_;
  
    std::atomic_bool exit_flag_; 
    const int CYCLE_DURATION = 200;
};

}  // namespace rov_planning

#endif