/*!
 * \file application_impl.h
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

    std::shared_ptr<RosCom> roscom_{nullptr};
    
};

}  // namespace rov_planning

#endif
