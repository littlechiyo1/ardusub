/*!
 * \file application_base.h
 * \brief
 * \author IAT Digital
 * \version 1.0
 * \date 2023-12-25
 *
 * \copyright Copyright (c) 2014-2025  阿尔特(北京)汽车数字科技有限公司
 *
 * Unpublished copyright. All rights reserved. This material contains
 * proprietary information that should be used or copied only within
 * IAT (Beijing) Automotive Digital Technology Co., Ltd., except with
 * written permission of IAT (Beijing) Automotive Digital Technology Co., Ltd.
 *
 * \par 修改日志:
 * <table>
 * <tr><th>Date      <th>Version <th>Author     <th>Description
 * <tr><td>2024-1-15 <td>1.0     <td>kan tao    <td>Initialize create
 * </table>
 */

#ifndef _ADAPTIVEAPP_H_INCLUDE
#define _ADAPTIVEAPP_H_INCLUDE

#include <pthread.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include "ros/ros.h"  


namespace rov_planning {
/* Main class representing the application */
class MyApplication {
public:
    /**
     * \brief constructor
     */
    MyApplication();
    /**
     * \brief deconstructor
     */
    virtual ~MyApplication(){};
    /**
     * \brief Threads initialize method
     */
    void Initialize();
    /**
     * \brief Threads run method
     */
    void Run();
    /**
     * \brief Threads shutdown method
     */
    void Shutdown();

protected:
    /**
     * \brief Actual workload to be executed periodically. The parameter contains the\
     * state to be preserved across iterations.
     */
    virtual void PeriodicWork(void) = 0;
    /**
     * \brief signal  handle method
     * \param  sig        sig  Id
     */
    virtual void SignalHandler(int sig) = 0;
    /**
     * \brief   start servers
     */
    virtual void ComManagementInitialize() = 0;
    /**
     * \brief shut down servers
     */
    virtual void ComManagementShutdown() = 0;
    /**
     *  \brief Time period of server
     */
    static constexpr int kRunnablePeriodTime = 200;
    /**
     * \brief Flag to identify whether the application was requested to terminate,
     * i.e., has received a SIGTERM
     */
    std::atomic_bool exit_requested_;
    /**
     * \brief signal handle thread
     */
    std::thread sig_thread_;

private:
    /**
     * \brief Vector to store all threads spawned by this application.
     */
    std::thread period_thread_;
    /**
     * \brief Entry point of the thread receiving signals from the execution manager
     */
    void SignalHandlerThread();
};
}  // namespace rov_planning

#endif
