/*!
 * \file application_base.cpp
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
 * <tr><th>Date      <th>Version <th>Author      <th>Description
 * <tr><td>2024-1-15 <td>1.0     <td>kan tao     <td>Initialize create
 * </table>
 */

#include "application_base.h"
namespace rov_planning {

//  Initialize the exit flag on creation of the variable
MyApplication::MyApplication() : exit_requested_{false} {
    // Block all signals for this thread. Signal mask will be inherited by subsequent threads.
    sigset_t signals;
    sigfillset(&signals);
    pthread_sigmask(SIG_SETMASK, &signals, NULL);
    // spawn a new signal handler thread
    sig_thread_ = std::thread(&MyApplication::SignalHandlerThread, this);
}

void MyApplication::SignalHandlerThread() {
    sigset_t signal_set;

    /*#10 empty the set of signals.*/
    if (0 != sigemptyset(&signal_set)) {
        ROS_ERROR("rov_planning could not empty signal set.");
    }
    /*#28 add SIGTERM to signal set.*/
    if (0 != sigaddset(&signal_set, SIGTERM)) {
         ROS_ERROR("rov_planning cannot add signal to signal set:SIG TERN");
    }
    /*#21 add SIGINT to signal set.*/
    if (0 != sigaddset(&signal_set, SIGINT)) {
        ROS_ERROR("rov_planning cannot add signal to signal set:SIGINT");
    }

    /*#30 loop until SIGTERM or SIGINT signal received.*/
    int sig{-1};

    do {
        // sigwait result cannot be 0
        if (0 != sigwait(&signal_set, &sig)) {
            ROS_ERROR("rov_planning called sigwait()with invalid signal set");
        }
        ROS_INFO("rov_planning received signal:%d",sig);

        if ((sig == SIGTERM) | (sig == SIGINT)) {
            ROS_INFO("rov_planning received SIGTERM or SIGINT,requesting application shutdown.");
            if (!exit_requested_) {
                /*#35 request application exit.(SignalHandler initiate the shutdown!)*/
                exit_requested_ = true;
            }
        }
    } while (!exit_requested_);
}

void MyApplication::Run() {
    /* store the current point of time in next_run */
    std::chrono::steady_clock::time_point next_run(std::chrono::steady_clock::now());
    /* define a time interval of 1000 milliseconds */
    std::chrono::steady_clock::duration duration(std::chrono::milliseconds(MyApplication::kRunnablePeriodTime));

    while (!exit_requested_) {
        /* calculate the next point of time to be reschedueled */
        next_run += duration;
        /* Blocks the execution of the current thread until specified point of time (next_run) has been reached */
        std::this_thread::sleep_until(next_run);
    }
}

void MyApplication::Initialize() {
    ComManagementInitialize();
    /* May spawn some more threads here */
    period_thread_ = std::thread(&MyApplication::PeriodicWork, this);
}

void MyApplication::Shutdown() {
    /* wait till other threads have joined */
    if (!exit_requested_) {
        exit_requested_ = true;
    }
    if (sig_thread_.joinable()) {
        sig_thread_.join();
    } else {
        ROS_ERROR("sig_thread_ join fail!");
    }

    ComManagementShutdown();
    if (period_thread_.joinable()) {
        period_thread_.join();
    } else {
        ROS_ERROR("period_thread_ join fail!");
    }

}

}  // namespace rov_planning
