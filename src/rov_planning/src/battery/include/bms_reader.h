/*!
 * \file geofence_checker.h
 * \brief
 * \author IAT Digital
 * \version 1.0
 * \date 2025-8-19
 *
 * \copyright Copyright (c) 2014-  阿尔特(北京)汽车数字科技有限公司
 *
 * Unpublished copyright. All rights reserved. This material contains
 * proprietary information that should be used or copied only within
 * IAT (Beijing) Automotive Digital Technology Co., Ltd., except with
 * written permission of IAT (Beijing) Automotive Digital Technology Co., Ltd.
 *
 * \par 修改日志:
 * <table>
 * <tr><th>Date      <th>Version <th>Author     <th>Description
 * <tr><td>2025-8-19 <td>1.0     <td>kan tao    <td>Initialize create
 * </table>
 */

#ifndef DALY_BMS_READER_H
#define DALY_BMS_READER_H

#include <cstdint>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <ros/ros.h>
#include "planning_common.h"

namespace bms{

class DalyBmsReader {

public:
    DalyBmsReader();
    ~DalyBmsReader() = default;

    void Start();

    void Stop();

    void GetBmsInfo(BmsBasicInfo&) const;

private:
    
    void SendProcess();
    void RecvProcess();
    
    int socket_;         
    BmsBasicInfo bms_info_;   ///< BMS信息结构体
    mutable std::mutex bms_info_mutex_;

    std::atomic<bool> send_flag_;
    std::atomic<bool> recv_flag_;
    struct can_frame frame_;

    std::thread send_thread_;
    std::thread recv_thread_;

    static constexpr uint32_t SLEEP_TIME = 500U;  


};
}

#endif // DALY_BMS_READER_H