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

#include "bms_reader.h"
#include <cstring>
#include <cstdio>

namespace bms{

DalyBmsReader::DalyBmsReader()
    : socket_(0),
      frame_({0}),
      send_flag_(false),
      recv_flag_(false){}

void DalyBmsReader::Start(){

    struct sockaddr_can addr;
    struct ifreq ifr;

    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(ifr.ifr_name, "can0");
    ioctl(socket_, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ROS_ERROR("bind can0 socket error!");
        return;
    }

     if (socket_ < 0) {
        ROS_ERROR("Create CAN socket failed");
        return;
    }
    
    if (ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
        ROS_INFO("Get CAN interface index failed");
        close(socket_);
        socket_ = -1;
        return;
    }

    send_flag_.store(true);
    recv_flag_.store(true);

    if(send_flag_.load()){
        send_thread_ = std::thread(&DalyBmsReader::SendProcess, this);
    }
    if(recv_flag_.load()){
        recv_thread_ = std::thread(&DalyBmsReader::RecvProcess, this);
    }
}

void DalyBmsReader::Stop(){
	if (send_flag_.load() == false && recv_flag_.load() == false) {
		return;
	}
	
    send_flag_.store(false);
    recv_flag_ .store(false);

    if (socket_ > 0) {
        close(socket_);
        socket_ = 0;
    }

    if (send_thread_.joinable()) {
        send_thread_.join();
    }
    
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
}


void DalyBmsReader::GetBmsInfo(BmsBasicInfo& ret) const
{
    std::lock_guard<std::mutex> lock(bms_info_mutex_);
    ret = bms_info_;
}

void DalyBmsReader::SendProcess(){
    int nbytes = 0;
	std::chrono::steady_clock::time_point next_run(std::chrono::steady_clock::now());
    std::chrono::steady_clock::duration duration(std::chrono::milliseconds(DalyBmsReader::SLEEP_TIME));
    while(send_flag_.load()){
		frame_ = {0};
		frame_.can_id = 0x18900140;
		frame_.can_dlc = 8;
		nbytes = write(socket_, &frame_, sizeof(frame_));
        
        next_run += duration;
        /* Blocks the execution of the current thread until specified point of time (next_run) has been reached */
        std::this_thread::sleep_until(next_run);
    }
}

void DalyBmsReader::RecvProcess(){
    int nbytes = 0;
    struct can_frame frame;
    while(recv_flag_.load()){
        nbytes = read(socket_, &frame, sizeof(frame));
        if(nbytes > 0){
            if(frame.can_id == 0x18D04001){
                std::lock_guard<std::mutex> lock(bms_info_mutex_);

                uint16_t raw_current = (frame.data[4] << 8) | frame.data[5];
                bms_info_.current = (raw_current - 30000) * 0.1f;

                uint16_t raw_soc = (frame.data[6] << 8) | frame.data[7];
                bms_info_.state_of_charge = raw_soc * 0.1f;

                uint16_t gathered_voltage = (frame.data[2] << 8) | frame.data[3];
                bms_info_.gather_total_voltage = gathered_voltage * 0.1f;

                uint16_t cumulative_voltag = (frame.data[0] << 8) | frame.data[1];
                bms_info_.cumulative_total_voltag = cumulative_voltag * 0.1f;

                
                ROS_INFO("BMS回复: 采集电压=%.1fV, 累计电压=%.1fV, 电流=%.1fA, SOC=%.1f%%\n", 
                    bms_info_.gather_total_voltage, bms_info_.cumulative_total_voltag, bms_info_.current, bms_info_.state_of_charge);
            }
            else {
                ROS_INFO("BMS回复: 帧ID错误");
            }
        }
        else {
            ROS_INFO("BMS回复: 接收失败");
        }
    }
}
}