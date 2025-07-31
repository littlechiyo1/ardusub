/**
 * @file mqtt.h
 * @brief
 * @author zeng panapn (zengpanpan@iat-auto.cn)
 * @version 1.0
 * @date 2022-04-07
 *
 * @copyright Copyright (c) 2022  阿尔特汽车技术股份有限公司
 *
 * Unpublished copyright. All rights reserved. This material contains
 * proprietary information that should be used or copied only within
 * IAT, except with written permission of IAT.
 *
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2022-04-07 <td>1.0     <td>Zeng Panpan     <td>Initialize create
 * </table>
 */

#ifndef MQTT_H
#define MQTT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <unistd.h>

#ifdef __cplusplus
}
#endif

#include <atomic>
#include <chrono>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <mutex>
#include <functional>
#include <vector>

#include "common.h"
#include "mosquitto.h"
#include "mosquittopp.h"
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "ros/ros.h" 

namespace MQTT {
class Mqtt_imp : public mosqpp::mosquittopp {
public:
    enum class Topic { NO_TOPIC,
                       SPAD_RC_CONTROL,
                       SPAD_MODE_CONTROL,
                       SPAD_ARMED_CONTROL,
                       SPAD_HEARTBEAT
                       };

public:
    static Mqtt_imp &get_single();
    ~Mqtt_imp();
    bool init();
    void start();
    void stop();

    void SetRCControlCallBack(const std::function<void(const RCControl&)>& cb);
    void SetModeControlCallBack(const std::function<void(const ModeControl&)>& cb);
    void SetArmedControlCallBack(const std::function<void(const ArmedControl&)>& cb);
    void SetState(const ModeStatus& status);
    void SetRCOut(const std::vector<int>&);
    void SetDepthData(double current_depth);
    void SetImuIfo(const ImuInfo&);

    void on_connect(int rc);
    void on_disconnect(int rc);
    void on_publish(int rc);
    void on_subscribe(int mid, int qos_count, const int *granted_qos);
    void on_message(const struct mosquitto_message *message);

private:
    Mqtt_imp(const char *id);
    void process();
    bool ParseRCControl(const char *mess);  
    bool ParseModeControl(const char *mess);
    bool ParseArmedControl(const char *mess);
    bool ParseHeartbeat(const char *mess);

    void ConnectionMonitor();
    void ConnectMqttBroker();

private:
    std::thread mqtt_thread_;
    std::thread heartbeat_monitor_thread_;
    std::atomic<bool> closed_;
    std::mutex data_mutex_;
    std::mutex heartbeat_mutex_;
    std::map<std::string, Topic> string_topic_map_;
    std::function<void(const RCControl&)> rc_control_func_ ;
    std::function<void(const ModeControl&)> mode_control_func_ ;
    std::function<void(const ArmedControl&)> armed_control_func_ ;

    ModeStatus mode_status_{0};
    ImuInfo imu_info_{0};

    const char* WILL_MSG = "offline";
    const char* SERVER_IP = "192.168.1.200";
    const int PORT = 1883;
    const int KEEP_ALIVE = 60;

    // 心跳和重连相关成员变量
    std::atomic<bool> connected_;
    std::atomic<bool> pad_connected_;
    int reconnect_attempts_;
    uint64_t heartbeat_reset_count_{0U};
    static const int MAX_RECONNECT_ATTEMPTS = 3;
    static const uint16_t HEART_BEAT_ADD_PERIOD = 500U;
    static const int MAX_HEARTBEAT_COUNT = 4;
    


};
}  // namespace MQTT
#endif
