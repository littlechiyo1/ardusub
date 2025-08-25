/**
 * @file mqtt.cpp
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

#include "mqtt.h"
namespace MQTT {

Mqtt_imp::Mqtt_imp(const char *id) 
    : mosquittopp(id), 
      closed_(true),
      connected_(false),
      pad_connected_(false) {}

Mqtt_imp &Mqtt_imp::get_single() {
    static Mqtt_imp instance("planning");
    return instance;
}

Mqtt_imp::~Mqtt_imp() { stop(); }

bool Mqtt_imp::init() {
    // 设置MQTT遗言消息
    will_set(WILL_MSG);
    // 设置订阅消息的主题
    string_topic_map_["SPAD/ModeControl"] = Topic::SPAD_MODE_CONTROL;
    string_topic_map_["SPAD/RCControl"] = Topic::SPAD_RC_CONTROL;
    string_topic_map_["SPAD/ArmedControl"] = Topic::SPAD_ARMED_CONTROL;
    string_topic_map_["SPAD/Heartbeat"] = Topic::SPAD_HEARTBEAT;
  	string_topic_map_["SPAD/AutoCruise"] = Topic::SPAD_AUTO_CRUISE;
  	string_topic_map_["SPAD/DebugTargetValue"] = Topic::SPAD_DEBUG_TARGET_VALUE;
	
    return true;
}

void Mqtt_imp::start() {
    closed_ = false;
    mqtt_thread_ = std::thread(&Mqtt_imp::process, this);
}

void Mqtt_imp::stop() {
    closed_ = true;
    try {
        if (mqtt_thread_.joinable()) {
            mqtt_thread_.detach();
        }
       
    } catch (const std::exception &e) {
        std::cout << "catched mqtt thread exception (shutdown):" << e.what() << std::endl;
    }
} 

void Mqtt_imp::on_connect(int rc) 
{ 
    if (rc == 0) {
      std::cout << "Broker connected! " << std::endl;
      connected_.store(true);
      subscribe(nullptr, "SPAD/RCControl");
      subscribe(nullptr, "SPAD/ModeControl");
      subscribe(nullptr, "SPAD/ArmedControl");
      subscribe(nullptr, "SPAD/Heartbeat");
	  subscribe(nullptr, "SPAD/AutoCruise");
	  subscribe(nullptr, "SPAD/DebugTargetValue");
    } else {
        std::cerr << "Connect failed: " << mosqpp::connack_string(rc) << std::endl;
    }
    
}

void Mqtt_imp::on_disconnect(int rc) 
{ 
    std::cout << "Disconnected: " << rc << ". Reconnecting..." << std::endl;
    connected_.store(false);
}

void Mqtt_imp::on_publish(int rc) {  }

void Mqtt_imp::on_subscribe(int mid, int qos_count, const int *granted_qos) {
    std::cout << "sub mid:" << mid << std::endl;
}

void Mqtt_imp::SetRCControlCallBack(const std::function<void(const RCControl&)>& cb)
{
    rc_control_func_ = cb;
}

void Mqtt_imp::SetModeControlCallBack(const std::function<void(const ModeControl&)>& cb)
{
    mode_control_func_ = cb;
}

void Mqtt_imp::SetArmedControlCallBack(const std::function<void(const ArmedControl&)>& cb)
{
    armed_control_func_ = cb;
}

void Mqtt_imp::SetDebugTargetValueCallBack(const std::function<void(int, double)>& cb)
{
    debug_target_value_func_ = cb;
}

void Mqtt_imp::SetMotionPlannerCallBack(const std::function<void(const RCControl&)>& cb)
{
    motion_planner_func_ = cb;
}

void Mqtt_imp::on_message(const struct mosquitto_message *message) {
    bool res = false;
    Topic topic = Topic::NO_TOPIC;
    for (const auto &temp : string_topic_map_) {
        mosqpp::topic_matches_sub(temp.first.c_str(), message->topic, &res);
        if (res) {
            topic = temp.second;
            break;
        }
    }
	 std::cout << (const char *)message->topic << std::endl;
     std::cout << (const char *)message->payload << std::endl;
    switch (topic) {
        case Topic::SPAD_RC_CONTROL: {
            ParseRCControl(static_cast<const char *>(message->payload));
            break;
        }
        case Topic::SPAD_MODE_CONTROL: {
            ParseModeControl(static_cast<const char *>(message->payload));
            break;
        }
        case Topic::SPAD_ARMED_CONTROL: {
            ParseArmedControl(static_cast<const char *>(message->payload));
            break;
        }
        case Topic::SPAD_HEARTBEAT: {            
            ParseHeartbeat(static_cast<const char *>(message->payload));
            break;
        }
		case Topic::SPAD_AUTO_CRUISE: {            
            ParseAutoCruise(static_cast<const char *>(message->payload));
            break;
        }
		case Topic::SPAD_DEBUG_TARGET_VALUE: {            
            DebugTargetValue(static_cast<const char *>(message->payload));
            break;
        }
        default:
            break;
    }
}


void Mqtt_imp::process() {
  
	const int delay_time{2};
    reconnect_delay_set(delay_time, delay_time, true);
    connect_async(SERVER_IP,PORT,  KEEP_ALIVE); // 异步连接
        
    // 循环处理数据
    loop_start();
    auto period = std::chrono::milliseconds(HEART_BEAT_ADD_PERIOD);
    while (!closed_) {
        {
            std::lock_guard<std::mutex> lock(heartbeat_mutex_);
            heartbeat_reset_count_++;
            if (heartbeat_reset_count_ >= MAX_HEARTBEAT_COUNT) {
                pad_connected_.store(false);
            }
        }
        
        /* Blocks the execution of the current thread until specified point of time (next_run) has been reached */
        std::this_thread::sleep_for(period);
        
    }
}


bool Mqtt_imp::ParseRCControl(const char *mess) {

    rapidjson::Document dom;
    RCControl temp;
    if (!dom.Parse(mess).HasParseError()) {
        if (dom.HasMember("rc_in") && dom["rc_in"].IsInt()) {
            temp.rc_in = dom["rc_in"].GetInt();
        }
        if (dom.HasMember("rc_control_value") && dom["rc_control_value"].IsDouble()) {
            temp.rc_control_value = dom["rc_control_value"].GetDouble();
        }
		auto_cruise_->CloseAutoCruiseThread();
		motion_planner_func_(temp);
        rc_control_func_(temp);
		
      
    } else {
        return false;
    }
    return true;
}

bool Mqtt_imp::ParseModeControl(const char *mess) {
    rapidjson::Document dom;
    ModeControl temp;
    if (!dom.Parse(mess).HasParseError()) {
        if (dom.HasMember("mode_control") && dom["mode_control"].IsInt()) {
            temp.mode_control = dom["mode_control"].GetInt();
        }
		auto_cruise_->CloseAutoCruiseThread();
        mode_control_func_(temp);
    } else {
        return false;
    }
    return true;

}

bool Mqtt_imp::ParseArmedControl(const char *mess) {
    rapidjson::Document dom;
    ArmedControl temp;
    if (!dom.Parse(mess).HasParseError()) {
        if (dom.HasMember("armed_control") && dom["armed_control"].IsInt()) {
            temp.armed_control = dom["armed_control"].GetInt();
        }
		auto_cruise_->CloseAutoCruiseThread();
        armed_control_func_(temp);
        
      
    } else {
        return false;
    }
    return true;

}

bool Mqtt_imp::ParseHeartbeat(const char *mess) {
    ROS_INFO("Received heartbeat from HMI.");
    {
        std::lock_guard<std::mutex> lock(heartbeat_mutex_);
        heartbeat_reset_count_ = 0U;
        pad_connected_.store(true);
    }

    return true;
}

bool Mqtt_imp::ParseAutoCruise(const char *mess) {
    ROS_INFO("Received auto cruise");
    auto_cruise_->StartAutoCruiseThread();
    return true;
}

bool Mqtt_imp::DebugTargetValue(const char *mess) {
    ROS_INFO("Received DebugTargetValue");
	rapidjson::Document dom;
	int type{0};
	double value{0.0};
    if (!dom.Parse(mess).HasParseError()) {
        if (dom.HasMember("type") && dom["type"].IsInt()) {
            type = dom["type"].GetInt();
        }
		if (dom.HasMember("value") && dom["value"].IsDouble()) {
            value = dom["value"].GetDouble();
        }
		auto_cruise_->CloseAutoCruiseThread();       
        debug_target_value_func_(type, value);
    } else {
        return false;
    }
    return true;
}

void Mqtt_imp::SetImuIfo(const ImuInfo& imu_info)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    imu_info_ = imu_info;
}

void Mqtt_imp::SetState(const ModeStatus& status) {
    
    rapidjson::StringBuffer buf_json;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buf_json);
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        mode_status_ = status;
        writer.StartObject();
        writer.Key("motion_status");
        writer.Int(mode_status_.motion_status);
        writer.Key("armed_status");
        writer.Int(mode_status_.armed_status);
        writer.Key("depth");
        writer.Double(mode_status_.depth);
        writer.EndObject();

    }
    
    std::string result = buf_json.GetString();
    const char* topic = "MACH/ModeStatus";
    // ROS_INFO("motion:%d, armed:%d, depth:%f",status.motion_status, status.armed_status, status.depth);
    if (!connected_.load()) {
        return;
    }
    publish(nullptr, topic, result.size(), result.data());
}

void Mqtt_imp::SetRCOut(const std::vector<int>& data) {
   
    rapidjson::StringBuffer buf_json;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buf_json);
    writer.StartObject();
    writer.Key("motion_control");
    writer.StartArray();
    // ROS_INFO("-------------------------------");
    for (const auto& value : data) {
        writer.Int(value);         
        // ROS_INFO("value:%d",value);
    }
    writer.EndArray();
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        writer.Key("yaw");
        writer.Double(imu_info_.yaw);
        writer.Key("pitch");
        writer.Double(imu_info_.pitch);
        writer.Key("roll");
        writer.Double(imu_info_.roll);
    }
    writer.EndObject();
    std::string result = buf_json.GetString();
    const char* topic="MACH/RCOut";
    if (!connected_.load()) {
        return;
    }
    publish(nullptr,topic,result.size(),result.data());
}

void Mqtt_imp::SetBatteryStatus(const BatteryStatus& battery_status) {
    rapidjson::StringBuffer buf_json;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buf_json);
    {
        battery_status_ = battery_status;
        writer.StartObject();
        writer.Key("voltage");
        writer.Double(battery_status_.voltage);
        writer.Key("current");
        writer.Double(battery_status_.current);
        writer.Key("remaining");
        writer.Double(battery_status_.remaining);
    }
    writer.EndObject();
    std::string result = buf_json.GetString();
    const char* topic = "MACH/BatteryStatus";
    publish(nullptr, topic, result.size(), result.data());
}

void Mqtt_imp::SetAutoCruisePtr(std::shared_ptr<rov_planning::AutoCruise>& ptr)
{
	auto_cruise_ = ptr;
}

}  // namespace MQTT
