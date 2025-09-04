/**
 * @file auto_cruise.cpp
 * @brief
 * @author IAT Digital
 * @version 1.0
 * @date 2025-8-18
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
 * <tr><td>2025-8-18 <td>1.0     <td>IAT Digital     <td>Initialize create
 * </table>
 */
 
#include "auto_cruise.h"
#include <ros/ros.h>
 
namespace rov_planning {
	 
AutoCruise::AutoCruise(std::shared_ptr<ThreadPool> pool, std::shared_ptr<RosCom>& ros_com)
	: closed_(true),
	  pool_(pool),
	  ros_com_(ros_com)
{
}

void AutoCruise::StartAutoCruiseThread()
{
	if (closed_.load()) {
		closed_.store(false);
		pool_->enqueue(&AutoCruise::RunAutoCruise, this);
	}
}
			
void AutoCruise::AutoCruise::Init()
{    
	std::ifstream ifs(config_file_path_);
    if (!ifs.is_open()) {
        ROS_ERROR("Failed to open auto cruise config file: %s", config_file_path_.c_str());
        return;
    }
    
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    std::string json_content = buffer.str();
    
    rapidjson::Document doc;
    if (doc.Parse(json_content.c_str()).HasParseError()) {
        ROS_ERROR("Failed to parse auto cruise config file");
        return;
    }
    
    if (!doc.HasMember("auto_path") || !doc["auto_path"].IsArray()) {
        ROS_ERROR("Invalid auto cruise config: missing 'auto_path' array");
        return;
    }
    
    const rapidjson::Value& auto_path = doc["auto_path"];
    for (rapidjson::SizeType i = 0; i < auto_path.Size(); ++i) {
        const rapidjson::Value& segment = auto_path[i];
        
        if (!segment.HasMember("type_list") || !segment["type_list"].IsArray() ||
            !segment.HasMember("time") || !segment["time"].IsInt()) {
            ROS_WARN("Invalid segment format, skipping...");
            continue;
        }
        
        PathSegment path_segment;
        path_segment.time = segment["time"].GetInt();
        
        const rapidjson::Value& type_list = segment["type_list"];
        for (rapidjson::SizeType j = 0; j < type_list.Size(); ++j) {
            const rapidjson::Value& action = type_list[j];
            
            if (!action.HasMember("type") || !action["type"].IsString() ||
                !action.HasMember("duty") || !action["duty"].IsInt()) {
                ROS_WARN("Invalid action format, skipping...");
                continue;
            }
            
            Action act;
            act.type = action["type"].GetString();
            act.duty = action["duty"].GetInt();
            path_segment.actions.push_back(act);
        }
        
        path_segments_.emplace_back(path_segment);
    }
    

    ROS_INFO("Successfully loaded auto cruise config: %s, %zu segments", 
             config_file_path_.c_str(), path_segments_.size());
}	

int AutoCruise::ActionTypeToControlCode(const std::string& type) {
    if (type == "forward") return FORWARD_CONTROL;
    if (type == "yaw") return YAW_CONTROL;
    if (type == "throttle") return THROTTLE_CONTROL;
    if (type == "lateral") return LATERAL_CONTROL;
    if (type == "roll") return ROLL_CONTROL;
    if (type == "pitch") return PITCH_CONTROL;
    return -1; 
}

void AutoCruise::RunAutoCruise()
{
	std::chrono::steady_clock::time_point next_run(std::chrono::steady_clock::now());
    /* define a time interval of 1000 milliseconds */
    std::chrono::steady_clock::duration duration(std::chrono::milliseconds(AutoCruise::kPeriodTime));
	int64_t counter = 0;
	const int period_to_s = 5;
	
	// 每个type_list对应counter
    std::vector<int> segment_end_counters;
    int temp_counter = 0;
    for (const auto& segment : path_segments_) {
        temp_counter += segment.time * period_to_s;
        segment_end_counters.push_back(temp_counter);
    }
    
    int max_counter = temp_counter;
	
    // 当前处理的路径段索引
    size_t current_segment_index = 0;
    // 上一个路径段结束的counter值
    int last_segment_end_counter = 0;
	
	RCControl stop_cmd;
	stop_cmd.rc_in = CLOSE_CONTROL;
	stop_cmd.rc_control_value = 0;

    while (!closed_.load() && counter < max_counter) {
        next_run += duration;
        /* Blocks the execution of the current thread until specified point of time (next_run) has been reached */
        std::this_thread::sleep_until(next_run);
        counter++;
		ROS_INFO("current_couter: %ld", counter);
        
        // 是否新路径段的开始
        if (current_segment_index < path_segments_.size() && 
            counter == last_segment_end_counter + 1) {
			ROS_WARN("---path_segment_couter---");
            // 执行
            const PathSegment& current_segment = path_segments_[current_segment_index];
            for (const auto& action : current_segment.actions) {
                int control_code = ActionTypeToControlCode(action.type);
                if (control_code != -1) {
					RCControl rc;
					rc.rc_in = control_code;
					rc.rc_control_value = action.duty;
                    ros_com_->RCControlCallBack(rc);
                    ROS_INFO("Sending command: type=%s, duty=%d, control_code=%d", 
                            action.type.c_str(), action.duty, control_code);
                } else {
                    ROS_WARN("Unknown action type: %s", action.type.c_str());
                }
            }
        }
        
        // 是否当前路径段结束
        if (current_segment_index < segment_end_counters.size() && 
            counter == segment_end_counters[current_segment_index]) {
			ROS_WARN("---path_segment_couter---");
            last_segment_end_counter = segment_end_counters[current_segment_index];
            current_segment_index++;
			ros_com_->RCControlCallBack(stop_cmd);
        }
		
		if (closed_.load()) {
			ROS_INFO("Closed signal received, stop current segment");
			ros_com_->RCControlCallBack(stop_cmd);
			break;
		}
    }
    
    // 巡航结束
    ROS_INFO("Quit for while");
	closed_.store(true);
 }
}  // namespace rov_planning