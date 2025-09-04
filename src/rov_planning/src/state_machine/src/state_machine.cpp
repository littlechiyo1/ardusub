#include "state_machine.h"
#include "ros_com.h"
#include "mqtt.h" 
#include <ros/ros.h>

namespace rov_planning {

RovStateMachine::RovStateMachine(std::shared_ptr<RosCom> ros_com,
                               std::shared_ptr<MQTT::Mqtt_imp> mqtt,
                               std::shared_ptr<LeakageParser> leakage_parser)
    : current_state_(STANDBY),
      ros_com_ptr_(ros_com),
      mqtt_ptr_(mqtt),
      leakage_parser_(leakage_parser),
      voltage_threshold_(22.2),
      soc_threshold_(0.3),
      mqtt_degrade_timeout_(10.0),
      mqtt_error_timeout_(20.0),
      mqtt_connected_(true) {
    InitializeFsmTable();
    ROS_INFO("[StateMachine] Initialized with initial state: STANDBY");
}

void RovStateMachine::SetParameters(double voltage_threshold, double soc_threshold,
                                  double degrade_timeout, double error_timeout) {
    voltage_threshold_ = voltage_threshold;
    soc_threshold_ = soc_threshold;
    mqtt_degrade_timeout_ = degrade_timeout;
    mqtt_error_timeout_ = error_timeout;
}

void RovStateMachine::Start() 
{

}

void RovStateMachine::Stop()
 {
    // Implementation if needed
}

int RovStateMachine::GetTargetState(int current_state, ConditionEnum event) 
{
    for (const auto& rule : fsm_table_) {
        if (rule.cur_state == current_state && rule.event == event) { 
                    ROS_DEBUG("StateMachine Query target state: current=%d, event=%d, target=%d",
                     current_state, event, rule.next_state);
            return rule.next_state;
        }
    }

    ROS_WARN("StateMachine No matching transition rule: current=%d, event=%d",
             current_state, event);      
             return current_state;  
}

void RovStateMachine::UpdateSystemStatus(bool mqtt_connected, int connect_counter, 
                                       bool leakage_detected, double battery_voltage, 
                                       double battery_soc) {
    ProcessSystemStatus(mqtt_connected, connect_counter, leakage_detected, 
                       battery_voltage, battery_soc);
}

void RovStateMachine::ProcessSystemStatus(bool mqtt_connected, int connect_counter,
                                        bool leakage_detected, double battery_voltage,
                                        double battery_soc) {
    bool is_mqtt_normal = mqtt_connected && (connect_counter == 0);
    if (mqtt_connected != mqtt_connected_) {
        mqtt_connected_ = mqtt_connected;
        if (!mqtt_connected) {
            mqtt_disconnect_time_ = std::chrono::steady_clock::now();
        }
    }

    bool is_leakage_normal = leakage_parser_ ? !leakage_detected : true;
    bool is_battery_normal = (battery_voltage >= voltage_threshold_) && 
                           (battery_soc >= soc_threshold_);

    ConditionEnum current_event = CONDITION_NONE;

    if (!is_leakage_normal) {
        current_event = CONDITION_TO_ERROR;
        ROS_WARN("StateMachine Leakage detected, triggering ERROR event");
    } 
    else if (!is_mqtt_normal && current_state_ != 4) {
        auto now = std::chrono::steady_clock::now();
        auto disconnect_duration = std::chrono::duration_cast<std::chrono::seconds>(
            now - mqtt_disconnect_time_).count();
        
        if (disconnect_duration >= mqtt_error_timeout_) {
            current_event = CONDITION_TO_ERROR;
            ROS_WARN("StateMachine MQTT disconnected timeout (%ld seconds), triggering ERROR event", 
                    disconnect_duration);
        } else if (disconnect_duration >= mqtt_degrade_timeout_) {
            current_event = CONDITION_TO_DEGRADE;
            ROS_WARN("StateMachine MQTT disconnected (%ld seconds), triggering DEGRADE event", 
                    disconnect_duration);
        }
    }
    else if (!is_battery_normal && current_state_ != ERROR) {
        current_event = CONDITION_TO_DEGRADE;
        ROS_WARN("[StateMachine] Battery abnormal (Voltage: %.1fV, SOC: %.1f%%), triggering DEGRADE event",
                battery_voltage, battery_soc * 100);
    }
    else {
        if (current_state_ == DEGRADE || current_state_ == ERROR) {
            current_event = CONDITION_RECOVER_NORMAL;
            ROS_INFO("StateMachine System recovered to normal, triggering recovery event");
        }
    }

    if (current_event != CONDITION_NONE) {
        HandleEvent(current_event);
    }
    else {
        ROS_DEBUG("StateMachine System status normal, no state transition needed");
    }
}

void RovStateMachine::InitializeFsmTable()
 {
    fsm_table_.clear();

    // INIT state
    fsm_table_.emplace_back(CONDITION_STANDBY_TO_ACTIVE, INIT, std::bind(&RovStateMachine::DoNothing,this), INIT);
    fsm_table_.emplace_back(CONDITION_ACTIVE_TO_STANDBY, INIT, std::bind(&RovStateMachine::DoNothing,this), INIT);
    fsm_table_.emplace_back(CONDITION_TO_DEGRADE, INIT, std::bind(&RovStateMachine::ExecuteWarning,this), DEGRADE);
    fsm_table_.emplace_back(CONDITION_TO_ERROR, INIT, std::bind(&RovStateMachine::ExecuteEmergencyAscend,this), ERROR);
    fsm_table_.emplace_back(CONDITION_RECOVER_NORMAL, INIT, std::bind(&RovStateMachine::DoNothing,this), STANDBY);

    // STANDBY state
    fsm_table_.emplace_back(CONDITION_STANDBY_TO_ACTIVE, STANDBY, std::bind(&RovStateMachine::ExecuteArmCommand,this), ACTIVE);
    fsm_table_.emplace_back(CONDITION_ACTIVE_TO_STANDBY, STANDBY, std::bind(&RovStateMachine::DoNothing,this), STANDBY);
    fsm_table_.emplace_back(CONDITION_TO_DEGRADE, STANDBY, std::bind(&RovStateMachine::ExecuteWarning,this), DEGRADE);
    fsm_table_.emplace_back(CONDITION_TO_ERROR, STANDBY, std::bind(&RovStateMachine::ExecuteEmergencyAscend,this), ERROR);
    fsm_table_.emplace_back(CONDITION_RECOVER_NORMAL, STANDBY, std::bind(&RovStateMachine::DoNothing,this), STANDBY);

    // ACTIVE state
    fsm_table_.emplace_back(CONDITION_STANDBY_TO_ACTIVE, ACTIVE, std::bind(&RovStateMachine::DoNothing,this), ACTIVE);
    fsm_table_.emplace_back(CONDITION_ACTIVE_TO_STANDBY, ACTIVE, std::bind(&RovStateMachine::ExecuteDisarmCommand,this), STANDBY);
    fsm_table_.emplace_back(CONDITION_TO_DEGRADE, ACTIVE, std::bind(&RovStateMachine::ExecuteWarning,this), DEGRADE);
    fsm_table_.emplace_back(CONDITION_TO_ERROR, ACTIVE, std::bind(&RovStateMachine::ExecuteEmergencyAscend,this), ERROR);
    fsm_table_.emplace_back(CONDITION_RECOVER_NORMAL, ACTIVE, std::bind(&RovStateMachine::DoNothing,this), ACTIVE);

    // DEGRADE state
    fsm_table_.emplace_back(CONDITION_STANDBY_TO_ACTIVE, DEGRADE, std::bind(&RovStateMachine::DoNothing,this), DEGRADE);
    fsm_table_.emplace_back(CONDITION_ACTIVE_TO_STANDBY, DEGRADE, std::bind(&RovStateMachine::DoNothing,this), DEGRADE);
    fsm_table_.emplace_back(CONDITION_TO_DEGRADE, DEGRADE, std::bind(&RovStateMachine::DoNothing,this), DEGRADE);
    fsm_table_.emplace_back(CONDITION_TO_ERROR, DEGRADE, std::bind(&RovStateMachine::ExecuteEmergencyAscend,this), ERROR);
    fsm_table_.emplace_back(CONDITION_RECOVER_NORMAL, DEGRADE, std::bind(&RovStateMachine::DoNothing,this), STANDBY);

    // ERROR state
    fsm_table_.emplace_back(CONDITION_STANDBY_TO_ACTIVE, ERROR, std::bind(&RovStateMachine::DoNothing,this), ERROR);
    fsm_table_.emplace_back(CONDITION_ACTIVE_TO_STANDBY, ERROR, std::bind(&RovStateMachine::DoNothing,this), ERROR);
    fsm_table_.emplace_back(CONDITION_TO_DEGRADE, ERROR, std::bind(&RovStateMachine::DoNothing,this), ERROR);
    fsm_table_.emplace_back(CONDITION_TO_ERROR, ERROR, std::bind(&RovStateMachine::DoNothing,this), ERROR);
    fsm_table_.emplace_back(CONDITION_RECOVER_NORMAL, ERROR, std::bind(&RovStateMachine::DoNothing,this), STANDBY);
}

bool RovStateMachine::HandleEvent(ConditionEnum event) 
{
    bool transition_success = false;
    int old_state = current_state_;

    for (const auto& transition : fsm_table_) {
        if (transition.event == event && transition.cur_state == current_state_) {
            ROS_INFO("StateMachine Trigger transition: state %d,%d (event %d)",
                    current_state_, transition.next_state, event);
            
            if (transition.action_func) {
                transition.action_func();
            }
            
            current_state_ = transition.next_state;
            transition_success = true;
                        
            break;
        }
    }

    if (!transition_success) {
        ROS_WARN("StateMachine No matching transition rule found: current state=%d, event=%d",
                current_state_, event);
    }

    return transition_success;
}

void RovStateMachine::DoNothing() 
{
    ROS_INFO("StateMachine DoNothing - Current state: %d, Action: No operation", current_state_);
    ROS_DEBUG("StateMachine Maintaining current state %d", current_state_);
}

void RovStateMachine::ExecuteArmCommand() 
{
    ROS_INFO("StateMachine ExecuteArmCommand - Current state: %d, Action: Arm system", current_state_);
    ROS_DEBUG("StateMachine Transition from state %d to ACTIVE(2)", current_state_);
    
    if (current_state_ != STANDBY) {
        ROS_WARN("[StateMachine] Warning: Arm command usually triggered in STANDBY state, current state: %d",
                current_state_);
    }
}

void RovStateMachine::ExecuteDisarmCommand() 
{
    ROS_INFO("StateMachine ExecuteDisarmCommand - Current state: %d, Action: Disarm system", current_state_);
    ROS_DEBUG("StateMachine Transition from state %d to STANDBY(1)", current_state_);
    
    if (current_state_ != ACTIVE) {
        ROS_WARN("StateMachine Warning: Disarm command usually triggered in ACTIVE state, current state: %d",
                current_state_);
    }
}

void RovStateMachine::ExecuteWarning() 
{
    ROS_INFO("StateMachine ExecuteWarning - Current state: %d, Action: System degradation warning", current_state_);
    ROS_DEBUG("StateMachine Transition from state %d to DEGRADE(3)", current_state_);
    
    if (current_state_ == DEGRADE) {
        ROS_DEBUG("StateMachine System already in DEGRADE state, repeated warning");
    }
}

void RovStateMachine::ExecuteEmergencyAscend() 
{
    ROS_INFO("StateMachine ExecuteEmergencyAscend - Current state: %d, Action: Emergency ascent", current_state_);
    ROS_DEBUG("StateMachine Transition from state %d to ERROR(4)", current_state_);
    
    ROS_WARN("StateMachine Emergency! System entering ERROR state, executing emergency ascent procedure");
}

int RovStateMachine::GetCurrentState() const 
{
    return current_state_;
}

}  // namespace rov_planning