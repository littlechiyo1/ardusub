#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "planning_common.h"
#include <cstdint>
#include <functional>
#include <vector>
#include <memory>
#include <chrono>
#include "mqtt.h"

namespace rov_planning {

	
    class RovStateMachine {
    public:
    typedef enum {
	    INIT = 0,
	    STANDBY,
	    ACTIVE,
	    DEGRADE,
	    ERROR
	} StateEnum;
    
    typedef enum {
	        CONDITION_NONE = 0,
	        CONDITION_STANDBY_TO_ACTIVE,
	        CONDITION_ACTIVE_TO_STANDBY,
	        CONDITION_TO_DEGRADE,
	        CONDITION_TO_ERROR,
	        CONDITION_RECOVER_NORMAL
	    } ConditionEnum;
	
 struct FsmTable {
    FsmTable(ConditionEnum _event, int _cur_state,  
             std::function<void()> _action_func, int _next_state)
        : event(_event),
          cur_state(_cur_state),
          action_func(_action_func),
          next_state(_next_state) {}

    FsmTable(){};
    ConditionEnum event;  
    int cur_state;
    std::function<void()> action_func;
    int next_state;
};
        explicit RovStateMachine(std::shared_ptr<RosCom> ros_com, 
                                std::shared_ptr<MQTT::Mqtt_imp> mqtt,
                                std::shared_ptr<LeakageParser> leakage_parser);
        ~RovStateMachine() = default;
        
        RovStateMachine(const RovStateMachine&) = delete;
        RovStateMachine(RovStateMachine&&) = delete;
        RovStateMachine& operator=(const RovStateMachine&) = delete;
        RovStateMachine& operator=(RovStateMachine&&) = delete;

        void Start();
        void Stop();
        bool HandleEvent(ConditionEnum event);
        int GetCurrentState() const;

        void SetParameters(double voltage_threshold, double soc_threshold, 
                          double degrade_timeout, double error_timeout);
        void UpdateSystemStatus(bool mqtt_connected, int connect_counter, 
                               bool leakage_detected, double battery_voltage, 
                               double battery_soc);
        void ProcessSystemStatus(bool mqtt_connected, int connect_counter,
                               bool leakage_detected, double battery_voltage,
                               double battery_soc);
        int GetTargetState(int current_state, ConditionEnum event);

    private:
        void DoNothing();
        void ExecuteArmCommand();
        void ExecuteDisarmCommand();
        void ExecuteWarning();
        void ExecuteEmergencyAscend();
        void InitializeFsmTable();

        int current_state_;
        std::shared_ptr<RosCom> ros_com_ptr_;
        std::shared_ptr<MQTT::Mqtt_imp> mqtt_ptr_;
        std::vector<FsmTable> fsm_table_;
        std::shared_ptr<LeakageParser> leakage_parser_;

        double voltage_threshold_;
        double soc_threshold_;
        double mqtt_degrade_timeout_;
        double mqtt_error_timeout_;
        
        std::chrono::steady_clock::time_point mqtt_disconnect_time_;
        bool mqtt_connected_;
    };
}  // namespace rov_planning

#endif