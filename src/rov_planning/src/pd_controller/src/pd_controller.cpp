#include "pd_controller.h"

namespace rov_planning {

PDController::PDController() 
    : kp_(0.0), 
      kd_(0.0), 
      max_output_(1.0), 
      min_output_(-1.0),
      target_(0.0),
      previous_error_(0.0),
      previous_time_(0.0),
      initialized_(false) {
}

void PDController::Init(ros::NodeHandle& nh) {
    nh_ = nh;
    UpdateParams();
    initialized_ = true;
    previous_time_ = ros::Time::now().toSec();
}

void PDController::UpdateParams() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 获取参数
    nh_.param("pd_controller/kp", kp_, 1.0);
    nh_.param("pd_controller/kd", kd_, 0.1);
    nh_.param("pd_controller/max_output", max_output_, 1.0);
    nh_.param("pd_controller/min_output", min_output_, -1.0);
    
    ROS_INFO("PD Controller parameters updated: Kp=%f, Kd=%f, max_output=%.4f, min_output=%.4f", 
             kp_, kd_, max_output_, min_output_);
}

double PDController::Calculate(double target, double current, double dt) {
    if (!initialized_) {
        ROS_WARN("PD Controller not initialized!");
        return 0.0;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 误差
    double error = target - current;
    
    // 微分
    double derivative = 0.0;
    if (dt > 0.0) {
        derivative = (error - previous_error_) / dt;
    }
    
    // 输出
    double output = kp_ * error + kd_ * derivative;
    
    if (output > max_output_) {
        output = max_output_;
    } else if (output < min_output_) {
        output = min_output_;
    }
    
    // 保存当前误差 => 下次计算微分项
    previous_error_ = error;
    
    return output;
}

void PDController::SetTarget(double target) {
    std::lock_guard<std::mutex> lock(mutex_);
    target_ = target;
}

double PDController::GetTarget() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return target_;
}

double PDController::GetError() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return previous_error_;
}

void PDController::Reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    previous_error_ = 0.0;
    target_ = 0.0;
    previous_time_ = ros::Time::now().toSec();
}

std::unique_ptr<BaseController> ControllerFactory::CreateController(const std::string& type) {
    if (type == "PD" || type == "pd") {
        return std::make_unique<PDController>();
    } else {
        throw std::invalid_argument("Unknown controller type: " + type);
    }
}

} // namespace rov_planning