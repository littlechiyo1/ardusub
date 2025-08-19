/*
1、五个维度类(roll、pitch、yaw、depth、#speed#)
2、GetParam接口(state->depth、yaw)
3、PD算法 => 匀速——减速(差值小于等于 value 时)
4、维度目标值：
    1.ros_com初始化，current值====(yaw、depth)
    2.rc_control结束，rc_in置于1500时，current值====(yaw、depth)
    3.mqtt发布目标值====(yaw、depth、roll、pitch)
5、新线程，单独kp、kd，PD算法执行开关(launch开关、rc_control开关)
*/

#ifndef PD_CONTROLLER_H
#define PD_CONTROLLER_H

#include <ros/ros.h>
#include <mutex>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace rov_planning {

class BaseController {
public:
    virtual ~BaseController() = default;

    virtual void Init(ros::NodeHandle& nh) = 0;
    virtual void UpdateParams() = 0;
    virtual double Calculate(double target, double current, double dt) = 0;
    virtual void SetTarget(double target) = 0;
    virtual double GetTarget() const = 0;
    virtual double GetError() const = 0;
    virtual void Reset() = 0; 
};

class PDController : public BaseController {
public:
    PDController();
    ~PDController() = default;
    
    void Init(ros::NodeHandle& nh) override;
    void UpdateParams() override;
    double Calculate(double target, double current, double dt) override;
    void SetTarget(double target) override;
    double GetTarget() const override;
    double GetError() const override;
    void Reset() override;

private:
    double kp_;  // 比例系数
    double kd_;  // 微分
    double max_output_;  // max
    double min_output_;  // min
    
    // 状态
    double target_;
    double previous_error_;
    double previous_time_;
    bool initialized_;
    
    // 参数服务器
    ros::NodeHandle nh_;
    mutable std::mutex mutex_;
};

class ControllerFactory {
public:
    static std::unique_ptr<BaseController> CreateController(const std::string& type);
};

} // namespace rov_planning

#endif // PD_CONTROLLER_H