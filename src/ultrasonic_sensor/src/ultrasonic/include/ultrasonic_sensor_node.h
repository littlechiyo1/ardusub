#ifndef ULTRASONIC_SENSOR_NODE_H_
#define ULTRASONIC_SENSOR_NODE_H_

#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <serial/serial.h>
#include <string>

namespace ultrasonic_sensor {
namespace ultrasonic {

class UltrasonicSensorNode {
public:
    explicit UltrasonicSensorNode(ros::NodeHandle& nh, 
                                 const std::string& serial_port, 
                                 int baud_rate);
    
    ~UltrasonicSensorNode();
    
    bool Init();

    void Run();

    bool IsRunning() const { return is_running_; }

private:
    bool InitSerial();

    void ProcessSensorData();

    ros::NodeHandle nh_;                  // ROS节点句柄
    ros::Publisher distance_pub_;         // 距离数据发布
    serial::Serial serial_;               // 串口对象
    std::string serial_port_;             // 串口设备路径
    int baud_rate_;                       // 波特率
    bool is_running_;                     // 节点运行状态标志

    static const int kLoopRate = 2;     // 2Hz---500ms
    static const int kDelayUs = 20000;  // 20ms
    static const int kResponseSize = 4;
    static const uint8_t kHeader = 0xFF;
};

}  // namespace ultrasonic
}  // namespace ultrasonic_sensor

#endif  // ULTRASONIC_SENSOR_NODE_H_
