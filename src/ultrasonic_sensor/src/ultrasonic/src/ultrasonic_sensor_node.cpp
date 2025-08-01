#include "ultrasonic_sensor_node.h"
#include <iostream>

namespace ultrasonic_sensor {
namespace ultrasonic {

UltrasonicSensorNode::UltrasonicSensorNode(ros::NodeHandle& nh, 
                                           const std::string& serial_port, 
                                           int baud_rate)
    : nh_(nh),
      serial_port_(serial_port),
      baud_rate_(baud_rate),
      is_running_(false),
      mqtt_client_(nullptr) {
    distance_pub_ = nh_.advertise<std_msgs::UInt16>("mavros/sensor/distance", 1000);
}

UltrasonicSensorNode::~UltrasonicSensorNode() {
    if (serial_.isOpen()) {
        serial_.close();
        ROS_INFO_STREAM("close sensor port " << serial_port_);
    } else {

    }

    if (mqtt_client_) {
        mqtt_client_->stop();
        delete mqtt_client_;
        mqtt_client_ = nullptr;
    } else {

    }
}

bool UltrasonicSensorNode::Init() {
    mqtt_client_ = new MQTT::Mqtt_imp("ultrasonic_sensor", true);
    mqtt_client_->init();
    mqtt_client_->start();

    ros::Duration(2.0).sleep();
    
    if (InitSerial()) {
        is_running_ = true;
        return true;
    }
    return false;
}

void UltrasonicSensorNode::Run() {
    ros::Rate loop_rate(kLoopRate);
    
    while (ros::ok() && is_running_) {
        ProcessSensorData();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

bool UltrasonicSensorNode::InitSerial() {
    try {
        serial_.setPort(serial_port_);
        serial_.setBaudrate(baud_rate_);
        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        serial_.setTimeout(to);
        serial_.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("unable to open sensor port: " << e.what());
        return false;
    }

    if (serial_.isOpen()) {
        ROS_INFO_STREAM("open sensor port: " << serial_port_);
        return true;
    } else {
        ROS_ERROR_STREAM("faild to open sensor port");
        return false;
    }
}

void UltrasonicSensorNode::ProcessSensorData() {
    // Step 1: Send trigger 'a'
    serial_.write("a");

    // Step 2: Wait for sensor response
    usleep(kDelayUs);

    // # Step 3: Try reading 4 bytes
    if (serial_.available() >= kResponseSize) {
        std::string response = serial_.read(kResponseSize);
        
        uint8_t header = static_cast<uint8_t>(response[0]);
        uint8_t high = static_cast<uint8_t>(response[1]);
        uint8_t low = static_cast<uint8_t>(response[2]);
        uint8_t checksum = static_cast<uint8_t>(response[3]);

        if (header == kHeader) {
            uint8_t calculated_sum = (header + high + low) & 0xFF;
            if (calculated_sum == checksum) {
                uint16_t distance = (high << 8) | low;
                ROS_INFO_STREAM("distance: " << distance << " mm");

                std_msgs::UInt16 distance_msg;
                distance_msg.data = distance;
                distance_pub_.publish(distance_msg);

                mqtt_client_->SetUltrasonicData(distance);   // Mqtt
            } else {
                ROS_WARN_STREAM("Checksum mismatch: " << static_cast<int>(checksum)
                    << ", sum: " << static_cast<int>(calculated_sum));
            }
        } else {
            ROS_WARN_STREAM("Invalid header: 0x" << std::hex << static_cast<int>(header));
        }
    } else {
        ROS_WARN("Incomplete packet or no response");
        serial_.flushInput();
    }
}

}  // namespace ultrasonic
}  // namespace ultrasonic_sensor
