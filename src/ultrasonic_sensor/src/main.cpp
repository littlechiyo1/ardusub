#include <ros/ros.h>
#include "ultrasonic_sensor_node.h"

const std::string kDefaultSerialPort = "/dev/ttyTHS2";
const int kDefaultBaudRate = 115200;

int main(int argc, char**argv) {
    ros::init(argc, argv, "sensor_serial_node");
    ros::NodeHandle nh;

    ultrasonic_sensor::ultrasonic::UltrasonicSensorNode sensor_node(
        nh, 
        kDefaultSerialPort, 
        kDefaultBaudRate
    );

    if (!sensor_node.Init()) {
        ROS_FATAL("faild to init sensor node");
        return -1;
    }

    sensor_node.Run();

    return 0;
}
