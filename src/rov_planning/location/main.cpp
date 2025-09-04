#include "location_node.h"  // 包含LocationNode类的声明
#include <ros/ros.h>
#include <stdexcept>

int main(int argc, char**argv) {
    ros::init(argc, argv, "location_node");
    ros::NodeHandle nh("~");

    try {
        location_node::LocationNode node(nh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("Location node failed: %s", e.what());
        return 1;  
    }

    return 0;  
}