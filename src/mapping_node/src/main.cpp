#include "gps_position.h"
#include "mapping_node.h"
#include "planning_common.h"  
#include <ros/ros.h>

int main(int argc, char* argv[]) {
    // 初始化ROS节点
    ros::init(argc, argv, "mapping_node",ros::init_options::NoSigintHandler);
    ROS_INFO("Boundary mapping node started");

    //获取GetInstance()实例
    Map::BoundaryCollector& collector = Map::BoundaryCollector::GetInstance(global_gps);

    //注册 ctrl + c函数
    signal(SIGINT,SignalHandler);


    // 进入ROS主循环
    ros::spin();

    ROS_INFO("Node shutdown complete");
    return 0;
}
    