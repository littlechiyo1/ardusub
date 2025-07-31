#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <serial/serial.h>
#include <iostream>
#include <string>

// 串口配置参数
const std::string SERIAL_PORT = "/dev/ttyTHS2";
const int BAUD_RATE = 115200;

int main(int argc, char**argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "sensor_serial_node");
    ros::NodeHandle nh;
    
    // 创建距离数据发布者
    ros::Publisher distance_pub = nh.advertise<std_msgs::UInt16>("sensor_distance", 10);
    
    // 初始化串口
    serial::Serial ser;
    try
    {
        // 配置串口
        ser.setPort(SERIAL_PORT);
        ser.setBaudrate(BAUD_RATE);
        serial::Timeout to = serial::Timeout::simpleTimeout(100); // 100ms超时
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("无法打开串口: " << e.what());
        return -1;
    }
    
    // 检查串口是否打开
    if (ser.isOpen())
    {
        ROS_INFO_STREAM("串口已打开: " << SERIAL_PORT);
    }
    else
    {
        ROS_ERROR_STREAM("串口打开失败!");
        return -1;
    }
    
    // 主循环
    ros::Rate loop_rate(2); // 2Hz，与原代码0.5秒间隔一致
    while (ros::ok())
    {
        try
        {
            // 步骤1: 发送触发信号 'a'
            ser.write("a");
            ROS_INFO("已发送触发信号");
            
            // 步骤2: 等待传感器响应
            usleep(20000); // 20ms延时
            
            // 步骤3: 读取4字节数据
            if (ser.available() >= 4)
            {
                std::string response = ser.read(4);
                
                // 解析数据
                uint8_t header = static_cast<uint8_t>(response[0]);
                uint8_t high = static_cast<uint8_t>(response[1]);
                uint8_t low = static_cast<uint8_t>(response[2]);
                uint8_t checksum = static_cast<uint8_t>(response[3]);
                
                // 验证帧头
                if (header == 0xFF)
                {
                    // 计算校验和
                    uint8_t calculated_sum = (header + high + low) & 0xFF;
                    
                    // 校验和验证
                    if (calculated_sum == checksum)
                    {
                        // 计算距离值
                        uint16_t distance = (high << 8) | low;
                        ROS_INFO_STREAM(distance << " mm");
                        
                        // 发布距离数据
                        std_msgs::UInt16 distance_msg;
                        distance_msg.data = distance;
                        distance_pub.publish(distance_msg);
                    }
                    else
                    {
                        ROS_WARN_STREAM("校验和不匹配! 接收: " << static_cast<int>(checksum) 
                            << ", 计算: " << static_cast<int>(calculated_sum));
                    }
                }
                else
                {
                    ROS_WARN_STREAM("无效帧头: 0x" << std::hex << static_cast<int>(header));
                }
            }
            else
            {
                ROS_WARN("未收到完整数据包或无响应");
                // 清空缓冲区，避免累积无效数据
                ser.flushInput();
            }
            
            ros::spinOnce();
            loop_rate.sleep();
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR_STREAM("串口通信错误: " << e.what());
            // 尝试重新打开串口
            if (!ser.isOpen())
            {
                try
                {
                    ser.open();
                    ROS_INFO_STREAM("已重新打开串口");
                }
                catch (serial::IOException& e)
                {
                    ROS_ERROR_STREAM("重新打开串口失败: " << e.what());
                    ros::Duration(1).sleep(); // 等待1秒后重试
                }
            }
        }
    }
    
    // 关闭串口
    if (ser.isOpen())
    {
        ser.close();
        ROS_INFO_STREAM("串口已关闭");
    }
    
    return 0;
}
