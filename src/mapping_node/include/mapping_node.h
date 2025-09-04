#ifndef MAPPING_NODE_H
#define MAPPING_NODE_H

#include <ros/ros.h>
#include <vector>
#include <std_srvs/Trigger.h>
#include "gps_position.h"
#include "planning_common.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <fstream>
#include <thread>     //线程
#include <mutex>    //互斥锁
#include <atomic>  //原子变量
#include <signal.h> //信号量

//声明全局GPS实例
extern GPS::GPS_Position global_gps;
//声明GPS实例访问函数
GPS::GPS_Position& GetGlobalGpsInstance();
//声明信号处理函数
void SignalHandler(int sig);


namespace Map{

class BoundaryCollector {
public:
    //单例模式  获取唯一实例
    static BoundaryCollector& GetInstance(GPS::GPS_Position& gps);
    //禁止拷贝构造和赋值运算符
    BoundaryCollector(const BoundaryCollector&) = delete;
    BoundaryCollector& operator = (const BoundaryCollector&) = delete;
   
    ~BoundaryCollector() = default;

    // 设置原点
    bool SetOrigin(const GeoPoint& origin);

    // 保存边界点到文件
    bool SaveBoundaryToFile(const std::string& file_path);

    // 开始采集边界点
    void StartCollecting();

    // 停止采集边界点
    void StopCollecting();

    // 添加当前位置为边界点
    bool AddCurrentPoint();

    // 获取采集的地理坐标边界点
    std::vector<GeoPoint> GetBoundaryGeoPoints() const;

    // 获取采集的笛卡尔坐标边界点
    std::vector<CartesianPoint> GetBoundaryCartPoints() const;

    // 清空边界点
    void ClearBoundaryPoints();

    // 检查是否正在采集
    bool IsCollecting() const;

    bool IsPointValid(const CartesianPoint& new_point);

    bool LoadGeoBoundaryFromFile(const std::string& file_path);

    bool LoadAndFilterGeoFile(const std::string& file_path);


    //  获取原始保存路径
    std::string GetAutoSavePath() const {
        return auto_save_path_;

    }

private:
    //构造函数私有化  禁止外部创建
    BoundaryCollector(GPS::GPS_Position& gps);
    GeoPoint origin_position_;
    bool is_origin_ = false;
    double distance_threshold_ = 1.0;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    //过滤距离与误差
    double filter_distance_threshold_;//过滤距离阈值(2m)
    double lower_error_threshold_ = 1.5;  // 最小允许距离（1.5m）
    double upper_error_threshold_ = 3.0;  // 最大允许距离（3.0m）
    double auto_collect_interval_  = 2.0;

    std::string auto_save_path_;//自动保存文件路径
    std::string filtered_output_path_;//过滤后文件保存路径
    std::string load_geo_file_path_; //加载文件路径

    
                
    bool is_collecting_;              // 是否正在采集
    std::vector<GeoPoint> boundary_geo_;   // 地理坐标边界点
    std::vector<CartesianPoint> boundary_cart_; // 笛卡尔坐标边界点
    GPS::GPS_Position& gps_;

    //线程
    std::thread auto_collect_thread_;       // 自动采集线程
    std::atomic<bool> is_auto_collecting_;  // 原子变量：标记自动采集是否运行（线程安全）
    std::mutex point_mutex_;                // 互斥锁：保护边界点列表的读写

    // 自动采集函数（线程入口）
    void AutoCollectLoop();

    //新增
    std::vector<GeoPoint> FilterPoints(const std::vector<GeoPoint>& points);
    bool SaveFilteredPointsToFile(const std::vector<GeoPoint>& filtered_points);
    
    ros::NodeHandle nh_;

    ros::Publisher boundary_path_pub_;  // 发布边界路径
    ros::Subscriber gps_sub_; 
    nav_msgs::Path boundary_path_;      // 存储边界路径数据
    
    // ROS服务
    ros::ServiceServer start_srv_;
    ros::ServiceServer stop_srv_;
    ros::ServiceServer add_point_srv_;
 

    //新增
    ros::ServiceServer load_geo_srv_;   // 加载边界点的服务

    //新增
    ros::ServiceServer load_filter_srv_;

    // 计算两点距离
    double CalculateDistance(const CartesianPoint& a, const CartesianPoint& b);

    // 服务回调函数
    bool StartSrvCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool StopSrvCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool AddPointSrvCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    void GpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

    bool SaveSrvCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    bool LoadGeoSrvCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    //过滤服务回调函数
    bool LoadFilterSrvCallback(std_srvs::Trigger::Request& req,std_srvs::Trigger::Response& res);

};

} // namespace Map

#endif // MAPPING_NODE_H
