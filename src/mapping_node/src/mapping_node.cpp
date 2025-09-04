#include "mapping_node.h"
#include <ros/console.h>
#include <cmath>

//定义全局GPS实例
GPS::GPS_Position global_gps;
//全局GPS访问函数
GPS::GPS_Position& GetGlobalGpsInstance() 
{
    return global_gps;
}

//信号处理函数
void SignalHandler(int sig) 
{
    ROS_INFO("\nReceived Ctrl+C signal, stopping collection and processing data...");

    try {
        //单例获取BoundaryCollector实例
        Map::BoundaryCollector& collector = Map::BoundaryCollector::GetInstance(GetGlobalGpsInstance());
        //停止采集
        if(collector.IsCollecting()) {
            collector.StopCollecting();
        } else {
            ROS_WARN("Not in collecting state, skipping stop step");
        }
        //加载原始数据并触发过�?
        std::string raw_file_path = collector.GetAutoSavePath();
        std:: ifstream file_check(raw_file_path);  //声明文件流对�?
        if (file_check.good()) {
            file_check.close();
            bool filter_success = collector.LoadAndFilterGeoFile(raw_file_path);
            if (filter_success) {
                ROS_INFO("The raw data filtering has been completed, and the filtered_boundary.txt and .cartesian files have been generated.");
            } else {
                ROS_ERROR("Filtering of raw data failed. Please check the file content");
            }
        } else {
            ROS_WARN("The original file does not exist or is empty; the filtering step is skipped.");
            file_check.close();
        }
    } catch(const std::exception& e) {
        ROS_ERROR("Error during auto-processing: %s", e.what());
    }

    Map::BoundaryCollector& collector = Map::BoundaryCollector::GetInstance(GetGlobalGpsInstance());
    //延迟1�?确保发送到rviz�?
     ros::Duration(1.0).sleep();  
    ros::shutdown();
}


namespace Map{
//实现单例的GetInstance()方法
BoundaryCollector& BoundaryCollector::GetInstance(GPS::GPS_Position& gps) 
{
    static BoundaryCollector instance(gps);
    return instance;

}

BoundaryCollector::BoundaryCollector(GPS::GPS_Position& gps) : gps_(gps), is_collecting_(false) ,is_origin_(false),is_auto_collecting_(false) {
     //读取过滤参数
    nh_.param("distance_threshold", filter_distance_threshold_, 2.0);
     // 读取自动保存路径参数 
    nh_.param("auto_save_path", auto_save_path_, std::string("raw_boundary.txt"));
    //读取加载文件的路径参�?
    nh_.param("load_geo_file_path", load_geo_file_path_, auto_save_path_);

    nh_.param("filtered_output_path", filtered_output_path_, std::string("filtered_boundary.txt"));
    nh_.param("lower_error_threshold", lower_error_threshold_, 1.5);
    nh_.param("upper_error_threshold", upper_error_threshold_, 3.0);
    nh_.param("auto_collect_interval", auto_collect_interval_, 2.0);
   
    ROS_INFO("Auto-collect interval set to: %.1fs", auto_collect_interval_);

    ROS_INFO("Filter thresholds set:");
    ROS_INFO("  Base distance: %.2f meters", filter_distance_threshold_);
    ROS_INFO("  Lower error range: %.2f meters", lower_error_threshold_);   
    ROS_INFO("  Upper error range: %.2f meters", upper_error_threshold_);

    ROS_INFO("Filter distance threshold set to: %.2f meters",filter_distance_threshold_);
    ROS_INFO("Filtered output path:%s", filtered_output_path_.c_str());
    // 注册ROS服务
    start_srv_ = nh_.advertiseService("/boundary/start_collect", &BoundaryCollector::StartSrvCallback, this);
    stop_srv_ = nh_.advertiseService("/boundary/stop_collect", &BoundaryCollector::StopSrvCallback, this);
    add_point_srv_ = nh_.advertiseService("/boundary/add_point", &BoundaryCollector::AddPointSrvCallback, this);
    gps_sub_ = nh_.subscribe("/mavros/global_position/global", 1000, &BoundaryCollector::GpsCallback, this);
    boundary_path_pub_ = nh_.advertise<nav_msgs::Path>("/boundary/path", 10);
    load_geo_srv_ = nh_.advertiseService("/boundary/load_geo_file", &BoundaryCollector::LoadGeoSrvCallback, this);
    //过滤服务
    load_filter_srv_=nh_.advertiseService("/boundary/load_filter_file", &BoundaryCollector::LoadFilterSrvCallback, this);

    // 初始化路径消息的坐标系（使用GPS原点坐标系）
    boundary_path_.header.frame_id = "map";  // 需与rviz中坐标系匹配

    // 等待GPS信号有效
    ros::Rate rate(10);
    while (ros::ok() && !is_origin_) {
        ROS_WARN_THROTTLE(1, "Waiting for valid GPS data to set origin...");
        ros::spinOnce();
        rate.sleep();
    }

    if (is_origin_) {
        ROS_INFO("GPS origin set successfully. Ready to collect boundaries");
    } else {
        ROS_ERROR("Failed to get valid GPS data. Node may not function correctly");
    }
}

bool BoundaryCollector::SetOrigin(const GeoPoint& origin){
    if (is_origin_) {
        ROS_WARN("Origin already set. Use /boundary/reset_origin to re-set");
        return false;
    }

    if (origin.lat == 0.0 && origin.lon == 0.0) {
        ROS_ERROR("Cannot set origin: invalid GPS data (0,0)");
        return false;
    }

    // 设置原点数据
    origin_position_ = origin;
    gps_.reference_origin_ = origin;
    is_origin_ = true;

    ROS_INFO("Origin set successfully: (lat: %.6f, lon: %.6f, alt: %.2f)",
             origin.lat, origin.lon, origin.alt);

    // 发布TF变换
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "gps_origin";
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.w = 1.0;
    tf_broadcaster_.sendTransform(transform);

    // 自动开始采�?
    if (!is_collecting_) {
        StartCollecting();  
        ROS_INFO("Auto-started boundary collection after origin setup");
    }

    return true;
}
bool BoundaryCollector::SaveBoundaryToFile(const std::string& file_path) {
    
    // 检查是否有采集到的边界�?
    if (boundary_geo_.empty()) {
        ROS_WARN("No boundary points to save (geographic coordinates)");
        return false;
    }

    // 打开文件
    std::ofstream outfile(file_path);     //覆盖已有文件 若文件已存在,会清空写�?
    if (!outfile.is_open()) {
        ROS_ERROR("Failed to open file for saving: %s", file_path.c_str());
        return false;
    }

    // 写入文件头（记录原点信息，便于后续加载时校准�?
    outfile << "# Boundary points (WGS84 geographic coordinates)\n";
    outfile << "# Reference origin: lat=" << origin_position_.lat 
            << ", lon=" << origin_position_.lon 
            << ", alt=" << origin_position_.alt << "\n";
    outfile << "# Format: latitude(deg)  longitude(deg)  altitude(m)\n";

    // 逐行写入经纬度坐标（保留6位小数，满足厘米级精度）
    for (size_t i = 0; i < boundary_geo_.size(); ++i) {
        const auto& geo_point = boundary_geo_[i];
        outfile << std::fixed << std::setprecision(6)  // 经纬度保�?位小�?
                << geo_point.lat << "  "               // 纬度
                << geo_point.lon << "  "               // 经度
                << std::setprecision(3) << geo_point.alt  // 高度保留3位小�?
                << "\n";
    }

    outfile.close();
    ROS_INFO("Successfully saved %zu boundary points (geographic) to %s", 
             boundary_geo_.size(), file_path.c_str());
    return true;
}
//从文件中加载经纬�?
bool BoundaryCollector::LoadGeoBoundaryFromFile(const std::string& file_path) {
    ROS_INFO("Attempting to load boundary file: %s", file_path.c_str());
    
    // 参考原点信�?
    ROS_INFO("Current reference origin: (%.6f, %.6f, %.2f)",
             gps_.reference_origin_.lat, 
             gps_.reference_origin_.lon, 
             gps_.reference_origin_.alt);
    
    std::ifstream infile(file_path);
    if (!infile.is_open()) {
        ROS_ERROR("Failed to open file: %s", file_path.c_str());
        return false;
    }
    ROS_INFO("File opened successfully");

    // 清空现有数据
    boundary_geo_.clear();
    boundary_cart_.clear();
    boundary_path_.poses.clear();

    std::string line;
    int line_num = 0;
    int points_loaded = 0;
    bool is_first_point = true;  // 声明局部变�?
    GeoPoint first_point;        // 声明局部变�?
    
    while (std::getline(infile, line)) {
        line_num++;
        if (line.empty() || line[0] == '#') {
            ROS_DEBUG("Skipping comment/empty line %d", line_num);
            continue;
        }

        double lat, lon, alt;
        std::istringstream iss(line);
        if (!(iss >> lat >> lon >> alt)) {
            ROS_WARN("Invalid format in line %d: %s", line_num, line.c_str());
            continue;
        }

        ROS_INFO("Parsed point %d: lat=%.6f, lon=%.6f, alt=%.2f", 
                 points_loaded + 1, lat, lon, alt);

        // 存储地理坐标
        GeoPoint geo_point = {lat, lon, alt};
        boundary_geo_.push_back(geo_point);

         if (is_first_point) {
            first_point = geo_point;
            origin_position_ = geo_point;
            gps_.reference_origin_ = geo_point;
            is_origin_ = true;
            is_first_point = false;
            
            
            ROS_INFO("Set first point as origin: (%.6f, %.6f, %.2f)",
                     origin_position_.lat, origin_position_.lon, origin_position_.alt);
            
            // 发布 map -> origin_position 的TF变换
            geometry_msgs::TransformStamped transform;
            transform.header.stamp = ros::Time::now();
            transform.header.frame_id = "map";
            transform.child_frame_id = "gps_origin";
            transform.transform.translation.x = 0.0;
            transform.transform.translation.y = 0.0;
            transform.transform.translation.z = 0.0;
            transform.transform.rotation.w = 1.0;
            tf_broadcaster_.sendTransform(transform);
            
            ROS_INFO("TF transform published: map -> gps_origin");
        }

        // 坐标转换 - 添加详细日志
        CartesianPoint cart_point = gps_.GetCartFromPos(geo_point);
        boundary_cart_.push_back(cart_point);
        
        ROS_INFO("Converted to Cartesian: x=%.3f, y=%.3f, z=%.3f",
                 cart_point.x, cart_point.y, cart_point.z);

        // 添加到RViz路径
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.position.x = cart_point.x;
        pose.pose.position.y = cart_point.y;
        pose.pose.position.z = cart_point.z;
        pose.pose.orientation.w = 1.0;
        boundary_path_.poses.push_back(pose);
        
        points_loaded++;
    }

    infile.close();
    
    if (points_loaded > 0) {
        boundary_path_.header.stamp = ros::Time::now();
        boundary_path_pub_.publish(boundary_path_);
        ROS_INFO("Successfully loaded %d points. Published path with %zu poses", 
                 points_loaded, boundary_path_.poses.size());
        
        // 打印第一个和最后一个点的坐标用于调�?
        if (!boundary_cart_.empty()) {
            ROS_INFO("First point Cartesian: (%.3f, %.3f, %.3f)",
                     boundary_cart_[0].x, boundary_cart_[0].y, boundary_cart_[0].z);
            ROS_INFO("Last point Cartesian: (%.3f, %.3f, %.3f)",
                     boundary_cart_.back().x, boundary_cart_.back().y, boundary_cart_.back().z);
        }
    } else {
        ROS_WARN("No valid points were loaded from the file");
    }

    return points_loaded > 0;
}

// 点过滤算�?
std::vector<GeoPoint> BoundaryCollector::FilterPoints(const std::vector<GeoPoint>& points) 
{
    std::vector<GeoPoint> filtered_points;  
    if (points.empty()) {
        ROS_WARN("No points to filter");
        return filtered_points;
    }
    if (points.size() == 1) {
        ROS_WARN("Only one point, no filtering needed");
        filtered_points.push_back(points[0]);
        return filtered_points;
    }
    
    // 定义误差范围（可以改为从参数服务器读取）
    const double lower_threshold = 1.5;  // 最小允许距�?
    const double upper_threshold = 3.0;   // 最大允许距�?
    
    // 总是添加第一个点
    filtered_points.push_back(points[0]);
    ROS_DEBUG("Added first point: (%.6f, %.6f)", points[0].lat, points[0].lon);
    
    for (size_t i = 1; i < points.size(); ++i) {
        // 转换为笛卡尔坐标来计算距�?
        CartesianPoint last_cart = gps_.GetCartFromPos(filtered_points.back());
        CartesianPoint current_cart = gps_.GetCartFromPos(points[i]);
        
        double distance = CalculateDistance(last_cart, current_cart);
        ROS_DEBUG("Distance between point %zu and last: %.3f meters", i, distance);
        
        if (distance < lower_threshold) {  // 距离小于1.5米，去掉当前�?
            ROS_DEBUG("Removed point %zu (distance: %.2fm < %.2fm)", 
                     i, distance, lower_threshold);
        } 
        else if (distance > upper_threshold) {  // 距离大于3米，计算需要插入的点数
            ROS_DEBUG("Distance %.2fm > %.2fm, need to insert points", distance, upper_threshold);
            
            // 计算两点距离对基准值取余，再向下取整得到插入点�?
            double remainder = fmod(distance, filter_distance_threshold_);
            int insert_count = static_cast<int>(remainder);
            
            ROS_DEBUG("Distance modulo %.2f = %.2f, inserting %d points", 
                     filter_distance_threshold_, remainder, insert_count);
            
            // 插入计算得到的点�?
            for (int j = 1; j <= insert_count; ++j) {
                // 计算插值比�?
                double ratio = static_cast<double>(j) / (insert_count + 1);
                
                // 计算插值点
                GeoPoint mid_point;
                mid_point.lat = filtered_points.back().lat + 
                               (points[i].lat - filtered_points.back().lat) * ratio;
                mid_point.lon = filtered_points.back().lon + 
                               (points[i].lon - filtered_points.back().lon) * ratio;
                mid_point.alt = filtered_points.back().alt + 
                               (points[i].alt - filtered_points.back().alt) * ratio;
                
                filtered_points.push_back(mid_point);
                ROS_DEBUG("Inserted point %zu between %zu and %zu (ratio: %.2f)", 
                         filtered_points.size() - 1, filtered_points.size() - 2, i, ratio);
            }
            
            // 添加当前�?
            filtered_points.push_back(points[i]);
            ROS_DEBUG("Added point %zu after inserting intermediate points", i);
        }
        else {  // 距离�?.5米到3米之间，直接保留当前�?
            filtered_points.push_back(points[i]);
            ROS_DEBUG("Added point %zu (distance: %.2fm within [%.2f, %.2f]m range)", 
                     i, distance, lower_threshold, upper_threshold);
        }
    }
    
    // 记录详细的过滤结�?
    if (filtered_points.size() < points.size()) {
        ROS_INFO("Filtering completed: %zu points -> %zu points (removed %zu points)", 
                 points.size(), filtered_points.size(), points.size() - filtered_points.size());
    } 
    else if (filtered_points.size() > points.size()) {
        ROS_INFO("Filtering completed: %zu points -> %zu points (added %zu points)", 
                 points.size(), filtered_points.size(), filtered_points.size() - points.size());
    }
    else {
        ROS_INFO("No points filtered or added. All %zu points retained.", points.size());
    }
    
    return filtered_points;
}
    

//保存过滤后的�?
bool BoundaryCollector::SaveFilteredPointsToFile(const std::vector<GeoPoint>& filtered_points)
{
    if(filtered_points.empty()) {
        ROS_WARN("No filtered points to save");
        return false;
    }
    //保存地理坐标文件
    std::ofstream geo_outfile(filtered_output_path_);
    std::string cart_output_path = filtered_output_path_ + ".cartesian";  //笛卡尔坐标文�?

    if(!geo_outfile.is_open()) {
        ROS_ERROR("Failed to open filtered output file: %s",filtered_output_path_.c_str());
        return false;
    }

    std::ofstream cart_outfile(cart_output_path);
    if(!cart_outfile.is_open()) {
        ROS_ERROR("Failed to open cartesian output file: %s",cart_output_path.c_str());
        geo_outfile.close();
        return false;
    }

     // 写入地理坐标文件�?
    geo_outfile << "# Filtered boundary points (WGS84, distance threshold: " << filter_distance_threshold_ << "m)\n";
    geo_outfile << "# Origin: lat=" << origin_position_.lat 
                << ", lon=" << origin_position_.lon 
                << ", alt=" << origin_position_.alt << "\n";
    geo_outfile << "# Format: lat(deg) lon(deg) alt(m)\n";

     // 写入笛卡尔坐标文件头
    cart_outfile << "# Filtered boundary points (Cartesian ENU, distance threshold: " << filter_distance_threshold_ << "m)\n";
    cart_outfile << "# Origin: lat=" << origin_position_.lat 
                 << ", lon=" << origin_position_.lon 
                 << ", alt=" << origin_position_.alt << "\n";
    cart_outfile << "# Format: x(m) y(m) z(m)\n";
    
    for (size_t i = 0; i < filtered_points.size(); ++i) {
        const auto& geo_point = filtered_points[i];
        CartesianPoint cart_point = gps_.GetCartFromPos(geo_point);
        // 保存地理坐标
        geo_outfile << std::fixed << std::setprecision(9)
                    << geo_point.lat << "  "
                    << geo_point.lon << "  "
                    << std::setprecision(3) << geo_point.alt << "\n";
        
        // 保存笛卡尔坐�?
        cart_outfile << std::fixed << std::setprecision(3)
                     << i + 1 << "  "
                     << cart_point.x << "  "
                     << cart_point.y << "  "
                     << cart_point.z << "\n";
    }
    geo_outfile.close();
    cart_outfile.close();
    
    ROS_INFO("Successfully saved %zu filtered points:", filtered_points.size());
    ROS_INFO("  - Geographic coordinates: %s", filtered_output_path_.c_str());
    ROS_INFO("  - Cartesian coordinates: %s", cart_output_path.c_str());
    
    return true;

}

//实现加载和过�?
bool BoundaryCollector::LoadAndFilterGeoFile(const std::string& file_path)
{
    ROS_INFO("Loading and filtering geo file: %s",file_path.c_str());

    //加载原始文件
    if(!LoadGeoBoundaryFromFile(file_path)) {
        ROS_ERROR("Failed to load_geo_file for filtering");
        return false;
    }

    //过滤�?
    std::vector<GeoPoint> filtered_geo_points = FilterPoints(boundary_geo_);

    //清空原有数据，准备显示过滤后的点
    boundary_geo_.clear();
    boundary_cart_.clear();
    boundary_path_.poses.clear();

    //将过滤后的点转换显示出来
    for(const auto& geo_point : filtered_geo_points) {
        CartesianPoint  cart_point = gps_.GetCartFromPos(geo_point);
        boundary_geo_.push_back(geo_point);
        boundary_cart_.push_back(cart_point);

        //添加到Rviz
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.position.x = cart_point.x;
        pose.pose.position.y = cart_point.y;
        pose.pose.position.z = cart_point.z;
        pose.pose.orientation.w = 1.0;
        boundary_path_.poses.push_back(pose);

    }
      //发布过滤后的路径
        boundary_path_.header.stamp = ros::Time::now();
        boundary_path_pub_.publish(boundary_path_);

        //保存过滤后的点到文件
        bool save_success = SaveFilteredPointsToFile(filtered_geo_points);
        ROS_INFO("Filtering and visualization complete. %zu points remaining.", filtered_geo_points.size());
        return save_success;
}

//自动采集函数
void Map::BoundaryCollector::AutoCollectLoop() {
    ros::Rate rate(1.0 / auto_collect_interval_);  // 按配置间隔采�?
    ROS_INFO("Auto-collect thread started. Interval: %.1fs, Origin: %s",
             auto_collect_interval_,
             is_origin_ ? "set" : "not set");

    // 循环条件不变，确保线程可正常退�?
    while (is_auto_collecting_.load() && ros::ok()) {
        if (is_collecting_ && is_origin_) {
            bool success = AddCurrentPoint();
            if (success) {
                ROS_INFO("Auto-added boundary point. Total: %zu", boundary_cart_.size());
            } else {
                ROS_WARN("Failed to add point (too close to previous or invalid position)");
            }
        } 

        rate.sleep();
    }

    ROS_INFO("Auto-collect thread exited. Remaining points: %zu", boundary_cart_.size());
}

//开始采�?
void BoundaryCollector::StartCollecting() {
    if (!is_origin_) {
        ROS_WARN("Cannot start collecting: GPS data is invalid");
        return;
    }
    
    is_collecting_ = true;
    boundary_geo_.clear();
    boundary_cart_.clear();
    boundary_path_.poses.clear();
    ROS_INFO("Start collecting boundary points...");

    // �����Զ��ɼ��߳�
    if (!is_auto_collecting_) {
        is_auto_collecting_ = true;
        try {
            auto_collect_thread_ = std::thread(&BoundaryCollector::AutoCollectLoop, this);
            ROS_INFO("Auto-collect thread started (%.1fs interval)", auto_collect_interval_);
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to start auto-collect thread: %s", e.what());
            is_auto_collecting_ = false;
            is_collecting_ = false;
        }
    } else {
        ROS_INFO("Auto-collect thread already running");
    }
}
//停止采集
void BoundaryCollector::StopCollecting() {
    if (!is_collecting_) {
        return;
    }
    
    is_collecting_ = false;
    ROS_INFO("Stop collecting. Total boundary points: %zu", boundary_geo_.size());

    // ֹͣ�Զ��ɼ��߳�
    if (is_auto_collecting_) {
        is_auto_collecting_ = false;
        if (auto_collect_thread_.joinable()) {
            ROS_INFO("Waiting for auto-collect thread to finish...");
            auto_collect_thread_.join();
            ROS_INFO("Auto-collect thread stopped");
        }
    }

    // û�вɼ�����,ֱ���˳�,��ִ�б���
    if (boundary_geo_.empty()) {
        ROS_WARN("No boundary points collected, skip saving");
        return;
    }

    // �Զ��պϱ߽磨��������㹻��
    if (boundary_geo_.size() > 2) {
        double dist = CalculateDistance(boundary_cart_.front(), boundary_cart_.back());
        ROS_INFO("Boundary closed. Distance between first and last point: %.2f m", dist);
    }

    std::string auto_save_path = auto_save_path_;
    SaveBoundaryToFile(auto_save_path);
    ROS_INFO("Auto-saved geographic points to %s", auto_save_path.c_str());
}

// 添加当前位置
bool BoundaryCollector::AddCurrentPoint() {
    std::lock_guard<std::mutex> lock(point_mutex_);  // 加锁
     ROS_INFO("AddCurrentPoint: function called (is_collecting_=%d)", is_collecting_);  // 新增日志
    if (!is_collecting_) {
        ROS_WARN("Not collecting. Start collection first");
        return false;
    }

    GeoPoint current_geo = gps_.GetPosition();
    CartesianPoint current_cart = gps_.GetCartFromPos(current_geo);
    if(!IsPointValid(current_cart)){
        ROS_DEBUG("Point too close (distance < %.2f m), skipped", distance_threshold_);
        return false;
    }

    boundary_geo_.push_back(current_geo);  // 存储经纬�?
    boundary_cart_.push_back(current_cart);  // 存储大地坐标

    geometry_msgs::PoseStamped pose;  // 创建位姿消息
    pose.header.stamp = ros::Time::now();  // 时间�?
    pose.header.frame_id = boundary_path_.header.frame_id;  // 与路径坐标系一�?
    // 笛卡尔坐�?x,y,z)对应到位姿的位置
    pose.pose.position.x = current_cart.x;
    pose.pose.position.y = current_cart.y;
    pose.pose.position.z = current_cart.z;
    // 方向默认为原点（无需旋转�?
    pose.pose.orientation.w = 1.0;  // 四元数：无旋�?

    // 添加到位姿列�?
    boundary_path_.poses.push_back(pose);
    // 更新路径的时间戳
    boundary_path_.header.stamp = ros::Time::now();
    // 发布路径消息
    boundary_path_pub_.publish(boundary_path_);
    ROS_INFO("Added boundary point %zu: (%.6f, %.6f)", 
             boundary_geo_.size(),
             current_geo.lat, current_geo.lon);
    return true;
}
// 获取地理边界�?
std::vector<GeoPoint> BoundaryCollector::GetBoundaryGeoPoints() const {
    return boundary_geo_;
}

// 获取笛卡尔边界点
std::vector<CartesianPoint> BoundaryCollector::GetBoundaryCartPoints() const {
    return boundary_cart_;
}

// 清空边界�?
void BoundaryCollector::ClearBoundaryPoints() {
    boundary_geo_.clear();
    boundary_cart_.clear();
    ROS_INFO("Boundary points cleared");
}

bool BoundaryCollector::IsCollecting() const { 
    return is_collecting_; 
}

//判断采集点是否有�?
bool BoundaryCollector::IsPointValid(const CartesianPoint& new_point) {
    if (boundary_cart_.empty()) {
        return true;
    }

    const CartesianPoint& last_point = boundary_cart_.back();
    double dist = CalculateDistance(new_point, last_point);

    return dist > distance_threshold_;
}

double BoundaryCollector::CalculateDistance(const CartesianPoint& a, const CartesianPoint& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx*dx + dy*dy);
}

// 服务回调实现
bool BoundaryCollector::StartSrvCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (is_origin_) {  // 复用现有原点校验（原is_origin_对应新代码的is_origin_set_�?
        StartCollecting();  // 调用现有开始采集逻辑（清空数据等�?
        is_collecting_ = true;
        res.success = true;
        res.message = "Start collecting boundary points...";
        ROS_INFO("%s", res.message.c_str());

        // 启动自动采集线程（仅当线程未运行时）
        if (!is_auto_collecting_) {
            is_auto_collecting_ = true;
            auto_collect_thread_ = std::thread(&BoundaryCollector::AutoCollectLoop, this);
            ROS_INFO("Auto-collect thread started (%.1fs interval)", auto_collect_interval_);
        }
    } else {
        res.success = false;
        res.message = "GPS origin not set. Wait for valid GPS signal.";
        ROS_WARN("%s", res.message.c_str());
    }
    return true;
}

bool BoundaryCollector::StopSrvCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    StopCollecting();  // 调用现有停止采集逻辑
    res.success = true;
    res.message = "Stop collecting boundary points.";
    ROS_INFO("%s", res.message.c_str());

    // 停止自动采集线程（若已启动）
    if (is_auto_collecting_) {
        is_auto_collecting_ = false;
        if (auto_collect_thread_.joinable()) {
            auto_collect_thread_.join();
            ROS_INFO("Auto-collect thread stopped");
        }
    }
    return true;
}

bool BoundaryCollector::AddPointSrvCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    bool success = AddCurrentPoint();
    res.success = success;
    res.message = success ? "Point added" : "Failed to add point";
    return true;
}

//经纬度加载回调函�?
bool BoundaryCollector::LoadGeoSrvCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    std::string file_path ;
    nh_.param("load_geo_file_path", file_path, auto_save_path_); 
    bool success = LoadGeoBoundaryFromFile(file_path);
    res.success = success;
    res.message = success ? 
        "Geo boundary loaded from " + file_path : 
        "Failed to load geo boundary from " + file_path;
    return true;
}

//过滤加载回调
bool BoundaryCollector::LoadFilterSrvCallback(std_srvs::Trigger::Request& req,std_srvs::Trigger::Response& res)
{
    std::string file_path ;
    nh_.param("load_geo_file_path", file_path, auto_save_path_); 
    bool success = LoadAndFilterGeoFile(file_path);
    res.success = success;
    res.message = success ?
        "Geo boundary loaded, filtered and saved to " + filtered_output_path_ :
        "Failed to load and filter geo boundary";
        return true;

}
void BoundaryCollector::GpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) 
{
    // 检查GPS状态是否有�?- 建议使用 == 0 而不�?>= 0
    if (msg->status.status >= 0) {
        if(!is_origin_){
            // 使用SetOrigin函数来设置原�?
            GeoPoint gps_origin;
            gps_origin.lat = msg->latitude;
            gps_origin.lon = msg->longitude;
            gps_origin.alt = msg->altitude;
            
            if (SetOrigin(gps_origin)) {
                ROS_INFO("Origin successfully set from GPS callback");
            } else {
                ROS_WARN("Failed to set origin from GPS callback");
            }
        }
        
        // 更新当前位置
        gps_.current_pos_ = (GeoPoint){msg->latitude, msg->longitude, msg->altitude};
        ROS_DEBUG("Updated current position from GPS");
        
    } else if (msg->status.status > 0) {
        // SBAS或GBAS定位，可以更新位置但不设置原�?
        ROS_WARN_THROTTLE(10, "Using enhanced GPS positioning (status: %d)", msg->status.status);
        gps_.current_pos_ = (GeoPoint){msg->latitude, msg->longitude, msg->altitude};
    } else {
        ROS_WARN_THROTTLE(1, "GPS data invalid (status: %d)", msg->status.status);
    }
}

} // namespace Map
