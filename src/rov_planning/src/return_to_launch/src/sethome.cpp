#include "sethome.h"
#include <ros/console.h>

namespace gps {

bool SetHome::SetCurrentAsHome() {
    if (!gps_) {
        ROS_ERROR("SetHome: GPSPosition pointer is null");
        return false;
    }

   
    home_pos_ = gps_->Getposition();

    // 2. 调用GPSPosition设置原点（核心操作完全依赖现有功能）
    gps_->SetOrigin(home_pos_.lat, home_pos_.lon, home_pos_.alt);

    is_home_set_ = true;
    ROS_INFO("Home set to current GPS position: lat=%.6f, lon=%.6f, alt=%.2f",
             home_pos_.lat, home_pos_.lon, home_pos_.alt);
    return true;
}

} // namespace gps