#ifndef SETHOME_H
#define SETHOME_H

#include "gps_postion.h"

namespace gps {

class SetHome {
public:
    explicit SetHome(GPSPosition* gps) : gps_(gps) {}
    ~SetHome() = default;

    // 禁止拷贝（避免指针管理问题）
    SetHome(const SetHome&) = delete;
    SetHome& operator=(const SetHome&) = delete;

    // 设置当前GPS位置为Home点（直接调用GPSPosition接口）
    bool SetCurrentAsHome();

    // 获取已设置的Home点
    GeoPoint GetHomePos() const { return home_pos_; }

    // 判断Home点是否已设置
    bool IsHomeSet() const { return is_home_set_; }

private:
    GPSPosition* gps_;       // 复用GPSPosition实例
    GeoPoint home_pos_ = {0, 0, 0};  // 存储Home点
    bool is_home_set_ = false;       // 状态标记
};

} // namespace gps

#endif // SETHOME_H