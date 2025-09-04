#include "home_checker.h"
#include <cmath>

namespace homechecker{
HomeChecker::HomeChecker()
   : ros_com_{std::make_shared<rov_planning::RosCom>()}
{	

}

bool HomeChecker::IsHomeReached(const CartesianPoint& home_pos)
{
    ros_com_->GetCurrentPos(robot_pos_);
    double dx = robot_pos_.x - home_pos.x;
    double dy = robot_pos_.y - home_pos.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    return distance < 5;
}
}