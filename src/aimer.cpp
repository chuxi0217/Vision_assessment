#include "aimer.hpp"
#include <cmath>

namespace auto_aim {

void Aimer::update(const Armor& armor) {
    const Eigen::Vector3d& pos = armor.xyz_in_gimbal;
    
    // yaw：绕Z轴旋转，水平方向
    // atan2(Y, X)：目标在左侧(Y>0) → yaw>0（云台向左转）
    yaw_ = std::atan2(pos.y(), pos.x());
    
    // pitch：绕Y轴旋转，俯仰方向
    // atan2(Z, 水平距离)：目标在上方(Z>0) → pitch>0（云台抬头）
    double horizontal = std::sqrt(pos.x() * pos.x() + pos.y() * pos.y());
    pitch_ = std::atan2(pos.z(), horizontal);
}

double Aimer::yaw() const { return yaw_; }
double Aimer::pitch() const { return pitch_; }

} // namespace auto_aim