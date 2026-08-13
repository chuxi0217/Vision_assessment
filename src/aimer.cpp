#include "aimer.hpp"
#include <cmath>

namespace auto_aim {

void Aimer::update(const Armor& armor) {
    const Eigen::Vector3d& pos = armor.xyz_in_gimbal;//’&‘给xyz_in_gimbal起一个别名pos(position)，“const”承诺只读
    //云台坐标系是z轴向上，y轴向左，x轴向前
    // yaw：绕Z轴旋转，水平方向，朝向与x轴正向的夹角
    // atan2(Y, X)：目标在左侧(Y>0) → yaw>0（云台向左转）
    yaw_ = std::atan2(pos.y(), pos.x());//atan2即arctan,std::atan2(对边，临边)
    
    // pitch：绕Y轴旋转，俯仰方向，朝向与xy平面的夹角
    // atan2(Z, 水平距离)：目标在上方(Z>0) → pitch>0（云台抬头）
    double horizontal = std::sqrt(pos.x() * pos.x() + pos.y() * pos.y());
    pitch_ = std::atan2(pos.z(), horizontal);
}

double Aimer::yaw() const { return yaw_; }
double Aimer::pitch() const { return pitch_; }

} // namespace auto_aim