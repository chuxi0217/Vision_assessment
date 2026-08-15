#ifndef AUTO_AIM_EKF_HPP
#define AUTO_AIM_EKF_HPP

#pragma once
#include <Eigen/Dense>

namespace auto_aim {

class EKF {
public:
    EKF();

    // 预测步：假设匀速直线运动，推进 dt 秒
    // 传入：dt（秒），>0
    void predict(double dt);

    // 更新步：融合 PnP 观测
    // 传入：obs = 装甲板/底盘的 3D 位置（云台坐标系，米）
    void update(const Eigen::Vector3d& obs);

    // 获取当前估计的位置
    Eigen::Vector3d position() const;

    // 获取当前估计的速度
    Eigen::Vector3d velocity() const;

    // 预测未来 dt 秒后的位置（用于画红框）
    // 传入：dt（如 0.1 表示预测 100ms 后）
    // 返回：3D 位置
    Eigen::Vector3d predictPosition(double dt) const;

    // 重置：确认换了目标时调用
    // 传入：新目标的初始位置（通常用第一帧 PnP 结果）
    void reset(const Eigen::Vector3d& init_pos);

    // 异常检测：观测与预测的差异是否过大
    // 传入：新的观测位置
    // 返回：true = 可能是换了目标或检测错误
    bool isOutlier(const Eigen::Vector3d& obs) const;

private:
    static constexpr int N_STATE = 6;  // [x,y,z,vx,vy,vz]
    static constexpr int N_OBS   = 3;  // [x,y,z]

    Eigen::VectorXd x_;  // 6x1 状态
    Eigen::MatrixXd P_;  // 6x6 协方差
    Eigen::MatrixXd Q_;  // 6x6 过程噪声
    Eigen::MatrixXd R_;  // 3x3 观测噪声
};

} // namespace auto_aim


#endif