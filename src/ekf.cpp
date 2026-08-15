#include "ekf.hpp"

namespace auto_aim {

/**
 * @brief 构造函数：把状态、协方差和噪声矩阵初始化为合理初值
 *
 * 状态 x = [x, y, z, vx, vy, vz]^T，初始全 0（表示还没有目标）。
 * 协方差 P = 10*I，表示初始不确定性较大。
 * 过程噪声 Q 的前三维对应位置扰动，后三维对应速度扰动，
 * 数值上速度比位置更难保持恒定，所以速度项噪声更大。
 * 观测噪声 R 对应 PnP 解算出的 x/y/z 的测量不确定度。
 */
EKF::EKF()
{
  x_ = Eigen::VectorXd::Zero(N_STATE);
  P_ = Eigen::MatrixXd::Identity(N_STATE, N_STATE) * 10.0;

  Q_ = Eigen::MatrixXd::Zero(N_STATE, N_STATE);
  // 过程噪声：位置扰动较小，速度扰动适当降低以抑制 PnP 抖动导致的速度尖峰
  Q_.diagonal() << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5;

  R_ = Eigen::MatrixXd::Zero(N_OBS, N_OBS);
  R_.diagonal() << 0.05, 0.05, 0.1;
}

/**
 * @brief 预测步：假设目标匀速直线运动，推进 dt 秒
 *
 * 状态转移矩阵 F 在 identity 基础上把速度积分到位置：
 *   x_{k+1} = x_k + vx*dt
 *   y_{k+1} = y_k + vy*dt
 *   z_{k+1} = z_k + vz*dt
 * 速度保持不变。
 * 然后协方差按 P = F*P*F^T + Q 传播。
 */
void EKF::predict(double dt)
{
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(N_STATE, N_STATE);
  F(0, 3) = dt;  // x = x + vx*dt
  F(1, 4) = dt;  // y = y + vy*dt
  F(2, 5) = dt;  // z = z + vz*dt

  x_ = F * x_;
  P_ = F * P_ * F.transpose() + Q_;
}

/**
 * @brief 更新步：融合 PnP 观测到的 3D 位置
 *
 * 观测矩阵 H = [I3 | 0]，只观测位置，不观测速度。
 * 标准卡尔曼滤波：
 *   y = z - H*x        新息（观测与预测的差异）
 *   S = H*P*H^T + R    新息协方差
 *   K = P*H^T*S^{-1}   卡尔曼增益
 *   x = x + K*y        状态修正
 *   P = (I - K*H)*P    协方差修正
 */
void EKF::update(const Eigen::Vector3d & obs)
{
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(N_OBS, N_STATE);
  H.block(0, 0, N_OBS, N_OBS) = Eigen::MatrixXd::Identity(N_OBS, N_OBS);

  Eigen::Vector3d y = obs - H * x_;           // 新息
  Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
  Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();  // 卡尔曼增益

  x_ = x_ + K * y;
  P_ = (Eigen::MatrixXd::Identity(N_STATE, N_STATE) - K * H) * P_;
}

/** @brief 返回当前估计的位置 [x, y, z] */
Eigen::Vector3d EKF::position() const { return x_.head<3>(); }

/** @brief 返回当前估计的速度 [vx, vy, vz] */
Eigen::Vector3d EKF::velocity() const { return x_.tail<3>(); }

/**
 * @brief 预测未来 dt 秒后的位置，不修改内部状态
 *
 * 基于匀速模型：p_future = p_now + v * dt。
 * 用于把预测位置反投影到图像上画红框。
 */
Eigen::Vector3d EKF::predictPosition(double dt) const
{
  return position() + velocity() * dt;
}

/**
 * @brief 重置 EKF：用于确认换了新目标，或第一帧初始化
 *
 * 位置设为 init_pos，速度清零，协方差重新放大到 10*I，
 * 表示对新目标的不确定性较大。
 */
void EKF::reset(const Eigen::Vector3d & init_pos)
{
  x_.setZero();
  x_.head<3>() = init_pos;
  P_ = Eigen::MatrixXd::Identity(N_STATE, N_STATE) * 10.0;
}

/**
 * @brief 异常检测：观测与当前估计位置差异是否过大
 *
 * 如果欧氏距离 > 1.0 m，认为可能是换了目标或检测错误，返回 true。
 */
bool EKF::isOutlier(const Eigen::Vector3d & obs) const
{
  return (obs - position()).norm() > 1.0;
}

}  // namespace auto_aim
