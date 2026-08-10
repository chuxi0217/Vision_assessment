/**
 * @file math_tools.hpp
 * @brief 数学与几何计算工具函数集合
 *
 * 本文件提供自瞄系统中常用的数学工具函数，包括：
 * - 角度归一化与限制
 * - 四元数/旋转矩阵与欧拉角之间的转换
 * - 直角坐标系与球坐标系转换及雅可比矩阵
 * - 时间差计算、向量夹角、平方函数、数值限幅等
 */

#ifndef TOOLS__MATH_TOOLS_HPP
#define TOOLS__MATH_TOOLS_HPP

#include <Eigen/Geometry>  // Eigen 旋转矩阵、四元数、欧拉角等几何工具
#include <chrono>          // 时间库，用于计算时间差

namespace tools
{

/**
 * @brief 将弧度值限制在 (-π, π] 区间
 * @param angle 输入弧度值
 * @return 归一化后的弧度值
 *
 * 常用于处理 yaw/pitch 角的周期性跳变，使其适合差值计算和控制。
 */
double limit_rad(double angle);

/**
 * @brief 将四元数转换为欧拉角
 * @param q 输入四元数
 * @param axis0 第一次旋转轴，x=0, y=1, z=2
 * @param axis1 第二次旋转轴
 * @param axis2 第三次旋转轴
 * @param extrinsic 是否为外旋（被动旋转），默认 false（内旋/主动旋转）
 * @return 欧拉角向量（单位：弧度）
 *
 * 例如：先绕 z 轴，再绕 y 轴，最后绕 x 轴，应传入 axis0=2, axis1=1, axis2=0。
 * 参考实现：https://github.com/evbernardes/quaternion_to_euler
 */
Eigen::Vector3d eulers(
  Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic = false);

/**
 * @brief 将旋转矩阵转换为欧拉角
 * @param R 输入 3x3 旋转矩阵
 * @param axis0 第一次旋转轴，x=0, y=1, z=2
 * @param axis1 第二次旋转轴
 * @param axis2 第三次旋转轴
 * @param extrinsic 是否为外旋，默认 false
 * @return 欧拉角向量（单位：弧度）
 */
Eigen::Vector3d eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic = false);

/**
 * @brief 将欧拉角转换为旋转矩阵
 * @param ypr 欧拉角向量（yaw, pitch, roll 顺序），单位：弧度
 * @return 3x3 旋转矩阵
 *
 * 假设旋转顺序为 zyx：先绕 z 轴旋转（yaw），再绕 y 轴旋转（pitch），最后绕 x 轴旋转（roll）。
 */
Eigen::Matrix3d rotation_matrix(const Eigen::Vector3d & ypr);

/**
 * @brief 将直角坐标系（xyz）转换为球坐标系（ypd）
 * @param xyz 三维直角坐标向量，单位：米
 * @return 球坐标向量 [yaw, pitch, distance]，单位：弧度/米
 *
 * ypd 为 yaw、pitch、distance 的缩写。
 */
Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz);

/**
 * @brief 计算 xyz2ypd 转换函数对 xyz 的雅可比矩阵
 * @param xyz 三维直角坐标向量
 * @return 雅可比矩阵 J，满足 d(ypd) = J * d(xyz)
 *
 * 主要用于扩展卡尔曼滤波（EKF）或误差传播分析。
 */
Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d & xyz);

/**
 * @brief 将球坐标系（ypd）转换为直角坐标系（xyz）
 * @param ypd 球坐标向量 [yaw, pitch, distance]，单位：弧度/米
 * @return 三维直角坐标向量，单位：米
 */
Eigen::Vector3d ypd2xyz(const Eigen::Vector3d & ypd);

/**
 * @brief 计算 ypd2xyz 转换函数对 ypd 的雅可比矩阵
 * @param ypd 球坐标向量
 * @return 雅可比矩阵 J，满足 d(xyz) = J * d(ypd)
 */
Eigen::MatrixXd ypd2xyz_jacobian(const Eigen::Vector3d & ypd);

/**
 * @brief 计算两个 steady_clock 时间点之间的时间差
 * @param a 较晚时间点
 * @param b 较早时间点
 * @return 时间差，单位：秒（a - b）
 */
double delta_time(
  const std::chrono::steady_clock::time_point & a, const std::chrono::steady_clock::time_point & b);

/**
 * @brief 计算两个二维向量之间的绝对夹角
 * @param vec1 第一个向量
 * @param vec2 第二个向量
 * @return 向量夹角，范围 [0, π]，单位：弧度
 */
double get_abs_angle(const Eigen::Vector2d & vec1, const Eigen::Vector2d & vec2);

/**
 * @brief 计算输入值的平方
 * @tparam T 数值类型
 * @param a 输入值
 * @return a * a
 */
template <typename T>
T square(T const & a)
{
  return a * a;
};

/**
 * @brief 将输入值限制在 [min, max] 区间
 * @param input 输入值
 * @param min 下限
 * @param max 上限
 * @return 限幅后的值
 *
 * 若 input < min 返回 min；若 input > max 返回 max；否则返回 input。
 */
double limit_min_max(double input, double min, double max);

}  // namespace tools

#endif  // TOOLS__MATH_TOOLS_HPP
