#ifndef AUTO_AIM_TRACKER_HPP
#define AUTO_AIM_TRACKER_HPP

#pragma once
#include <vector>
#include "ekf.hpp"
#include "armor.hpp"

namespace auto_aim {

enum class TrackState {
    LOST,      // 未跟踪任何目标
    DETECTING, // 刚发现，需要连续几帧确认
    TRACKING,  // 稳定跟踪中
    TEMP_LOST  // 暂时丢失（可能被遮挡）
};

class Tracker {
public:
    Tracker();

    // 主入口：处理一帧检测结果
    // 传入：armors = YOLO 检测到的所有装甲板
    //       dt     = 距离上一帧的时间（秒）
    void update(const std::vector<Armor>& armors, double dt);

    // 获取当前追踪状态
    TrackState state() const;

    // 获取当前目标底盘中心位置（用于计算 yaw/pitch）
    Eigen::Vector3d targetPosition() const;

    // 获取当前目标速度（用于诊断）
    Eigen::Vector3d velocity() const;

    // 获取预测位置（画红框用）
    // 传入：dt = 预测到未来多少秒
    Eigen::Vector3d predictPosition(double dt) const;

    // 获取当前选中的装甲板（画绿框用）
    // 返回：nullptr 表示没有选中
    const Armor* trackedArmor() const;

    // 获取车辆四个装甲板估计位置（基于底盘中心 + 车辆航向），
    // 用于整车观测：即使不在视野内也可推算。
    std::vector<Eigen::Vector3d> getArmorPositions() const;

    // 手动重置
    void reset();

private:
    EKF ekf_;
    TrackState state_ = TrackState::LOST;
    Armor tracked_armor_;        // 当前画绿框的装甲板
    int detect_count_ = 0;       // 连续检测到计数（用于 DETECTING→TRACKING）
    int lost_count_ = 0;         // 连续丢失计数
    const int DETECT_THRESHOLD = 3;  // 连续3帧确认目标
    const int LOST_THRESHOLD = 5;    // 连续5帧丢失认为目标消失

    double vehicle_yaw_ = 0.0;   // 车辆在水平面内的航向（rad），用于推算四个装甲板位置
    bool yaw_initialized_ = false;

    // 数据关联：找到与预测位置最接近的装甲板
    // 返回：索引，-1 表示无匹配
    int findBestMatch(const std::vector<Armor>& armors,
                      const Eigen::Vector3d& predicted_pos) const;

    // 找到与 best_idx 属于同一车辆的其他装甲板
    std::vector<int> findCluster(const std::vector<Armor>& armors, int best_idx) const;

    // 装甲板位置 → 底盘中心位置
    // 根据装甲板法向量向车体内部偏移一段距离（small≈0.30m，big≈0.40m）
    Eigen::Vector3d armorToChassis(const Armor& armor) const;

    // 多个装甲板属于同一车辆时，计算它们底盘中心的平均观测
    Eigen::Vector3d averageChassisPosition(
      const std::vector<Armor>& armors,
      const std::vector<int>& indices) const;

    // 更新车辆航向：优先用检测到装甲板的法向量，丢失时用速度方向
    void updateVehicleYaw(const Armor& matched_armor, double dt);
};

} // namespace auto_aim

#endif