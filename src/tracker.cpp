#include "tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace auto_aim {

Tracker::Tracker() : state_(TrackState::LOST), detect_count_(0), lost_count_(0) {}

/**
 * @brief 主入口：处理一帧检测结果，维护状态机
 *
 * 状态转移：
 *   LOST      -> 收到检测 -> DETECTING（记录候选装甲板）
 *   DETECTING -> 连续3帧同一目标（位置距离<0.5m） -> TRACKING（初始化 EKF）
 *   DETECTING -> 中途丢失 -> LOST
 *   TRACKING  -> 每帧 predict + findBestMatch，匹配成功 update，失败 lost_count++
 *   TRACKING  -> lost_count>5 -> TEMP_LOST
 *   TEMP_LOST -> 继续 predict，5帧内重新匹配 -> TRACKING，否则 -> LOST
 *
 * 本实现将 EKF 状态从“装甲板中心”提升到“整车底盘中心”：
 * 用装甲板法向量把装甲板中心向车体内部偏移一段距离，
 * 并对同一车辆上多个装甲板做平均，降低单块装甲板 PnP 噪声。
 */
void Tracker::update(const std::vector<Armor> & armors, double dt)
{
  switch (state_) {
    case TrackState::LOST: {
      if (!armors.empty()) {
        // 发现目标，进入确认阶段，记录第一个装甲板作为候选
        tracked_armor_ = armors.front();
        state_ = TrackState::DETECTING;
        detect_count_ = 1;
      }
      break;
    }

    case TrackState::DETECTING: {
      if (armors.empty()) {
        // 中途丢失，回到 LOST
        state_ = TrackState::LOST;
        detect_count_ = 0;
        break;
      }

      // 找与当前候选底盘位置最接近的装甲板
      Eigen::Vector3d candidate_chassis = armorToChassis(tracked_armor_);
      int best_idx = findBestMatch(armors, candidate_chassis);

      if (best_idx < 0) {
        // 没有足够近的匹配，认为目标丢失
        state_ = TrackState::LOST;
        detect_count_ = 0;
        break;
      }

      const Armor & matched = armors[best_idx];
      Eigen::Vector3d matched_chassis = armorToChassis(matched);

      if ((matched_chassis - candidate_chassis).norm() < 0.5) {
        // 连续同一目标，确认计数 +1
        tracked_armor_ = matched;
        detect_count_++;
        if (detect_count_ >= DETECT_THRESHOLD) {
          // 连续 3 帧确认，进入稳定跟踪并初始化 EKF
          state_ = TrackState::TRACKING;
          ekf_.reset(matched_chassis);
          updateVehicleYaw(matched, dt);
          lost_count_ = 0;
        }
      } else {
        // 位置差异过大，换成新候选，重新计数
        tracked_armor_ = matched;
        detect_count_ = 1;
      }
      break;
    }

    case TrackState::TRACKING: {
      // 即使暂时没有匹配，也要按匀速模型继续推进状态
      if (dt > 0.0) {
        ekf_.predict(dt);
      }

      Eigen::Vector3d predicted_chassis = ekf_.predictPosition(0.0);
      int best_idx = findBestMatch(armors, predicted_chassis);

      if (best_idx >= 0) {
        // 匹配成功，对同一车辆多个装甲板做平均观测，融合到 EKF
        const Armor & matched = armors[best_idx];
        std::vector<int> cluster_indices = findCluster(armors, best_idx);
        Eigen::Vector3d obs = averageChassisPosition(armors, cluster_indices);

        if (!ekf_.isOutlier(obs)) {
          ekf_.update(obs);
        }
        tracked_armor_ = matched;
        updateVehicleYaw(matched, dt);
        lost_count_ = 0;
      } else {
        // 连续丢失计数
        lost_count_++;
        if (lost_count_ > LOST_THRESHOLD) {
          // 超过 5 帧未匹配，进入暂时丢失状态
          state_ = TrackState::TEMP_LOST;
          lost_count_ = 0;
        }
      }
      break;
    }

    case TrackState::TEMP_LOST: {
      // 继续按匀速模型预测，给目标被遮挡后重新出现留出时间窗口
      if (dt > 0.0) {
        ekf_.predict(dt);
      }

      Eigen::Vector3d predicted_chassis = ekf_.predictPosition(0.0);
      int best_idx = findBestMatch(armors, predicted_chassis);

      if (best_idx >= 0) {
        // 重新匹配成功，恢复跟踪
        const Armor & matched = armors[best_idx];
        std::vector<int> cluster_indices = findCluster(armors, best_idx);
        Eigen::Vector3d obs = averageChassisPosition(armors, cluster_indices);

        if (!ekf_.isOutlier(obs)) {
          ekf_.update(obs);
        }
        tracked_armor_ = matched;
        state_ = TrackState::TRACKING;
        updateVehicleYaw(matched, dt);
        lost_count_ = 0;
      } else {
        lost_count_++;
        if (lost_count_ > LOST_THRESHOLD) {
          // 5 帧内仍未恢复，认为目标彻底丢失
          state_ = TrackState::LOST;
          lost_count_ = 0;
        }
      }
      break;
    }
  }
}

TrackState Tracker::state() const { return state_; }

/** @brief 返回当前目标底盘中心位置（用于计算 yaw/pitch） */
Eigen::Vector3d Tracker::targetPosition() const
{
  if (state_ == TrackState::TRACKING || state_ == TrackState::TEMP_LOST) {
    return ekf_.position();
  }
  return armorToChassis(tracked_armor_);
}

/** @brief 返回当前目标速度（用于终端诊断） */
Eigen::Vector3d Tracker::velocity() const
{
  if (state_ == TrackState::TRACKING || state_ == TrackState::TEMP_LOST) {
    return ekf_.velocity();
  }
  return Eigen::Vector3d::Zero();
}

/** @brief 返回预测未来 dt 秒后的位置（用于画红框） */
Eigen::Vector3d Tracker::predictPosition(double dt) const
{
  if (state_ == TrackState::TRACKING || state_ == TrackState::TEMP_LOST) {
    return ekf_.predictPosition(dt);
  }
  return armorToChassis(tracked_armor_);
}

/** @brief 返回当前选中的装甲板（画绿框用） */
const Armor * Tracker::trackedArmor() const
{
  if (state_ == TrackState::LOST) {
    return nullptr;
  }
  return &tracked_armor_;
}

void Tracker::reset()
{
  state_ = TrackState::LOST;
  detect_count_ = 0;
  lost_count_ = 0;
  yaw_initialized_ = false;
  vehicle_yaw_ = 0.0;
  ekf_.reset(Eigen::Vector3d::Zero());
}

/**
 * @brief 数据关联：找到与预测底盘位置最接近且距离小于 1.0 m 的装甲板
 *
 * 返回最近装甲板在 armors 中的索引；没有任何装甲板在阈值内则返回 -1。
 */
int Tracker::findBestMatch(const std::vector<Armor> & armors,
                           const Eigen::Vector3d & predicted_pos) const
{
  int best_idx = -1;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < armors.size(); ++i) {
    Eigen::Vector3d chassis_pos = armorToChassis(armors[i]);
    double dist = (chassis_pos - predicted_pos).norm();
    if (dist < min_dist && dist < 1.0) {
      min_dist = dist;
      best_idx = static_cast<int>(i);
    }
  }

  return best_idx;
}

/**
 * @brief 找到与 best_idx 装甲板属于同一车辆的其他装甲板
 *
 * 判断标准：两块装甲板的底盘中心距离 < 0.8 m（同一车辆不同装甲板的大致间距）。
 * 返回包含 best_idx 的索引列表。
 */
std::vector<int> Tracker::findCluster(const std::vector<Armor> & armors,
                                      int best_idx) const
{
  std::vector<int> indices;
  Eigen::Vector3d center_chassis = armorToChassis(armors[best_idx]);

  for (size_t i = 0; i < armors.size(); ++i) {
    Eigen::Vector3d chassis_pos = armorToChassis(armors[i]);
    if ((chassis_pos - center_chassis).norm() < 0.8) {
      indices.push_back(static_cast<int>(i));
    }
  }

  return indices;
}

/**
 * @brief 计算多个装甲板底盘中心的平均位置
 *
 * 用于同一车辆上多个装甲板同时被检测到时，降低单块装甲板 PnP 噪声。
 */
Eigen::Vector3d Tracker::averageChassisPosition(const std::vector<Armor> & armors,
                                                const std::vector<int> & indices) const
{
  if (indices.empty()) {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (int idx : indices) {
    sum += armorToChassis(armors[idx]);
  }
  return sum / static_cast<double>(indices.size());
}

/**
 * @brief 装甲板位置 → 底盘中心位置
 *
 * 根据装甲板法向量（指向车外/相机）向车体内部偏移：
 *   - small 装甲板：步兵/哨兵等，偏移约 0.30 m
 *   - big 装甲板：英雄/基地等，偏移约 0.40 m
 * 若法向量未解算（norm 接近 0），则回退到直接用装甲板中心。
 */
Eigen::Vector3d Tracker::armorToChassis(const Armor & armor) const
{
  if (armor.normal_in_gimbal.norm() < 0.01) {
    return armor.xyz_in_gimbal;
  }

  // 对地面车辆，底盘中心与装甲板中心的高度差很小；
  // 主要偏移在水平方向（车体内部）。把法向量投影到水平面，
  // 避免装甲板俯仰/倾斜带来的 z 方向噪声被放大。
  Eigen::Vector3d horizontal_normal = armor.normal_in_gimbal;
  horizontal_normal.z() = 0.0;
  if (horizontal_normal.norm() < 0.01) {
    return armor.xyz_in_gimbal;
  }
  horizontal_normal.normalize();

  double offset = (armor.type == ArmorType::big) ? 0.40 : 0.30;
  return armor.xyz_in_gimbal - offset * horizontal_normal;
}

/**
 * @brief 更新车辆水平航向
 *
 * 优先使用当前匹配到装甲板的法向量作为车辆朝向参考；
 * 当装甲板被遮挡或短暂丢失时，用 EKF 估计的速度方向辅助修正航向。
 * 使用一阶低通滤波，避免单帧 PnP 抖动导致四个装甲板位置剧烈跳动。
 */
void Tracker::updateVehicleYaw(const Armor & matched_armor, double dt)
{
  Eigen::Vector3d normal = matched_armor.normal_in_gimbal;
  normal.z() = 0.0;

  double measured_yaw = 0.0;
  bool has_measurement = false;
  if (normal.norm() > 0.01) {
    normal.normalize();
    measured_yaw = std::atan2(normal.y(), normal.x());
    has_measurement = true;
  }

  if (!yaw_initialized_) {
    if (has_measurement) {
      vehicle_yaw_ = measured_yaw;
      yaw_initialized_ = true;
    }
    return;
  }

  // 有装甲板法向量观测时，用较快的滤波跟上目标转向
  double alpha = 0.30;
  if (has_measurement) {
    double dyaw = measured_yaw - vehicle_yaw_;
    dyaw = std::atan2(std::sin(dyaw), std::cos(dyaw));
    vehicle_yaw_ += alpha * dyaw;
  }

  // 同时用速度方向做弱约束：车辆大致朝运动方向前进
  Eigen::Vector3d vel = ekf_.velocity();
  vel.z() = 0.0;
  if (vel.norm() > 0.2) {
    double velocity_yaw = std::atan2(vel.y(), vel.x());
    double dyaw = velocity_yaw - vehicle_yaw_;
    dyaw = std::atan2(std::sin(dyaw), std::cos(dyaw));
    double beta = 0.05;  // 弱权重，避免速度噪声主导
    vehicle_yaw_ += beta * dyaw;
  }
}

/**
 * @brief 推算车辆四个装甲板的三维位置
 *
 * 基于底盘中心（EKF 估计）和车辆航向，把四个装甲板摆放在车体四周。
 * 即使某些装甲板当前不在视野内，也能给出估计位置。
 * 装甲板间距由当前跟踪到的装甲板类型决定：small≈0.30m，big≈0.40m。
 */
std::vector<Eigen::Vector3d> Tracker::getArmorPositions() const
{
  std::vector<Eigen::Vector3d> positions;
  if (state_ == TrackState::LOST || !yaw_initialized_) {
    return positions;
  }

  Eigen::Vector3d chassis = targetPosition();
  double offset = 0.30;
  if (tracked_armor_.type == ArmorType::big) {
    offset = 0.40;
  }

  // 四块装甲板分别位于车辆前、右、后、左（间隔 90°）
  for (int i = 0; i < 4; ++i) {
    double angle = vehicle_yaw_ + i * M_PI / 2.0;
    Eigen::Vector3d normal(std::cos(angle), std::sin(angle), 0.0);
    positions.push_back(chassis + offset * normal);
  }

  return positions;
}

}  // namespace auto_aim
