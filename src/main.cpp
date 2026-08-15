#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "solver.hpp"
#include "tracker.hpp"
#include "yolov5.hpp"

using namespace auto_aim;
using namespace std;

/**
 * @brief 把装甲板中心+法向量展开成四个 3D 角点（云台坐标系）
 *
 * @param center  装甲板中心在云台坐标系下的位置（m）
 * @param normal  装甲板法向量（水平面内，指向车外/相机）
 * @param is_big  是否为大装甲板，决定板宽
 */
static vector<Eigen::Vector3d> buildArmorCorners(
  const Eigen::Vector3d & center, const Eigen::Vector3d & normal, bool is_big)
{
  double half_w = is_big ? (0.225 / 2.0) : (0.135 / 2.0);
  double half_h = 0.055 / 2.0;

  Eigen::Vector3d up(0.0, 0.0, 1.0);
  Eigen::Vector3d side = normal.cross(up).normalized();

  vector<Eigen::Vector3d> corners;
  corners.push_back(center + half_w * side + half_h * up);
  corners.push_back(center - half_w * side + half_h * up);
  corners.push_back(center - half_w * side - half_h * up);
  corners.push_back(center + half_w * side - half_h * up);
  return corners;
}

/**
 * @brief 将云台坐标系下的 3D 角点投影到图像 2D 坐标
 *
 * @param corners_gimbal 云台坐标系下的角点
 * @param solver         用于获取相机内参和云台→相机外参
 * @param img_size       图像尺寸，用于过滤越界点
 * @return 成功投影到图像内的 2D 角点；若任意角点在相机后方或越界则返回空
 */
static vector<cv::Point> projectCornersToImage(
  const vector<Eigen::Vector3d> & corners_gimbal, const Solver & solver,
  const cv::Size & img_size)
{
  Eigen::Matrix3d R_g2c = solver.R_gimbal2camera();
  Eigen::Vector3d t_g2c = solver.t_gimbal2camera();
  cv::Mat K = solver.cameraMatrix();
  double fx = K.at<double>(0, 0);
  double fy = K.at<double>(1, 1);
  double cx = K.at<double>(0, 2);
  double cy = K.at<double>(1, 2);

  vector<cv::Point> pts2d;
  for (const auto & c : corners_gimbal) {
    Eigen::Vector3d c_cam = R_g2c * c + t_g2c;
    if (c_cam.z() <= 0.05) {
      return {};  // 在相机后方或太近，整体不画
    }
    double u = c_cam.x() / c_cam.z();
    double v = c_cam.y() / c_cam.z();
    int x = static_cast<int>(fx * u + cx);
    int y = static_cast<int>(fy * v + cy);
    if (x < 0 || x >= img_size.width || y < 0 || y >= img_size.height) {
      return {};  // 越界，整体不画，避免画出不完整的框
    }
    pts2d.emplace_back(x, y);
  }
  return pts2d;
}

/**
 * @brief 在图像上绘制装甲板框
 */
static void drawArmorBox(
  cv::Mat & img, const vector<cv::Point> & pts, const cv::Scalar & color)
{
  if (pts.size() != 4) return;
  for (size_t i = 0; i < pts.size(); ++i) {
    cv::circle(img, pts[i], 4, color, -1);
    cv::line(img, pts[i], pts[(i + 1) % pts.size()], color, 2);
  }
}

/** @brief 将 TrackState 输出为可读的字符串 */
static string stateToString(TrackState s)
{
  switch (s) {
    case TrackState::LOST:
      return "LOST";
    case TrackState::DETECTING:
      return "DETECTING";
    case TrackState::TRACKING:
      return "TRACKING";
    case TrackState::TEMP_LOST:
      return "TEMP_LOST";
  }
  return "UNKNOWN";
}

int main(int argc, char ** argv)
{
  // 1. 配置文件和视频路径
  string config_path = "configs/infantry.yaml";
  string video_path = "assets/infantry.avi";

  // 2. 初始化检测器、PnP 解算器和 Tracker
  YOLOV5 yolo(config_path, false);
  Solver solver(config_path);

  // Tracker：状态机 + EKF，内部把装甲板观测转成整车底盘中心跟踪
  Tracker tracker;
  double t_last = -1.0;  // -1 表示还没有上一帧时间戳

  // 3. 打开视频
  cv::VideoCapture cap(video_path);
  if (!cap.isOpened()) {
    cerr << "Failed to open video: " << video_path << endl;
    return -1;
  }

  cv::Mat frame;
  int frame_count = 0;
  bool pause = false;
  bool hasFrame = false;

  // 5. 逐帧处理
  while (true) {
    if (!pause) {
      hasFrame = cap.read(frame);
      if (!hasFrame) break;
    }
    frame_count++;

    // 调用 YOLO 模型检测装甲板，返回一个装甲板列表
    auto armors_list = yolo.detect(frame, frame_count);

    // 修复 bug：先对所有装甲板做 PnP 解算，填充 3D 坐标/姿态/法向量
    for (auto & armor : armors_list) {
      solver.solve(armor);
    }

    // Tracker 需要 vector，复制一份
    std::vector<Armor> armors(armors_list.begin(), armors_list.end());

    // 6. 计算帧间隔 dt
    double t_now = static_cast<double>(cv::getTickCount()) / cv::getTickFrequency();
    double dt = (t_last < 0.0) ? 0.0 : (t_now - t_last);
    bool first_frame = (t_last < 0.0);
    t_last = t_now;

    // 过滤异常 dt，并把 dt 传给 Tracker 处理（内部负责第一帧重置、状态机切换）
    if (!first_frame && (dt <= 0.0 || dt > 0.5)) {
      cout << "[WARN] abnormal dt=" << dt << " s, skip predict" << endl;
      dt = 0.0;
    }
    tracker.update(armors, dt);

    // 7. 终端诊断：输出 Tracker 状态、底盘位置、速度
    TrackState state = tracker.state();
    Eigen::Vector3d pos = tracker.targetPosition();
    Eigen::Vector3d vel = tracker.velocity();
    cout << "frame=" << frame_count
         << " state=" << stateToString(state)
         << " pos=" << pos.transpose()
         << " vel=" << vel.transpose()
         << " |vel|=" << vel.norm() << endl;
    if (std::abs(vel.x()) >= 5.0 || std::abs(vel.y()) >= 5.0 ||
        std::abs(vel.z()) >= 5.0) {
      cout << "[WARN] velocity exceeds 5 m/s" << endl;
    }

    // 8. 整车观测：根据底盘中心和车辆航向推算四个装甲板位置，并用绿框绘制
    const Armor * tracked = tracker.trackedArmor();
    if ((state == TrackState::TRACKING || state == TrackState::TEMP_LOST) && tracked != nullptr) {
      Eigen::Vector3d chassis = tracker.targetPosition();
      vector<Eigen::Vector3d> armor_centers = tracker.getArmorPositions();
      bool is_big = (tracked->type == ArmorType::big);

      for (size_t i = 0; i < armor_centers.size(); ++i) {
        Eigen::Vector3d normal = (armor_centers[i] - chassis).normalized();
        auto corners = buildArmorCorners(armor_centers[i], normal, is_big);
        auto pts = projectCornersToImage(corners, solver, frame.size());
        drawArmorBox(frame, pts, cv::Scalar(0, 255, 0));
      }
    }

    // 9. 选择离相机最近的装甲板作为打击目标（红框）
    const Armor * target_armor = nullptr;
    if (!armors.empty()) {
      target_armor = &*std::min_element(
        armors.begin(), armors.end(), [](const Armor & a, const Armor & b) {
          return a.xyz_in_gimbal.norm() < b.xyz_in_gimbal.norm();
        });
    }

    // 10. 在 Tracker 稳定跟踪时，给最近装甲板画 0.5 s 预测红框
    if (target_armor != nullptr && state != TrackState::LOST) {
      Eigen::Vector3d chassis_now = tracker.targetPosition();
      Eigen::Vector3d chassis_pred = tracker.predictPosition(0.5);

      // 预测装甲板中心 = 预测底盘 + 当前装甲板相对于底盘的偏移
      Eigen::Vector3d armor_pred = chassis_pred + (target_armor->xyz_in_gimbal - chassis_now);

      Eigen::Vector3d normal = target_armor->normal_in_gimbal;
      normal.z() = 0.0;
      if (normal.norm() > 0.01) {
        normal.normalize();
      } else {
        normal = (target_armor->xyz_in_gimbal - chassis_now).normalized();
      }

      bool is_big = (target_armor->type == ArmorType::big);
      auto corners = buildArmorCorners(armor_pred, normal, is_big);
      auto pts = projectCornersToImage(corners, solver, frame.size());

      if (!pts.empty()) {
        drawArmorBox(frame, pts, cv::Scalar(0, 0, 255));
      } else {
        cout << "[WARN] predicted red box outside image or behind camera" << endl;
      }

      // 11. 显示文字信息（用最近装甲板的距离和底盘角度）
      double dist = target_armor->xyz_in_gimbal.norm();
      cv::putText(
        frame, cv::format("%.2f m", dist), target_armor->center,
        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);

      double yaw_now = std::atan2(chassis_now.y(), chassis_now.x());
      double horizontal_now = std::sqrt(chassis_now.x() * chassis_now.x() + chassis_now.y() * chassis_now.y());
      double pitch_now = std::atan2(chassis_now.z(), horizontal_now);
      double yaw_deg = yaw_now * 180.0 / CV_PI;
      double pitch_deg = pitch_now * 180.0 / CV_PI;
      cv::putText(
        frame, cv::format("Y:%.1f P:%.1f", yaw_deg, pitch_deg),
        cv::Point(target_armor->center.x, target_armor->center.y - 20),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
    }

    cv::putText(
      frame, cv::format("S:%s", stateToString(state).c_str()), cv::Point(10, 30),
      cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
    if (state != TrackState::LOST) {
      Eigen::Vector3d pred_pos = tracker.predictPosition(0.5);
      double yaw_pred = std::atan2(pred_pos.y(), pred_pos.x());
      cv::putText(
        frame, cv::format("Yp:%.1f", yaw_pred * 180.0 / CV_PI), cv::Point(10, 60),
        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }

    // 12. 显示画面
    cv::imshow("Vision Assessment", frame);

    // 按 ESC 退出，空格暂停
    int key = cv::waitKey(30);
    if (key == 27) {
      break;
    } else if (key == 32) {
      pause = !pause;
    }
  }

  cap.release();
  cv::destroyAllWindows();
  return 0;
}
