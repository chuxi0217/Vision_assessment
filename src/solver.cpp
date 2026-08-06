#include "solver.hpp"

#include <yaml-cpp/yaml.h>

namespace auto_aim
{

Solver::Solver(const std::string & config_path)
{
  // 1. 加载 yaml 配置文件
  auto yaml = YAML::LoadFile(config_path);

  // 2. 读取相机内参 camera_matrix
  //    yaml 里是一个包含 9 个数的列表，要转换成 3x3 的矩阵
  std::vector<double> camera_matrix_vec = yaml["camera_matrix"].as<std::vector<double>>();
  camera_matrix_ = cv::Mat(3, 3, CV_64F, camera_matrix_vec.data()).clone();

  // 3. 读取畸变系数 distort_coeffs
  //    yaml 里是 5 个数的列表，转换成 1x5 的矩阵
  std::vector<double> distort_coeffs_vec = yaml["distort_coeffs"].as<std::vector<double>>();
  distort_coeffs_ = cv::Mat(1, 5, CV_64F, distort_coeffs_vec.data()).clone();
}

void Solver::solve(Armor & armor) const
{
  // 4. 根据装甲板类型确定真实物理尺寸（单位：米）
  //    小装甲板：135mm x 55mm
  //    大装甲板：225mm x 55mm
  double half_w = 0.0;
  double half_h = 0.0;

  if (armor.type == ArmorType::small) {
    half_w = 0.135 / 2.0;
    half_h = 0.055 / 2.0;
  } else {  // ArmorType::big
    half_w = 0.225 / 2.0;
    half_h = 0.055 / 2.0;
  }

  // 5. 定义装甲板在物体坐标系下的 4 个 3D 角点
  //    坐标原点：装甲板中心
  //    x 轴：向右
  //    y 轴：向上
  //    z 轴：垂直装甲板向外（朝相机方向）
  //    顺序要和图像点 armor.points 对应：左上、右上、右下、左下
  std::vector<cv::Point3f> object_points = {
    cv::Point3f(-half_w,  half_h, 0.0),  // 左上
    cv::Point3f( half_w,  half_h, 0.0),  // 右上
    cv::Point3f( half_w, -half_h, 0.0),  // 右下
    cv::Point3f(-half_w, -half_h, 0.0)   // 左下
  };

  // 6. 图像上对应的 2D 角点（YOLO 已经检测出来了）
  std::vector<cv::Point2f> image_points = armor.points;

  // 7. 调用 OpenCV 的 solvePnP 解算位姿
  //    输入：3D 点、2D 点、相机内参、畸变系数
  //    输出：rvec（旋转向量）、tvec（平移向量）
  cv::Mat rvec, tvec;
  cv::solvePnP(
    object_points, image_points,
    camera_matrix_, distort_coeffs_,
    rvec, tvec,
    false,                       // 不使用初始猜测
    cv::SOLVEPNP_ITERATIVE);     // 迭代法，适合 4 个点的情况

  // 8. 把平移向量 tvec 保存到 armor 中
  //    tvec 就是装甲板中心在相机坐标系下的 3D 坐标（单位：米）
  armor.xyz_in_gimbal = Eigen::Vector3d(
    tvec.at<double>(0),
    tvec.at<double>(1),
    tvec.at<double>(2));
}

}  // namespace auto_aim
