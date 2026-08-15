#include "solver.hpp"
#include <yaml-cpp/yaml.h>

namespace auto_aim {

// 构造函数：从 yaml 文件读取相机内参和畸变系数，保存到成员变量
Solver::Solver(const std::string & config_path)
{
    // 1.1 加载 yaml 文件
    auto yaml = YAML::LoadFile(config_path);
    
    // 1.2 读取 camera_matrix 相机内参（yaml 里是 9 个数的列表）
    // yaml["camera_matrix"].as<std::vector<double>>() 可以读出 vector<double>
    // yaml["camera_matrix"] 按键名访问 yaml 节点，取出的值是一个列表，as<std::vector<double>>() 把yaml里面的数组转成 vector<double>
    // 用 cv::Mat(3, 3, CV_64F, vector.data()).clone() 转成 3x3 矩阵
    // 第一个参数 3 表示行数，第二个参数 3 表示列数，第三个参数 CV_64F 表示数据类型为 64bit double
    // 第四个参数是数据指针（vector.data()）返回vector底层数组的首地址，最后 clone() 是为了深拷贝数据，把数据拷贝到camera_matrix_自己的内存中，避免 vector 被销毁后 cv::Mat 数据丢失。
    auto camera_matrix = yaml["camera_matrix"].as<std::vector<double>>();
    camera_matrix_ = cv::Mat(3,3,CV_64F,camera_matrix.data()).clone();// 使用CV_64F是因为相机内参通常是浮点数，使用double双精度可以提高计算精度，避免数值误差对PnP解算的影响。
    // 1.3 读取 distort_coeffs 畸变系数（yaml 里是 5 个数的列表），转成 1x5 的 cv::Mat
    auto distort_coeffs = yaml["distort_coeffs"].as<std::vector<double>>();
    distort_coeffs_ = cv::Mat(1,5,CV_64F,distort_coeffs.data()).clone();
    std::vector<double> R_vec = yaml["R_camera2gimbal"].as<std::vector<double>>();
    R_camera2gimbal_ = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(R_vec.data());
    std::vector<double> t_vec = yaml["t_camera2gimbal"].as<std::vector<double>>();
    t_camera2gimbal_ = Eigen::Map<Eigen::Vector3d>(t_vec.data());
}

// solve 函数，对传入的 armor 做 PnP 解算，把 3D 坐标写回 armor.xyz_in_gimbal
void Solver::solve(Armor & armor) const// const 修饰成员函数：承诺不修改 Solver 自己的状态（camera_matrix_ 等）, Armor & armor 是非 const 引用：允许修改传入的 armor 对象
{
    // 2.1 根据 armor.type 判断大小，设置 half_w 和 half_h（单位：米）
    // small: 宽 0.135m，高 0.055m
    // big:   宽 0.225m，高 0.055m
    double half_w = 0.0;
    double half_h = 0.0;
    if(armor.type == ArmorType::small){
        half_w = 0.135 / 2.0;
        half_h = 0.055 / 2.0;
    }
    else{
        half_w = 0.225 / 2.0;
        half_h = 0.055 / 2.0;
    }
    // 2.2 构建 object_points（物体坐标系下的 4 个 3D 角点）
    // 原点：装甲板中心
    // 顺序：左上、右上、右下、左下（z 都等于 0，因为点在板面上）
    // Point3f是OpenCV里的三维浮点坐标点，f表示float，3表示三维
    std::vector<cv::Point3f> object_points;
    object_points.push_back(cv::Point3f(-half_w,half_h,0.0));//左上
    object_points.push_back(cv::Point3f(half_w,half_h,0.0));//右上
    object_points.push_back(cv::Point3f(half_w,-half_h,0.0));//右下
    object_points.push_back(cv::Point3f(-half_w,-half_h,0.0));//左下

    // 2.3 图像上的 2D 角点（YOLO 已经检测好了）
    std::vector<cv::Point2f> image_points = armor.points;
    if(image_points.size() != 4){
        return;
    }
    
    // 2.4 调用 cv::solvePnP
    cv::Mat rvec, tvec; // rvec 是旋转向量，tvec 是平移向量
    // false 表示不使用初始值，cv::SOLVEPNP_ITERATIVE 是迭代法
    bool pnpSuccess = cv::solvePnP(object_points,image_points,camera_matrix_,distort_coeffs_,rvec,tvec,false,cv::SOLVEPNP_ITERATIVE);
    if(!pnpSuccess){
        return;
    }
    // 2.5 把 tvec 的结果写入 armor.xyz_in_gimbal
    // tvec 是 cv::Mat(3,1,CV_64F)，用 tvec.at<double>(0/1/2) 读取
    // armor.xyz_in_gimbal 是 Eigen::Vector3d(x, y, z)
    Eigen::Vector3d xyz_camera (tvec.at<double>(0),tvec.at<double>(1),tvec.at<double>(2));
    Eigen::Vector3d xyz_gimbal = R_camera2gimbal_ * xyz_camera + t_camera2gimbal_;
    double distance = xyz_gimbal.norm();
    if(distance < 0.1 || distance > 20.0 || std::isnan(distance)){
        return;
    }
    armor.xyz_in_gimbal = xyz_gimbal;

    // 2.6 计算装甲板在云台坐标系下的姿态与法向量
    // rvec 是旋转向量，用 Rodrigues 公式转成旋转矩阵 R_camera
    cv::Mat R_camera_cv;
    cv::Rodrigues(rvec, R_camera_cv);

    // 将 OpenCV 的 3x3 旋转矩阵按元素复制到 Eigen（按行优先读）
    Eigen::Matrix3d R_camera;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        R_camera(i, j) = R_camera_cv.at<double>(i, j);
      }
    }

    // R_gimbal = R_camera2gimbal * R_camera：把物体坐标系转到云台坐标系
    Eigen::Matrix3d R_gimbal = R_camera2gimbal_ * R_camera;

    // 提取 ZYX 欧拉角 (yaw-pitch-roll)，单位 rad
    double yaw = std::atan2(R_gimbal(1, 0), R_gimbal(0, 0));
    double pitch = std::asin(-R_gimbal(2, 0));
    double roll = std::atan2(R_gimbal(2, 1), R_gimbal(2, 2));
    armor.ypr_in_gimbal = Eigen::Vector3d(yaw, pitch, roll);

    // 物体坐标系 z 轴在云台坐标系下的方向就是装甲板法向量（指向相机/车外）
    armor.normal_in_gimbal = R_gimbal.col(2).normalized();
}

const cv::Mat & Solver::cameraMatrix() const { return camera_matrix_; }

const cv::Mat & Solver::distortCoeffs() const { return distort_coeffs_; }

Eigen::Matrix3d Solver::R_gimbal2camera() const
{
  return R_camera2gimbal_.transpose();
}

Eigen::Vector3d Solver::t_gimbal2camera() const
{
  return -R_camera2gimbal_.transpose() * t_camera2gimbal_;
}

} // namespace auto_aim
