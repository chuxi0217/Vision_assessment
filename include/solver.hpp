/**
 * @file solver.hpp
 * @brief PnP 解算器接口定义
 *
 * 【小白注释】.hpp 头文件里一般只写声明，不写具体实现。
 * 这里声明了一个 Solver 类，告诉编译器：这个类叫什么、有哪些构造函数、
 * 有哪些成员函数、有哪些私有变量。真正的函数实现在 solver.cpp 里完成。
 *
 * 【小白注释】Doxygen 文档注释常用标记：
 * - @file：描述这个文件的作用
 * - @class：描述一个类
 * - @brief：简短描述
 * - @param：描述函数参数
 * - @return：描述函数返回值
 * - @section：文档分节，用于讲解知识点
 */

#ifndef AUTO_AIM__SOLVER_HPP  // 【小白注释】包含守卫：防止这个头文件被重复包含
#define AUTO_AIM__SOLVER_HPP  // 如果 AUTO_AIM__SOLVER_HPP 还没定义，就定义它

// 【小白注释】包含必要的头文件
#include <Eigen/Dense>            // Eigen 矩阵/向量库
#include <Eigen/Geometry>         // Eigen 四元数、旋转矩阵等几何工具
#include <opencv2/core/eigen.hpp> // OpenCV 与 Eigen 互转支持
#include <opencv2/opencv.hpp>     // OpenCV 图像处理与 PnP 算法
#include <string>                 // std::string 字符串类型

#include "armor.hpp"              // 装甲板数据结构，因为 Solver 要用到 Armor 类型

namespace auto_aim  // 命名空间：把相关类型封装在一起，避免与其他库冲突
{

/**
 * @class Solver
 * @brief 装甲板三维位姿解算器
 *
 * 【小白注释】class（类）是 C++ 面向对象编程的核心概念。
 * 类把“数据”和“操作数据的方法”封装在一起。
 * 比如 Solver 类保存了相机的内参和畸变系数（数据），
 * 并且提供了一个 solve() 方法（函数）来计算装甲板的三维位置。
 *
 * 通过相机模型和装甲板物理尺寸（如大小装甲板的长宽），
 * 利用 OpenCV solvePnP 系列算法将二维角点反投影到三维空间，
 * 得到装甲板在云台或世界坐标系下的位置与姿态。
 */
class Solver
{
public:  // 【小白注释】public 表示“公共的”，这些成员可以被类外部访问

  /**
   * @brief 构造 PnP 解算器
   * @param config_path YAML 配置文件路径
   *
   * 【小白注释】explicit 是构造函数修饰符，表示这个构造函数只能用于直接构造对象，
   * 不能用于隐式类型转换。例如：
   *   Solver s("config.yaml");        // OK，显式构造
   *   Solver s = "config.yaml";       // 如果加了 explicit，这种写法会报错
   * 这是为了防止意外地把字符串自动转换成 Solver 对象，增强代码安全性。
   *
   * const std::string & config_path：
   * - std::string 是 C++ 标准字符串类型
   * - & 表示引用，不会复制字符串，节省内存和时间
   * - const 表示这个函数不会修改 config_path 的内容
   */
  explicit Solver(const std::string & config_path);

  /**
   * @brief 核心解算函数
   * @param armor 待解算的装甲板对象
   *
   * 【小白注释】void 表示这个函数不返回任何值。
   * Armor & armor 中的 & 表示引用传递：
   * - 函数内部对 armor 的修改会影响到外部传入的 armor 对象
   * - 而不是只修改一个复制品
   * 这里我们需要把计算结果写回 armor 里，所以必须用引用。
   *
   * 函数名后面的 const 表示：
   * - 这个函数不会修改 Solver 类的成员变量（camera_matrix_, distort_coeffs_）
   * - 但传入的 armor 参数可以被修改，因为 armor 不是 const 的
   *
   * 在本函数内部实现 PnP 算法，根据装甲板类型（大/小）选择对应物理尺寸，
   * 结合图像角点和相机参数，计算 armor 中的三维空间字段：
   * - xyz_in_gimbal / xyz_in_world
   * - ypr_in_gimbal / ypr_in_world
   * - ypd_in_world
   */
  void solve(Armor & armor) const;

  // 以下 getter 供可视化模块把估计出的车体/装甲板三维位置投影回图像
  const cv::Mat & cameraMatrix() const;
  const cv::Mat & distortCoeffs() const;
  Eigen::Matrix3d R_gimbal2camera() const;
  Eigen::Vector3d t_gimbal2camera() const;

private:  // 【小白注释】private 表示“私有的”，这些成员只能被类内部的函数访问

  cv::Mat camera_matrix_;    ///< 相机内参矩阵（3x3），包含 fx, fy, cx, cy
                             // cv::Mat 是 OpenCV 里的矩阵类型，可以存储任意维度的矩阵/图像
  cv::Mat distort_coeffs_;   ///< 相机畸变系数（k1, k2, p1, p2, k3 等）
  Eigen::Matrix3d R_camera2gimbal_; ///< 相机到云台的旋转矩阵（3x3）
  Eigen::Vector3d t_camera2gimbal_; ///< 相机到云台的平移向量（3x1）
  // TODO: 可以根据需要添加辅助函数，例如从 yaml 读取内参、选择 armor 物理尺寸、坐标系转换等。
  // 辅助函数通常也放在 private 里，因为外部不需要直接调用它们。
};

}  // namespace auto_aim

#endif  
