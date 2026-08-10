/**
 * @file yolov5.hpp
 * @brief YOLOv5 装甲板检测器接口定义
 *
 * 本文件定义了基于 OpenVINO 推理引擎的 YOLOv5 装甲板检测器。
 * 该类支持：
 * - 通过 OpenVINO 加载并编译 YOLOv5 ONNX 模型
 * - 对输入图像进行前向推理，得到装甲板候选目标
 * - 后处理（NMS、类别筛选、ROI 裁剪、关键点解析）
 * - 调试输出（保存中间结果、绘制检测框等）
 *
 * 检测器同时支持 ROI 模式，可基于上一帧位置裁剪感兴趣区域以提升推理速度和稳定性。
 */

#ifndef AUTO_AIM__YOLOV5_HPP
#define AUTO_AIM__YOLOV5_HPP

#include <list>                   // 用于保存检测到的装甲板列表
#include <opencv2/opencv.hpp>     // OpenCV 图像处理
#include <openvino/openvino.hpp>  // OpenVINO 推理引擎
#include <string>                 // 字符串类型
#include <vector>                 // 动态数组容器

#include "armor.hpp"              // 装甲板数据结构

namespace auto_aim
{

/**
 * @class YOLOV5
 * @brief 基于 YOLOv5 + OpenVINO 的装甲板目标检测器
 *
 * 使用 OpenVINO 加载 YOLOv5 ONNX 模型，对每帧图像进行推理，
 * 输出一组 Armor 对象供后续 PnP 解算和跟踪模块使用。
 */
class YOLOV5
{
public:
  /**
   * @brief 构造检测器
   * @param config_path YAML 配置文件路径，包含模型路径、推理设备、阈值、ROI 参数等
   * @param debug 是否开启调试模式（保存图像、绘制检测结果等）
   */
  YOLOV5(const std::string & config_path, bool debug);

  /**
   * @brief 检测图像中的装甲板
   * @param bgr_img 输入 BGR 图像
   * @param frame_count 当前帧序号，用于调试输出命名
   * @return 检测到的装甲板列表（按置信度或后处理规则排序）
   */
  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count);

  /**
   * @brief 检测图像中的装甲板，并输出调试图像
   * @param bgr_img 输入 BGR 图像
   * @param frame_count 当前帧序号
   * @param out_debug_img 输出调试图像引用，将在此图像上绘制检测结果
   * @return 检测到的装甲板列表
   */
  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count, cv::Mat & out_debug_img);

  /**
   * @brief 对 OpenVINO 推理输出进行后处理
   * @param scale 图像预处理缩放比例（letterbox 缩放比例）
   * @param output 模型输出矩阵（通常为 [候选框数, 特征维度]）
   * @param bgr_img 原始输入图像，用于坐标映射
   * @param frame_count 当前帧序号
   * @return 解析后的装甲板目标列表
   *
   * 后处理流程通常包括：
   * - 置信度阈值过滤
   * - Sigmoid 激活
   * - 非极大值抑制（NMS）
   * - 关键点解析并构造 Armor 对象
   */
  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);

private:
  // 模型与推理参数
  std::string device_;       ///< 推理设备，如 "CPU" / "GPU" / "MYRIAD" 等
  std::string model_path_;   ///< ONNX 模型文件路径

  // 调试与保存路径
  std::string save_path_;    ///< 检测结果保存路径
  std::string debug_path_;   ///< 调试输出路径
  bool debug_;               ///< 是否启用调试模式

  // 功能开关
  bool use_roi_;             ///< 是否使用 ROI 区域裁剪以加速推理
  bool use_traditional_;     ///< 是否启用传统视觉辅助检测（预留）

  // 模型超参数
  const int class_num_ = 13;          ///< 模型类别总数（颜色 + 数字/名称的组合类别）
  const float nms_threshold_ = 0.3;   ///< NMS IoU 阈值
  const float score_threshold_ = 0.7; ///< 置信度过滤阈值
  double min_confidence_;             ///< 最终保留装甲板的最小置信度，从配置文件读取
  double binary_threshold_;           ///< 二值化/预处理阈值（传统辅助或可视化用）

  // OpenVINO 推理对象
  ov::Core core_;               ///< OpenVINO 核心对象，负责设备管理
  ov::CompiledModel compiled_model_; ///< 编译后的模型，用于创建推理请求

  // ROI 相关
  cv::Rect roi_;               ///< 当前帧 ROI 区域（在原始图像中的坐标）
  cv::Point2f offset_;       ///< ROI 左上角相对于原图的偏移，用于坐标还原
  cv::Mat tmp_img_;          ///< 临时图像缓存，用于 ROI 裁剪或预处理

  /**
   * @brief 检查装甲板名称是否在合法属性表中
   * @param armor 待检查的装甲板
   * @return 合法返回 true，否则返回 false
   */
  bool check_name(const Armor & armor) const;

  /**
   * @brief 检查装甲板尺寸类型是否合理
   * @param armor 待检查的装甲板
   * @return 合理返回 true，否则返回 false
   */
  bool check_type(const Armor & armor) const;

  /**
   * @brief 计算归一化中心坐标
   * @param bgr_img 原始图像
   * @param center 图像坐标系下的中心点
   * @return 归一化后的中心点坐标
   */
  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  /**
   * @brief 解析模型输出并构造 Armor 列表
   * @param scale letterbox 缩放比例
   * @param output 模型输出矩阵
   * @param bgr_img 原始图像
   * @param frame_count 当前帧序号
   * @param out_debug_img 调试输出图像指针，可选，为空则不绘制
   * @return 解析后的装甲板列表
   */
  std::list<Armor> parse(double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count, cv::Mat * out_debug_img = nullptr);

  /**
   * @brief 检测实现函数（核心推理 + 后处理）
   * @param raw_img 输入原始图像（可能已经是 ROI 裁剪后的图像）
   * @param frame_count 当前帧序号
   * @param out_debug_img 调试输出图像指针，可选
   * @return 检测到的装甲板列表
   */
  std::list<Armor> detect_impl(const cv::Mat & raw_img, int frame_count, cv::Mat * out_debug_img);

  /**
   * @brief 保存单个装甲板检测结果（如裁剪图案、调试图像）
   * @param armor 待保存的装甲板
   */
  void save(const Armor & armor) const;

  /**
   * @brief 在图像上绘制检测结果
   * @param img 待绘制图像
   * @param armors 检测到的装甲板列表
   * @param frame_count 当前帧序号
   * @param out_debug_img 调试输出图像指针，可选
   */
  void draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count, cv::Mat * out_debug_img = nullptr) const;

  /**
   * @brief Sigmoid 激活函数
   * @param x 输入值
   * @return sigmoid(x) = 1 / (1 + exp(-x))
   */
  double sigmoid(double x);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLOV5_HPP
