/**
 * @file img_tools.hpp
 * @brief 图像可视化辅助函数集合
 *
 * 本文件提供一组常用的 OpenCV 绘图封装函数，用于在调试图像上
 * 绘制点、点集、文字等标注信息，简化主流程中的可视化代码。
 */

#ifndef TOOLS__IMG_TOOLS_HPP
#define TOOLS__IMG_TOOLS_HPP

#include <opencv2/opencv.hpp>  // OpenCV 图像处理库
#include <string>              // 字符串类型
#include <vector>              // 动态数组容器

namespace tools
{

/**
 * @brief 在图像上绘制单个圆点
 * @param img 输入/输出图像
 * @param point 待绘制的点坐标
 * @param color 绘制颜色，默认为红色 (BGR: 0, 0, 255)
 * @param radius 圆点半径，默认为 3 像素
 */
void draw_point(
  cv::Mat & img, const cv::Point & point, const cv::Scalar & color = {0, 0, 255}, int radius = 3);

/**
 * @brief 在图像上绘制 cv::Point 类型的点集（连线）
 * @param img 输入/输出图像
 * @param points 待绘制的点集
 * @param color 绘制颜色，默认为红色 (BGR: 0, 0, 255)
 * @param thickness 线条粗细，默认为 2 像素
 *
 * 通常将点按顺序连接成闭合多边形，用于显示装甲板轮廓或灯条。
 */
void draw_points(
  cv::Mat & img, const std::vector<cv::Point> & points, const cv::Scalar & color = {0, 0, 255},
  int thickness = 2);

/**
 * @brief 在图像上绘制 cv::Point2f 类型的点集（连线）
 * @param img 输入/输出图像
 * @param points 待绘制的浮点型点集
 * @param color 绘制颜色，默认为红色 (BGR: 0, 0, 255)
 * @param thickness 线条粗细，默认为 2 像素
 */
void draw_points(
  cv::Mat & img, const std::vector<cv::Point2f> & points, const cv::Scalar & color = {0, 0, 255},
  int thickness = 2);

/**
 * @brief 在图像上绘制文字
 * @param img 输入/输出图像
 * @param text 待显示文字
 * @param point 文字左下角位置
 * @param color 文字颜色，默认为黄色 (BGR: 0, 255, 255)
 * @param font_scale 字体缩放比例，默认为 1.0
 * @param thickness 文字线条粗细，默认为 2 像素
 */
void draw_text(
  cv::Mat & img, const std::string & text, const cv::Point & point,
  const cv::Scalar & color = {0, 255, 255}, double font_scale = 1.0, int thickness = 2);

}  // namespace tools

#endif  // TOOLS__IMG_TOOLS_HPP
