/**
 * @file armor.hpp
 * @brief 自瞄系统装甲板数据结构定义
 *
 * 【小白注释】什么是 .hpp 文件？
 * .hpp（header plus plus）是 C++ 的头文件，里面主要放“声明”：
 * 告诉编译器有哪些变量、结构体、函数、类存在，它们的名称和类型是什么。
 * 真正的代码逻辑（比如函数体、结构体方法的实现）通常放在同名的 .cpp 文件里。
 * 这样做的好处是：多个 .cpp 文件都能 include 这个头文件，共享这些定义。
 *
 * 【小白注释】这些以 @ 开头的词是什么？
 * 这是 Doxygen 风格的文档注释标记。Doxygen 是一个自动根据注释生成文档的工具。
 * 常用标记：
 * - @file：说明这个文件的名字和总体作用
 * - @brief：简短描述
 * - @struct / @class / @enum / @var：说明后面的结构体/类/枚举/变量
 * - @param：说明函数的参数
 * - @return：说明函数的返回值
 * - ///<：写在变量右边，表示给紧挨着的这个变量加注释
 *
 * 这些注释不会影响程序运行，只是方便阅读和维护。
 */

#ifndef AUTO_AIM__ARMOR_HPP  // 【小白注释】#ifndef 是 "if not defined" 的缩写，表示如果 AUTO_AIM__ARMOR_HPP 还没被定义过
#define AUTO_AIM__ARMOR_HPP  // 【小白注释】就定义它。配合最后的 #endif，这叫做“头文件保护/包含守卫”，防止同一个文件被重复包含导致编译错误

// 【小白注释】#include 是“包含”的意思，把其他头文件的内容复制到当前位置
// 这样我们就能使用别人已经写好的类和函数了
#include <Eigen/Dense>     // Eigen：一个用于矩阵和向量运算的数学库
#include <opencv2/opencv.hpp>  // OpenCV：计算机视觉库，提供图像、点、矩阵等类型
#include <string>        // std::string：字符串类型
#include <vector>        // std::vector：动态数组，可以自动扩容

namespace auto_aim  // 【小白注释】namespace 叫“命名空间”，相当于给这些类型加了一个前缀，避免和其他库里的同名东西冲突
{

/**
 * @enum Color
 * @brief 装甲板颜色（或状态）枚举
 *
 * 【小白注释】enum 是“枚举”类型，作用是把一组有限的状态给它们起名字。
 * 比如不写 enum，我们就得用数字 0 代表红、1 代表蓝，很容易记混。
 * 用 enum 后，代码里直接写 red、blue，编译器会自动转换成数字。
 */
enum Color
{
  red,          ///< 红色装甲板
  blue,         ///< 蓝色装甲板
  extinguish,   ///< 熄灭（无有效颜色/已下电）
  purple        ///< 紫色（特殊状态/基地等）
};

/**
 * @var COLORS
 * @brief 颜色枚举对应的字符串名称，用于日志输出和可视化标注
 *
 * 【小白注释】std::vector<std::string> 是一个“字符串数组”。
 * 这里用 COLORS[red] 就能得到 "red" 这个字符串，方便打印和调试。
 * const 表示这个变量不能修改；std::vector 是动态数组，会自动管理内存。
 */
const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};

/**
 * @enum ArmorType
 * @brief 装甲板尺寸类型
 *
 * 大装甲板（big）通常用于英雄机器人、基地等；
 * 小装甲板（small）通常用于步兵、哨兵、前哨站等。
 */
enum ArmorType
{
  big,    ///< 大装甲板
  small   ///< 小装甲板
};

/**
 * @var ARMOR_TYPES
 * @brief 装甲板尺寸类型对应的字符串名称
 */
const std::vector<std::string> ARMOR_TYPES = {"big", "small"};

/**
 * @enum ArmorName
 * @brief 装甲板编号/名称枚举
 *
 * 对应 RoboMaster 规则中不同机器人的装甲板编号，以及哨兵、前哨站、基地等特殊目标。
 */
enum ArmorName
{
  one,       ///< 1 号装甲板
  two,       ///< 2 号装甲板
  three,     ///< 3 号装甲板
  four,      ///< 4 号装甲板
  five,      ///< 5 号装甲板
  sentry,    ///< 哨兵机器人
  outpost,   ///< 前哨站
  base,      ///< 基地
  not_armor  ///< 非装甲板（无效/背景类）
};

/**
 * @var ARMOR_NAMES
 * @brief 装甲板名称枚举对应的字符串名称
 */
const std::vector<std::string> ARMOR_NAMES = {"one",    "two",     "three", "four",     "five",
                                              "sentry", "outpost", "base",  "not_armor"};

/**
 * @enum ArmorPriority
 * @brief 装甲板打击优先级
 *
 * 数值越小表示优先级越高，用于多目标存在时选择优先打击目标。
 */
enum ArmorPriority
{
  first = 1,   ///< 第一优先级
  second,      ///< 第二优先级
  third,       ///< 第三优先级
  forth,       ///< 第四优先级
  fifth        ///< 第五优先级
};

// clang-format off
/**
 * @var armor_properties
 * @brief 装甲板属性组合表
 *
 * 每个三元组表示一种合法装甲板：颜色、名称、类型。
 * 该表用于传统视觉检测或深度学习后处理中筛选合法目标组合。
 *
 * 【小白注释】std::tuple<Color, ArmorName, ArmorType> 是一个“三元组”，
 * 可以把三个不同类型的数据打包在一起。
 * std::vector<...> 就是很多个这样的三元组组成的数组。
 * 这里的 armor_properties 是一个常量表，记录哪些（颜色、编号、大小）组合是真实存在的。
 *
 * 注意：此表由 clang-format 关闭自动格式化，以保持三元组的可读性排列。
 */
const std::vector<std::tuple<Color, ArmorName, ArmorType>> armor_properties = {
  {blue, sentry, small},     {red, sentry, small},     {extinguish, sentry, small},
  {blue, one, small},        {red, one, small},        {extinguish, one, small},
  {blue, two, small},        {red, two, small},        {extinguish, two, small},
  {blue, three, small},      {red, three, small},      {extinguish, three, small},
  {blue, four, small},       {red, four, small},       {extinguish, four, small},
  {blue, five, small},       {red, five, small},       {extinguish, five, small},
  {blue, outpost, small},    {red, outpost, small},    {extinguish, outpost, small},
  {blue, base, big},         {red, base, big},         {extinguish, base, big},      {purple, base, big},       
  {blue, base, small},       {red, base, small},       {extinguish, base, small},    {purple, base, small},    
  {blue, three, big},        {red, three, big},        {extinguish, three, big}, 
  {blue, four, big},         {red, four, big},         {extinguish, four, big},  
  {blue, five, big},         {red, five, big},         {extinguish, five, big}};
// clang-format on

/**
 * @struct Lightbar
 * @brief 装甲板灯条信息
 *
 * 【小白注释】struct（结构体）是一种把多个相关变量打包在一起的数据类型。
 * 比如一个灯条有中心、端点、颜色、角度等属性，我们就可以把它们都放在一个结构体里，
 * 这样代码更好理解，也更容易传递。
 *
 * 灯条是传统视觉检测的基本单元，一个装甲板通常由左右两个灯条组成。
 * 该结构体保存了灯条的几何中心、端点、角度、长宽比、旋转矩形等信息。
 */
struct Lightbar
{
  std::size_t id;          ///< 灯条在某一帧中的编号，用于跟踪或调试
  Color color;             ///< 灯条颜色

  cv::Point2f center;      ///< 灯条中心点（图像坐标）
                           // cv::Point2f 是 OpenCV 里的二维浮点坐标点，f 表示 float
  cv::Point2f top;         ///< 灯条上端点（靠近装甲板中心一侧）
  cv::Point2f bottom;      ///< 灯条下端点（远离装甲板中心一侧）
  cv::Point2f top2bottom;  ///< 从 top 指向 bottom 的向量，用于表示方向与长度

  std::vector<cv::Point2f> points;  ///< 灯条轮廓点集（或拟合用点）

  double angle;            ///< 灯条主轴与图像水平轴的夹角，单位：度
  double angle_error;      ///< 角度误差/可信度，用于筛选候选灯条
  double length;           ///< 灯条长度（像素）
  double width;            ///< 灯条宽度（像素）
  double ratio;            ///< 长宽比 length / width

  cv::RotatedRect rotated_rect;  ///< OpenCV 旋转矩形，包含位置、尺寸和角度

  /**
   * @brief 从旋转矩形构造灯条
   * @param rotated_rect 检测得到的旋转矩形
   * @param id 灯条编号
   *
   * 【小白注释】构造函数（Constructor）是在创建这个结构体/类的对象时自动调用的函数，
   * 用于初始化成员变量。这里的 Lightbar(...) 让我们可以用一个旋转矩形和一个编号来创建灯条。
   *
   * const cv::RotatedRect & rotated_rect 中的 & 表示“引用”，
   * 意思是传进来的 rotated_rect 不是复制品，而是原变量的别名，省内存、速度快；
   * const 表示函数内部不会修改它。
   */
  Lightbar(const cv::RotatedRect & rotated_rect, std::size_t id);

  /**
   * @brief 默认构造函数
   *
   * 【小白注释】默认构造函数就是没有任何参数的构造函数。
   * 比如定义 Lightbar lb; 时会调用这个函数，把成员变量初始化为默认值。
   * 如果结构体里有其他构造函数，编译器就不会再自动生成默认构造函数，所以这里我们手动写一个。
   */
  Lightbar() {};
};

/**
 * @struct Armor
 * @brief 装甲板目标信息
 *
 * 【小白注释】Armor 结构体比 Lightbar 更复杂，它代表一个完整的装甲板目标。
 * 一个装甲板可能由：
 * - 传统视觉检测的左右两个灯条组成；
 * - 或者深度学习网络输出的边界框和四个角点组成。
 *
 * 结构体保存了图像坐标、类别信息、三维空间位置（由 Solver 解算后填充）等。
 */
struct Armor
{
  Color color;             ///< 装甲板颜色

  Lightbar left, right;  ///< 装甲板左侧与右侧灯条（传统检测使用）

  cv::Point2f center;      ///< 图像坐标系下的装甲板中心（注：不是实际物理中心，仅用于图像层面）
  cv::Point2f center_norm; ///< 归一化坐标
  std::vector<cv::Point2f> points;  ///< 装甲板四个角点（或关键点集合），通常顺序为左上、右上、右下、左下

  // 几何特征（传统检测用）
  double ratio;              // 两灯条的中点连线与长灯条的长度之比
  double side_ratio;         // 长灯条与短灯条的长度之比
  double rectangular_error;  // 灯条和中点连线所成夹角与π/2的差值

  // 分类与识别信息
  ArmorType type;          ///< 装甲板尺寸类型（大/小）
  ArmorName name;          ///< 装甲板名称（如 one, sentry, base 等）
  ArmorPriority priority;  ///< 打击优先级
  int class_id;            ///< 神经网络输出的原始类别编号

  // 神经网络检测相关
  cv::Rect box;            ///< 检测框（左上角坐标 + 宽高）
  cv::Mat pattern;         ///< 用于分类/识别的装甲板图案 ROI（可选）
  double confidence;       ///< 检测置信度
  bool duplicated;         ///< 是否与其他装甲板重复/重叠，用于去重

  // 三维空间信息（由 Solver 解算后填充）
  // 【小白注释】Eigen::Vector3d 是 Eigen 库里的三维向量，d 表示 double 类型
  // 它可以表示空间中的一个点（x, y, z）或者一个姿态角（yaw, pitch, roll）
  Eigen::Vector3d xyz_in_gimbal;  ///< 在云台坐标系下的三维坐标，单位：m
  Eigen::Vector3d xyz_in_world;   ///< 在世界坐标系下的三维坐标，单位：m
  Eigen::Vector3d ypr_in_gimbal;  ///< 在云台坐标系下的姿态角（yaw, pitch, roll），单位：rad
  Eigen::Vector3d ypr_in_world;   ///< 在世界坐标系下的姿态角，单位：rad
  Eigen::Vector3d ypd_in_world;   ///< 在世界坐标系下的球坐标（yaw, pitch, distance），单位：弧度/米

  double yaw_raw;  // rad

  /**
   * @brief 由左右灯条构造装甲板（传统视觉检测）
   * @param left 左侧灯条
   * @param right 右侧灯条
   */
  Armor(const Lightbar & left, const Lightbar & right);

  /**
   * @brief 由神经网络检测结果构造装甲板
   * @param class_id 类别编号
   * @param confidence 置信度
   * @param box 边界框
   * @param armor_keypoints 装甲板关键点（如 4 个角点）
   */
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints);

  /**
   * @brief 由神经网络检测结果构造装甲板，支持 ROI 偏移修正
   * @param class_id 类别编号
   * @param confidence 置信度
   * @param box 边界框
   * @param armor_keypoints 装甲板关键点
   * @param offset ROI 在原图上的偏移量，用于将局部坐标转换到原图坐标
   */
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints,
    cv::Point2f offset);

  /**
   * @brief 由颜色和编号组合构造装甲板（适用于有颜色分类的检测模型）
   * @param color_id 颜色类别编号
   * @param num_id 数字/名称类别编号
   * @param confidence 置信度
   * @param box 边界框
   * @param armor_keypoints 装甲板关键点
   */
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints);

  /**
   * @brief 由颜色和编号组合构造装甲板，支持 ROI 偏移修正
   * @param color_id 颜色类别编号
   * @param num_id 数字/名称类别编号
   * @param confidence 置信度
   * @param box 边界框
   * @param armor_keypoints 装甲板关键点
   * @param offset ROI 在原图上的偏移量
   */
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__ARMOR_HPP
