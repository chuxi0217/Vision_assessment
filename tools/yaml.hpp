/**
 * @file yaml.hpp
 * @brief YAML 配置文件读取工具封装
 *
 * 本文件封装了 yaml-cpp 库的常用读取操作，提供统一的错误处理和日志输出。
 * 主要功能包括：
 * - 加载 YAML 文件并捕获异常
 * - 按 key 读取指定类型值，缺失时终止程序
 * - 按 key 读取值并提供默认值，缺失时返回默认值
 *
 * 使用方式示例：
 * @code
 *   YAML::Node config = tools::load("config.yaml");
 *   double threshold = tools::read<double>(config, "threshold");
 *   int max_iter = tools::read<int>(config, "max_iter", 100);
 * @endcode
 */

#ifndef TOOLS__YAML_HPP
#define TOOLS__YAML_HPP

#include <yaml-cpp/yaml.h>  // yaml-cpp 库

#include "tools/logger.hpp"  // 日志工具，用于输出加载/解析错误

namespace tools
{

/**
 * @brief 加载 YAML 文件
 * @param path YAML 文件路径
 * @return 解析后的 YAML 节点
 *
 * 如果文件不存在或解析失败，将记录错误日志并终止程序（exit(1)）。
 */
inline YAML::Node load(const std::string & path)
{
  try {
    return YAML::LoadFile(path);
  } catch (const YAML::BadFile & e) {
    logger()->error("[YAML] Failed to load file: {}", e.what());
    exit(1);
  } catch (const YAML::ParserException & e) {
    logger()->error("[YAML] Parser error: {}", e.what());
    exit(1);
  }
}

/**
 * @brief 从 YAML 节点读取指定类型的值
 * @tparam T 目标类型
 * @param yaml YAML 节点
 * @param key 键名
 * @return key 对应的值，转换为类型 T
 *
 * 如果 key 不存在，将记录错误日志并终止程序（exit(1)）。
 */
template <typename T>
inline T read(const YAML::Node & yaml, const std::string & key)
{
  if (yaml[key]) return yaml[key].as<T>();
  logger()->error("[YAML] {} not found!", key);
  exit(1);
}

/**
 * @brief 从 YAML 节点读取指定类型的值，若不存在则返回默认值
 * @tparam T 目标类型
 * @param yaml YAML 节点
 * @param key 键名
 * @param default_value 默认值
 * @return key 对应的值；若 key 不存在则返回 default_value
 *
 * 适用于有推荐默认值的配置项，缺失时不会报错或退出。
 */
template <typename T>
inline T read(const YAML::Node & yaml, const std::string & key, const T & default_value)
{
  if (yaml[key]) return yaml[key].as<T>();
  // logger()->warn("[YAML] {} not found! Using the default value: {}", key, default_value);
  return default_value;
}

}  // namespace tools

#endif  // TOOLS__YAML_HPP
