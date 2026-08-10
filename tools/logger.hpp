/**
 * @file logger.hpp
 * @brief 日志工具封装
 *
 * 本文件封装了 spdlog 日志库的单例/工厂函数，
 * 为项目各模块提供统一的日志记录器（logger）。
 *
 * 使用方式示例：
 * @code
 *   tools::logger()->info("Start processing frame {}", frame_count);
 * @endcode
 */

#ifndef TOOLS__LOGGER_HPP
#define TOOLS__LOGGER_HPP

#include <spdlog/spdlog.h>  // spdlog 日志库

namespace tools
{

/**
 * @brief 获取全局共享日志记录器
 * @return std::shared_ptr<spdlog::logger> 指向 spdlog logger 的共享指针
 *
 * 该 logger 通常在程序初始化时创建并注册为默认 logger，
 * 各模块通过此函数统一访问，避免重复创建日志对象。
 */
std::shared_ptr<spdlog::logger> logger();

}  // namespace tools

#endif  // TOOLS__LOGGER_HPP
