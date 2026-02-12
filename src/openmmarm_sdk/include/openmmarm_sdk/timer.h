#pragma once

/**
 * @file timer.h
 * @brief 定时器和循环线程工具类
 *
 * 提供 Timer（单次/周期计时）和 Loop（周期循环线程）两种工具，
 * 用于确保控制循环以固定频率执行。
 */

#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace OPENMMARM_SDK {

/**
 * @brief 周期定时器
 *
 * 用于控制循环的节拍，确保每次循环恰好消耗一个周期。
 *
 * 用法:
 * @code
 *   Timer timer(0.004); // 4ms = 250Hz
 *   while (running) {
 *     doWork();
 *     timer.sleep(); // 自动补齐剩余时间
 *   }
 * @endcode
 */
class Timer {
public:
  /**
   * @brief 构造定时器
   * @param period_sec 周期，单位：秒
   */
  explicit Timer(double period_sec) : period_(period_sec) { start(); }

  /// 获取周期（秒）
  double period() const { return period_; }

  /// 重置起始时间点
  void start() { start_time_ = std::chrono::steady_clock::now(); }

  /// 计算自上次 start() 以来经过的时间（秒）
  double elapsedTime() const {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = now - start_time_;
    return std::chrono::duration<double>(elapsed).count();
  }

  /// 计算距离一个周期结束还剩多少时间（秒，负值表示超时）
  double waitTime() const { return period_ - elapsedTime(); }

  /// 休眠至一个周期结束，然后自动重置起始时间
  void sleep() {
    double wt = waitTime();
    if (wt > 0) {
      std::this_thread::sleep_for(
          std::chrono::microseconds(static_cast<size_t>(wt * 1e6)));
    }
    start();
  }

private:
  double period_;
  std::chrono::steady_clock::time_point start_time_;
};
using TimerPtr = std::shared_ptr<Timer>;

/**
 * @brief 固定频率循环线程
 *
 * 在独立线程中以固定频率执行回调函数。
 *
 * 用法:
 * @code
 *   Loop loop("control", 0.004, []() { doControl(); });
 *   loop.start();
 *   // ... 做其他事 ...
 *   loop.shutdown();
 * @endcode
 */
class Loop {
public:
  /**
   * @param name     循环名称（用于日志）
   * @param period   周期，单位：秒
   * @param callback 每个周期执行的回调
   */
  Loop(std::string name, double period, std::function<void()> callback)
      : name_(std::move(name)), cb_(std::move(callback)) {
    timer_ = std::make_shared<Timer>(period);
  }

  ~Loop() { shutdown(); }

  /// 启动循环线程
  void start() {
    if (running_) {
      std::cout << "[WARNING] Loop " << name_ << " 已在运行\n";
      return;
    }
    running_ = true;
    thread_ = std::thread(&Loop::runImpl, this);
  }

  /// 停止循环线程并等待退出
  void shutdown() {
    if (!running_)
      return;
    running_ = false;
    if (thread_.joinable())
      thread_.join();
    if (show_info_) {
      double rate = run_times_ > 0 ? 100.0 * timeout_times_ / run_times_ : 0.0;
      std::cout << "[REPORT] Loop " << name_ << " 超时率: " << rate << "%\n";
    }
  }

  /// 执行一次循环迭代（含计时和休眠）
  void spinOnce() {
    timer_->start();
    ++run_times_;
    cb_();
    if (timer_->waitTime() > 0) {
      timer_->sleep();
    } else {
      ++timeout_times_;
      if (show_info_) {
        std::cout << "[WARNING] Loop " << name_ << " 超时，耗时 "
                  << timer_->elapsedTime() << "s\n";
      }
    }
  }

  /// 控制是否打印日志
  void showInfo(bool state) { show_info_ = state; }

private:
  void runImpl() {
    while (running_) {
      spinOnce();
    }
  }

  std::string name_;
  std::atomic<bool> running_{false};
  TimerPtr timer_;
  std::function<void()> cb_;
  std::thread thread_;
  size_t run_times_ = 0;
  size_t timeout_times_ = 0;
  bool show_info_ = false;
};
using LoopPtr = std::shared_ptr<Loop>;

} // namespace OPENMMARM_SDK
