// 参照元: autoware_utils (commit 6b89a2e1)
//   autoware_utils_system/include/autoware_utils_system/stop_watch.hpp
// 動作もメソッド名も同じ。
#pragma once
#include <chrono>
#include <string>
#include <unordered_map>

namespace ndt_localization::utils_system
{
/**
 * @brief 名前ごとに開始時刻を覚えて経過時間を測るストップウォッチ。
 * @tparam OutputUnit toc() が返す値の単位（既定は秒）。
 * @tparam InternalUnit 内部で数える単位。
 * @tparam Clock 使う時計（既定は steady_clock）。
 */
template<
  class OutputUnit = std::chrono::seconds, class InternalUnit = std::chrono::microseconds,
  class Clock = std::chrono::steady_clock>
class StopWatch
{
public:
  StopWatch() {tic(default_name);}

  /// 名前 name の計測を始める（同じ名前なら開始時刻を上書きする）。
  void tic(const std::string & name = default_name) {t_start_[name] = Clock::now();}

  void tic(const char * name) {tic(std::string(name));}

  /**
   * @brief 名前 name の経過時間を返す。
   * @param[in] name tic() で始めた名前。始めていない名前なら std::out_of_range。
   * @param[in] reset true なら開始時刻を今に戻す。
   * @return 経過時間（OutputUnit 単位）。
   */
  double toc(const std::string & name, const bool reset = false)
  {
    const auto t_start = t_start_.at(name);
    const auto t_end = Clock::now();
    const auto duration = std::chrono::duration_cast<InternalUnit>(t_end - t_start).count();

    if (reset) {
      t_start_[name] = Clock::now();
    }

    const auto one_sec = std::chrono::duration_cast<InternalUnit>(OutputUnit(1)).count();

    return static_cast<double>(duration) / one_sec;
  }

  double toc(const char * name, const bool reset = false) {return toc(std::string(name), reset);}

  double toc(const bool reset = false) {return toc(default_name, reset);}

private:
  using Time = std::chrono::time_point<Clock>;
  static constexpr const char * default_name{"__auto__"};

  std::unordered_map<std::string, Time> t_start_;
};
}  // namespace ndt_localization::utils_system
