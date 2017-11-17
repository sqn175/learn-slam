#ifndef COMMON_TIMER_H_
#define COMMON_TIMER_H_

#include <chrono>
#include <string>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "helper.h"

namespace lslam {

typedef std::chrono::milliseconds TimeT;
typedef std::chrono::steady_clock ClockT;

class Timer {
public:
  Timer(size_t handle, bool construct_stopped = false);
  Timer(const std::string& tag, bool construct_stopped = false);
  // Forbid nonsense timer 
  Timer(const Timer&) = delete;
  Timer& operator=(const Timer&) = delete;
  ~Timer();

  void Start();
  void Stop();
  bool is_timing() const;
  void DiscardTiming();
private:
  std::chrono::time_point<ClockT> time_;
  bool is_timing_;
  size_t handle_;
};

class Timing{
public:
  friend class Timer;

  // Static functions to query the timers:
  static size_t GetHandle(const std::string& tag);
  static std::string GetTag(size_t handle);
  static double GetTotalSeconds(size_t handle);
  static double GetTotalSeconds(const std::string& tag);
  static double GetMeanSeconds(size_t handle);
  static double GetMeanSeconds(const std::string& tag);
  static double GetNumSamples(size_t handle);
  static double GetNumSamples(const std::string& tag);
  static double GetVarianceSeconds(size_t handle);
  static double GetVarianceSeconds(const std::string& tag);
  static double GetMinSeconds(size_t handle);
  static double GetMinSeconds(const std::string& tag);
  static double GetMaxSeconds(size_t handle);
  static double GetMaxSeconds(const std::string& tag);
  static void Print(std::ostream& out);
  static std::string Print();
  static std::string SecondsToTimeString(double seconds);
private:
  void AddTime(size_t handle, double seconds);

  static Timing& instance();

  // Singleton design pattern
  Timing();
  ~Timing();

  std::mutex add_new_handle_mutex_;

  // Static members
  std::unordered_map<std::string,size_t> tag_map_;
  std::vector<Accumulator<double>> timers_;

  size_t max_tag_length_;
};

} // namespace LSLAM
#endif  //COMMON_TIMER_H_
