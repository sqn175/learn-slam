#include "timer.h"

#include <stdio.h> // printf()
#include <math.h>  // fmod()

#include <glog/logging.h>

namespace lslam {

Timing& Timing::instance() {
  static Timing t;
  return t;
}

Timing::Timing() : max_tag_length_(0) {

}

Timing::~Timing() {

}

size_t Timing::GetHandle(const std::string& tag) {
  auto i = instance().tag_map_.find(tag);
  if (i == instance().tag_map_.end()) {
    std::lock_guard<std::mutex> lock(instance().add_new_handle_mutex_);
    // If it is not there, create a tag.
    size_t handle = instance().timers_.size();
    instance().tag_map_[tag] = handle;
    instance().timers_.push_back(Accumulator<double>());
    // Track the maximum tag length to help printing a table of timing values later.
    instance().max_tag_length_ = std::max(instance().max_tag_length_, tag.size());
    return handle;
  } else {
    return i->second;
  }
}

std::string Timing::GetTag(size_t handle) {
  std::string tag;
  bool found = false;

  // Perform a linear search for the tag
  auto i = instance().tag_map_.begin();
  for (; i != instance().tag_map_.end();++i) {
    if (i->second == handle) {
      found = true;
      tag = i->first;
      break;
    }
  }

  CHECK(found) << "Unable to find the tag associated with handle " << handle;
  return tag;
}

Timer::Timer(size_t handle, bool construct_stopped)
  : is_timing_(false)
  , handle_(handle) {
  CHECK(handle < Timing::instance().timers_.size()) << "The handle is invalid. Handle: " << 
          handle << ", number of timers: " << Timing::instance().timers_.size();
  if (!construct_stopped) 
    Start();
}

Timer::Timer(const std::string& tag, bool construct_stopped)
  : is_timing_(false)
  , handle_(Timing::GetHandle(tag)) {

  if (!construct_stopped) 
    Start();
}

Timer::~Timer() {
  if (is_timing())
    Stop();
}

void Timer::Start() {
  CHECK(!is_timing_) << "The timer " + Timing::GetTag(handle_) + " is already running";
  is_timing_ = true;
  time_ = ClockT::now();
}

void Timer::Stop() {
  CHECK(is_timing_) << "The timer " + Timing::GetTag(handle_) + " is not running";
  auto time_now = ClockT::now();
  auto duration = std::chrono::duration_cast<TimeT>(time_now - time_);
  double dt_seconds = (double)duration.count() * TimeT::period::num / TimeT::period::den;
  Timing::instance().AddTime(handle_, dt_seconds);
  is_timing_ = false;
}

bool Timer::is_timing() const{
  return is_timing_;
}

void Timer::DiscardTiming() {
  is_timing_ = false;
}

void Timing::AddTime(size_t handle, double seconds) {
  timers_[handle](seconds);
}

double Timing::GetTotalSeconds(size_t handle) {
  CHECK(handle < instance().timers_.size()) << "Handle is out of range: " 
      << handle << ", number of timers: " << instance().timers_.size();
  
  return instance().timers_[handle].sum;
}

double Timing::GetTotalSeconds(const std::string& tag) {
  return GetTotalSeconds(GetHandle(tag));
}

double Timing::GetMeanSeconds(size_t handle) {
  CHECK(handle < instance().timers_.size()) << "Handle is out of range: " 
    << handle << ", number of timers: " << instance().timers_.size();

  return instance().timers_[handle].mean();
}

double Timing::GetMeanSeconds(const std::string& tag) {
  return GetMeanSeconds(GetHandle(tag));
}

double Timing::GetNumSamples(size_t handle) {
  CHECK(handle < instance().timers_.size()) << "Handle is out of range: " 
    << handle << ", number of timers: " << instance().timers_.size();

  return instance().timers_[handle].N;
}

double Timing::GetNumSamples(const std::string& tag) {
  return GetNumSamples(GetHandle(tag));
}

double Timing::GetVarianceSeconds(size_t handle) {
  CHECK(handle < instance().timers_.size()) << "Handle is out of range: " 
    << handle << ", number of timers: " << instance().timers_.size();

  return instance().timers_[handle].variance();
}

double Timing::GetVarianceSeconds(const std::string& tag) {
  return GetVarianceSeconds(GetHandle(tag));
}

double Timing::GetMinSeconds(size_t handle) {
  CHECK(handle < instance().timers_.size()) << "Handle is out of range: " 
    << handle << ", number of timers: " << instance().timers_.size();

  return instance().timers_[handle].min;
}
double Timing::GetMinSeconds(const std::string& tag) {
  return GetMinSeconds(GetHandle(tag));
}
double Timing::GetMaxSeconds(size_t handle) {
  CHECK(handle < instance().timers_.size()) << "Handle is out of range: " 
    << handle << ", number of timers: " << instance().timers_.size();

  return instance().timers_[handle].max;
}
double Timing::GetMaxSeconds(const std::string& tag) {
  return GetMaxSeconds(GetHandle(tag));
}

void Timing::Print(std::ostream& out) {
  auto tag_map = instance().tag_map_;

  out << "Timing\n";
  out << "---------------\n";
  std::map<std::string, size_t> ordered_map(tag_map.begin(), tag_map.end());
  for (auto& item : ordered_map) {
    size_t i = item.second;
    out.width((std::streamsize)instance().max_tag_length_);
    out.setf(std::ios::left, std::ios::adjustfield);
    out << item.first << "\t";
    out.width(7);

    out.setf(std::ios::right, std::ios::adjustfield);
    out << GetNumSamples(i) << "\t";
    if (GetNumSamples(i) > 0) {
      out << SecondsToTimeString(GetTotalSeconds(i)) << "\t";
      double mean_sec = GetMeanSeconds(i);
      double stddev = sqrt(GetVarianceSeconds(i));
      out << "(" << SecondsToTimeString(mean_sec) << "+-";
      out << SecondsToTimeString(stddev) << ")\t";

      double min_sec = GetMinSeconds(i);
      double max_sec = GetMaxSeconds(i);
      out << "[" << SecondsToTimeString(min_sec) << ", " << SecondsToTimeString(max_sec) << "]";
    }
    out << std::endl;
  }
}

std::string Timing::Print() {
  std::stringstream ss;
  Print(ss);
  return ss.str();
}

std::string Timing::SecondsToTimeString(double seconds) {
  double secs = fmod(seconds, 60);
  int minutes = (long)(seconds / 60);
  int hours = (long)(seconds/ 3600);
  minutes = minutes - (hours*60);

  char buffer[256];
  sprintf(buffer, "%02d:%02d:%09.6f",hours,minutes,secs);
  return buffer;
}
} // namespace LSLAM