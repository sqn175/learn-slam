/*
 * @Author: Shi Qin 
 * @Date: 2017-09-20 10:24:27 
 * @Last Modified by: Shi Qin
 * @Last Modified time: 2017-09-20 16:15:00
 */

#ifndef COMMON_THREADSAFE_QUEUE_H_
#define COMMON_THREADSAFE_QUEUE_H_

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

namespace lslam {

// Implementation of the multiple producer, multiple consumer 
template <typename QueueType>
class ThreadSafeQueue {
public:

// Constructor
ThreadSafeQueue() {
  shut_down_ = false;
}

// Destructor
~ThreadSafeQueue() {
  shut_down_ = true;
  cond_not_empty_.notify_all();
  cond_not_full_.notify_all();
}

size_t Size() const {
  std::unique_lock<std::mutex> lock(mutex_);
  size_t size = queue_.size();
  lock.unlock();
  return size;
}

bool Empty() const {
  std::unique_lock<std::mutex> lock(mutex_);
  bool empty = queue_.empty();
  lock.unlock();
  return empty;
}

// Push the value to queue if the queue actual size is less than max_queue_size(not full), else block 
bool PushBlockingIfFull(const QueueType& value, size_t max_queue_size) {
  while (!shut_down_) {
    std::unique_lock<std::mutex> lock(mutex_);
    cond_not_full_.wait(lock, [&]{return queue_.size() < max_queue_size; });
    // Push to queue
    queue_.push(value);
    // Signal other thread that queue is available
    cond_not_empty_.notify_one();
    return true;
  }
  return false;
}

bool PopBlocking(QueueType* value) {
  CHECK_NOTNULL(value);
  while (!shut_down_) {
    std::unique_lock<std::mutex> lock(mutex_);
    cond_not_empty_.wait(lock, [&]{return !queue_.empty(); });
    QueueType _value = queue_.front();
    queue_.pop();
    cond_not_full_.notify_one();
    *value = _value;
    return true;
  }
  return false;
}

private:
  std::queue<QueueType> queue_; // Actual queue
  std::mutex mutex_; // The queue mutex
  std::condition_variable cond_not_empty_; 
  std::condition_variable cond_not_full_;
  std::atomic_bool shut_down_; // Flag if shutdown is requested
};

} // namespace lslam
#endif  //COMMON_THREADSAFE_QUEUE_H_