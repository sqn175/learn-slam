
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <iostream>
#include <chrono>
#include <string>     // std::string, std::to_string

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

bool PopBlocking(QueueType& value) {
  while (!shut_down_) {
    std::unique_lock<std::mutex> lock(mutex_);
    cond_not_empty_.wait(lock, [&]{return !queue_.empty(); });
    value = std::move(queue_.front());
    queue_.pop();
    cond_not_full_.notify_one();
    return true;
  }
  return false;
}

private:
  std::queue<QueueType> queue_; // Actual queue
  mutable std::mutex mutex_; // The queue mutex
  std::condition_variable cond_not_empty_; 
  std::condition_variable cond_not_full_;
  std::atomic_bool shut_down_; // Flag if shutdown is requested
};

ThreadSafeQueue<int> que;

void pro() {
  
  int size = que.Size();
  std::cout<<size<<std::endl;
    int i;
    que.PopBlocking(i);
    std::cout<<"poped " << i << std::endl;
}

void com() {
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  int size = que.Size();
  std::cout<<size<<std::endl;
    que.PushBlockingIfFull(9999,1);
    std::cout<<"pushed 9999\n";

      que.PushBlockingIfFull(9999,1);
      std::cout<<"pushed 9999\n";
      
        que.PushBlockingIfFull(9999,1);
        std::cout<<"pushed 9999\n";
}

int main() {
    std::cout<<"start.\n";
    std::thread t1(com), t2(pro);
    t1.join();
    
    //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    t2.join();
    return 0;
}
