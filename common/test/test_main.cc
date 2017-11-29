#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <vector>
#include <iostream>
#include <chrono>
#include <string>
#include "helper.h"
#include "time_logger.h"
#include "threadsafe_queue.h"


TEST(BinarysearchTest, vector) {
	std::vector<int> v;
	for (size_t i = 0; i < 10; ++i) {
		v.push_back(i);
	}
	std::vector<int> v_found;
	for(int i = 0; i < 10; ++i) {
		auto idx = lslam::binary_search(v.begin(), v.end(), i);
		v_found.push_back(*idx);
	}
	ASSERT_EQ(v.size(), v_found.size()) << "Binary search result vector and origin vector are of unequal length";
	for (size_t i = 0; i < 10; ++i) {
		EXPECT_EQ(v[i], v_found[i]) << "Binary search result vector and origin vector differ at index" << i;
	}
}

int fib(int x) {
	if (x == 0)
		return 0;

	if (x == 1)
		return 1;

	return fib(x-1)+fib(x-2);
}

TEST(TimingTest, fib) {
	lslam::TimeLogger timer("fib");
	fib(10);
	timer.Stop();
	timer.Start();
	fib(15);
	timer.Stop();
	ASSERT_EQ(2, lslam::Timing::GetNumSamples(0));
}

lslam::ThreadSafeQueue<double> q1;
lslam::ThreadSafeQueue<double> q2;

void Consumer2() {
  // Pop
  double res;
  std::cout << "C2 Trying to pop... " << std::endl;
  q2.PopBlocking(res);
  std::cout << "Popped and processed." << res << std::endl;
}

void Consumer1AndProducer2() {
  // Pop and push
  double res;
  std::cout << "C1P2 Trying to pop... " << std::endl;
  q1.PopBlocking(res);
  std::cout << "Popped and processed." << res << std::endl;
  
  std::cout << "C1P2 Trying to push " << res << " to C2 " << std::endl;
  q2.PushBlockingIfFull(res, 1);
  std::cout << "C1P2 Pushed " << res << std::endl;
  
  // Pop and push
  std::cout << "Trying to pop... " << std::endl;
  q1.PopBlocking(res);
  std::cout << "Popped " << res << std::endl;
  std::cout << "C1P2 Trying to push " << res << " to C2 " << std::endl;
  q2.PushBlockingIfFull(res, 1);
  std::cout << "C1P2 Pushed " << res << std::endl;
  
}

void Producer1() {
  // Push
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  double res = 1;
  std::cout << "P1 Trying to push " << res << " to C1P2 " << std::endl;
  q1.PushBlockingIfFull(res, 1);
  std::cout << "P1 Pushed " << res << std::endl;
  
  // Push
  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  res = 21.1;
  std::cout << "P1 Trying to push " << res << " to C1P2 " << std::endl;
  q1.PushBlockingIfFull(res, 1);
  std::cout << "P1 Pushed " << res << std::endl;
}

TEST(ThreadsafeQueueTest, queue) {
	std::thread t1(Consumer1AndProducer2), t2(Producer1), t3(Consumer2);
  t1.join();
  t2.join();
  t3.join();
  ASSERT_TRUE(q1.Empty() && q2.Size() == 1);
}
