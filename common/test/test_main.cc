#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <vector>
#include <iostream>
#include <chrono>
#include <string>
#include "helper.h"
#include "timer.h"
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
	lslam::Timer timer("fib");
	fib(10);
	timer.Stop();
	timer.Start();
	fib(15);
	timer.Stop();
	ASSERT_EQ(2, lslam::Timing::GetNumSamples(0));
}

lslam::ThreadSafeQueue<double> q;
void Consumer() {
  double res;
  std::cout << "Trying to pop... " << std::endl;
  q.PopBlocking(res);
  std::cout << "Popped " << res << std::endl;
  
  std::cout << "Trying to pop... " << std::endl;
  q.PopBlocking(res);
  std::cout << "Popped " << res << std::endl;
}

void Producer() {
  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  double res = 11.1;
  std::cout << "Trying to push... " << std::endl;
  q.PushBlockingIfFull(res, 1);
  std::cout << "Pushed " << res << std::endl;
  
  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  res = 21.1;
  std::cout << "Trying to push... " << std::endl;
  q.PushBlockingIfFull(res, 1);
  std::cout << "Pushed " << res << std::endl;
}

TEST(ThreadsafeQueueTest, queue) {
	std::thread t1(Consumer), t2(Producer);
  t1.join();
  t2.join();
  ASSERT_TRUE(q.Empty());
}
