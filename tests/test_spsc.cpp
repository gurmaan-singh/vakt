#include "vakt/spsc_queue.hpp"

#include <gtest/gtest.h>

#include <thread>

using namespace vakt;

TEST(Spsc, FullAndEmpty) {
  SpscQueue<int, 4> q;  // usable capacity 3
  EXPECT_FALSE(q.try_pop().has_value());
  EXPECT_TRUE(q.try_push(1));
  EXPECT_TRUE(q.try_push(2));
  EXPECT_TRUE(q.try_push(3));
  EXPECT_FALSE(q.try_push(4));
  EXPECT_EQ(q.try_pop(), 1);
  EXPECT_TRUE(q.try_push(4));
  EXPECT_EQ(q.try_pop(), 2);
  EXPECT_EQ(q.try_pop(), 3);
  EXPECT_EQ(q.try_pop(), 4);
  EXPECT_TRUE(q.empty());
}

TEST(Spsc, TwoThreadsKeepOrderAndLoseNothing) {
  constexpr int kItems = 2'000'000;
  SpscQueue<int, 1024> q;
  std::thread producer([&] {
    for (int i = 0; i < kItems; ++i)
      while (!q.try_push(i)) std::this_thread::yield();
  });
  int expected = 0;
  while (expected < kItems) {
    if (auto v = q.try_pop()) {
      ASSERT_EQ(*v, expected);
      ++expected;
    } else {
      std::this_thread::yield();
    }
  }
  producer.join();
  EXPECT_TRUE(q.empty());
}
