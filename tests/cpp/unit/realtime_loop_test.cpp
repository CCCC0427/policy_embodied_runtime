#include <gtest/gtest.h>

#include "policy_runtime/robot_io/daemon/realtime_loop.hpp"

namespace {
using namespace policy_runtime;

class Clock final : public MonotonicClock {
 public:
  std::int64_t now_ns() const noexcept override { return now; }
  int sleep_until_ns(std::int64_t deadline) noexcept override {
    now = deadline;
    return 0;
  }
  std::int64_t now{};
};

class System final : public RealtimeSystem {
 public:
  bool capture_current_thread_state() noexcept override { return true; }
  bool preempt_rt_kernel() const noexcept override { return rt; }
  bool lock_process_memory() noexcept override { return true; }
  bool pin_current_thread(int) noexcept override { return true; }
  bool set_current_thread_fifo(int) noexcept override { return fifo; }
  void restore_current_thread_state() noexcept override { ++restores; }
  void unlock_process_memory() noexcept override { ++unlocks; }
  bool rt{};
  bool fifo{true};
  int restores{};
  int unlocks{};
};

TEST(RealtimeSetupTest, NonRtKernelKeepsSchedulingButDoesNotClaimGuarantee) {
  Clock clock;
  System system;
  RealtimeLoop loop{clock, system};
  auto setup = loop.prepare();
  ASSERT_TRUE(setup.has_value());
  EXPECT_FALSE(setup.value().realtime_guarantee);
  EXPECT_TRUE(setup.value().sched_fifo);
  EXPECT_TRUE(setup.value().affinity_set);
  EXPECT_TRUE(setup.value().memory_locked);
  EXPECT_EQ(system.restores, 0);
  EXPECT_EQ(system.unlocks, 0);
  EXPECT_FALSE(loop.prepare().has_value());
  EXPECT_EQ(loop.wait_next().sleep_error, 0);
  EXPECT_EQ(clock.now, 1'000'000);
  loop.release();
  EXPECT_EQ(system.restores, 1);
  EXPECT_EQ(system.unlocks, 1);
  loop.release();
  EXPECT_EQ(system.restores, 1);
  EXPECT_EQ(system.unlocks, 1);
}

TEST(RealtimeSetupTest, PartialFailureRestoresThreadAndMemory) {
  Clock clock;
  System system;
  system.fifo = false;
  RealtimeLoop loop{clock, system};
  auto setup = loop.prepare();
  ASSERT_TRUE(setup.has_value());
  EXPECT_FALSE(setup.value().realtime_guarantee);
  EXPECT_EQ(system.restores, 1);
  EXPECT_EQ(system.unlocks, 1);
  loop.release();
  EXPECT_EQ(system.restores, 1);
  EXPECT_EQ(system.unlocks, 1);
}

TEST(RealtimeSetupTest, RtKernelRetainsGuaranteeAndReleasesAtDestruction) {
  Clock clock;
  System system;
  system.rt = true;
  {
    RealtimeLoop loop{clock, system};
    auto setup = loop.prepare();
    ASSERT_TRUE(setup.has_value());
    EXPECT_TRUE(setup.value().realtime_guarantee);
    EXPECT_EQ(system.restores, 0);
  }
  EXPECT_EQ(system.restores, 1);
  EXPECT_EQ(system.unlocks, 1);
}
}  // namespace
