#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace policy_runtime::test::igh_shim {

void reset();
std::size_t last_download_size();
std::span<const std::byte> last_download_data();

struct ClockState {
  std::uint64_t application_time{};
  std::size_t time_updates{};
  std::size_t synchronized_sends{};
  bool time_supplied_before_activation{};
};
ClockState clock_state();
void set_sync_monitor_result(std::uint32_t deviation);

}  // namespace policy_runtime::test::igh_shim
