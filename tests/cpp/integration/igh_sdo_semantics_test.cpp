#include <cstddef>
#include <memory>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "igh_shim.hpp"
#include "policy_runtime/devices/cia402/elmo_gold.hpp"
#include "policy_runtime/transport/ethercat/master.hpp"

namespace {

using policy_runtime::ElmoGoldDeviceDescription;
using policy_runtime::EthercatAxisConfiguration;
using policy_runtime::EthercatMaster;
using policy_runtime::ErrorCode;
using policy_runtime::IghBackend;
using policy_runtime::MailboxRequestState;
using policy_runtime::profiles::AxisConfig;
using policy_runtime::profiles::Cia402Mode;

AxisConfig axis_config() {
  return AxisConfig{"axis", 0U, 0U, ElmoGoldDeviceDescription::vendor_id(),
                    ElmoGoldDeviceDescription::product_code(),
                    ElmoGoldDeviceDescription::revision(), Cia402Mode::csp, 1000.0,
                    -1.0, 1.0, std::chrono::milliseconds{100}, "arm", 0.1, 0.5};
}

TEST(IghSdoSemanticsTest, SuppliesStartupTimeAndSynchronizesEveryBusCycle) {
  using policy_runtime::test::igh_shim::clock_state;
  policy_runtime::test::igh_shim::reset();
  auto backend = std::make_shared<IghBackend>();
  EthercatMaster master{backend, {EthercatAxisConfiguration{axis_config(), {}}}};
  ASSERT_TRUE(master.open().has_value());
  const auto initial = clock_state();
  EXPECT_TRUE(initial.time_supplied_before_activation);
  EXPECT_GT(initial.application_time, 0U);

  master.cycle({});
  master.cycle({});
  const auto running = clock_state();
  EXPECT_EQ(running.time_updates, initial.time_updates + 2U);
  EXPECT_GE(running.application_time, initial.application_time);
  EXPECT_EQ(running.synchronized_sends, 2U);

  master.close();
  master.cycle({});
  EXPECT_EQ(clock_state().time_updates, running.time_updates);
  EXPECT_EQ(clock_state().synchronized_sends, running.synchronized_sends);
}

TEST(IghSdoSemanticsTest, MeasuresDcDeviationAndMarksMissingRepliesUnavailable) {
  using namespace policy_runtime::test::igh_shim;
  reset();
  auto backend = std::make_shared<IghBackend>();
  EthercatMaster master{backend, {EthercatAxisConfiguration{axis_config(), {}}}};
  std::int64_t measured = -1;
  ASSERT_TRUE(master.set_process_image_handler(
      [](void* context, std::span<std::byte>, const policy_runtime::DomainHealth& health,
         bool) noexcept { *static_cast<std::int64_t*>(context) = health.dc_deviation_ns; },
      &measured).has_value());
  ASSERT_TRUE(master.open().has_value());
  set_sync_monitor_result(731U);
  master.cycle({});
  EXPECT_EQ(measured, -1);
  master.cycle({});
  EXPECT_EQ(measured, 731);
  set_sync_monitor_result(std::numeric_limits<std::uint32_t>::max());
  master.cycle({});
  EXPECT_EQ(measured, -1);
  set_sync_monitor_result(0U);
  master.cycle({});
  EXPECT_EQ(measured, 0);
  master.close();
  ASSERT_TRUE(master.open().has_value());
  master.cycle({});
  EXPECT_EQ(measured, -1);
}

TEST(IghSdoSemanticsTest, DownloadUsesExactCallerSizeAfterUploadChangesDataSize) {
  policy_runtime::test::igh_shim::reset();
  auto backend = std::make_shared<IghBackend>();
  EthercatMaster master{backend, {EthercatAxisConfiguration{axis_config(), {}}}};
  ASSERT_TRUE(master.open().has_value());
  auto& mailbox = master.mailbox(0U);

  auto upload = mailbox.queue_upload({0x2000U, 0U});
  ASSERT_TRUE(upload.has_value());
  mailbox.cycle({});
  master.cycle({});
  master.cycle({});
  ASSERT_TRUE(mailbox.mailbox_status(upload.value()).has_value());
  ASSERT_EQ(mailbox.mailbox_status(upload.value())->state,
            MailboxRequestState::completed);
  ASSERT_EQ(mailbox.mailbox_status(upload.value())->uploaded_bytes.size(), 7U);

  const std::vector<std::byte> caller_data{std::byte{0x34U}, std::byte{0x12U}};
  auto download = mailbox.queue_download({0x2001U, 0U}, caller_data);
  ASSERT_TRUE(download.has_value());
  mailbox.cycle({});
  master.cycle({});

  EXPECT_EQ(policy_runtime::test::igh_shim::last_download_size(), 2U);
  const auto written = policy_runtime::test::igh_shim::last_download_data();
  ASSERT_EQ(written.size(), caller_data.size());
  EXPECT_EQ(written[0], caller_data[0]);
  EXPECT_EQ(written[1], caller_data[1]);
}

TEST(IghSdoSemanticsTest, RejectsDownloadSizesWithoutAPreactivatedRequest) {
  policy_runtime::test::igh_shim::reset();
  auto backend = std::make_shared<IghBackend>();
  EthercatMaster master{backend, {EthercatAxisConfiguration{axis_config(), {}}}};
  ASSERT_TRUE(master.open().has_value());
  auto& mailbox = master.mailbox(0U);
  const std::vector<std::byte> unsupported{std::byte{1U}, std::byte{2U},
                                           std::byte{3U}};
  auto download = mailbox.queue_download({0x2001U, 0U}, unsupported);
  ASSERT_TRUE(download.has_value());

  mailbox.cycle({});
  master.cycle({});

  const auto status = mailbox.mailbox_status(download.value());
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->state, MailboxRequestState::failed);
  ASSERT_TRUE(status->error.has_value());
  EXPECT_EQ(status->error->code, ErrorCode::invalid_argument);
  EXPECT_EQ(policy_runtime::test::igh_shim::last_download_size(), 0U);
}

}  // namespace
