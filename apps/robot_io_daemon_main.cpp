#include <iostream>
#include <string_view>
#include <thread>
#include <chrono>
#include <charconv>

#include "policy_runtime/profiles/loader.hpp"
#include "policy_runtime/robot_io/daemon/daemon.hpp"

namespace {
constexpr std::string_view kUsage = "Usage: robot-io-daemon ROBOT_PROFILE [--cpu CPU]\n";
}  // namespace

int main(int argc, char** argv) {
  // The executable is intentionally a thin composition root. Physical
  // transports and directional devices are created by RobotIoDaemon from the
  // validated profile; this file must not construct EtherCAT/CAN objects.
  if (argc == 2 && (std::string_view(argv[1]) == "--help" ||
                    std::string_view(argv[1]) == "-h")) {
    std::cout << "Run the DDS robot I/O daemon.\n" << kUsage;
    return 0;
  }
  if (argc != 2 && argc != 4) {
    std::cerr << kUsage;
    return 2;
  }
  policy_runtime::RealtimeLoopConfig loop_config;
  if (argc == 4) {
    const std::string_view cpu{argv[3]};
    const auto parsed = std::from_chars(cpu.data(), cpu.data() + cpu.size(), loop_config.cpu);
    if (std::string_view{argv[2]} != "--cpu" || parsed.ec != std::errc{} ||
        parsed.ptr != cpu.data() + cpu.size() || loop_config.cpu < 0 ||
        loop_config.cpu >= CPU_SETSIZE) {
      std::cerr << kUsage;
      return 2;
    }
    cpu_set_t allowed;
    if (pthread_getaffinity_np(pthread_self(), sizeof(allowed), &allowed) != 0 ||
        !CPU_ISSET(loop_config.cpu, &allowed)) {
      std::cerr << "Requested CPU is not in this thread's allowed CPU set\n";
      return 2;
    }
  }
  // Load and validate the static robot description before opening hardware.
  auto profile = policy_runtime::profiles::load_robot_profile(argv[1]);
  if (!profile.has_value()) {
    std::cerr << profile.error().message << '\n';
    return 1;
  }
  if (profile.value().dds.backend !=
      policy_runtime::profiles::RobotIoBackend::dds) {
    std::cerr << "robot profile must select the DDS backend\n";
    return 1;
  }
  // SIGTERM is converted into a daemon shutdown request. The signal handler
  // itself only records the request; shutdown runs in the normal control flow.
  policy_runtime::DaemonSignalLatch signals;
  auto installed = signals.install();
  if (!installed.has_value()) {
    std::cerr << installed.error().message << '\n';
    return 1;
  }
  // Configure compiles the physical topology and prepares all device state,
  // but does not start realtime workers or accept commands yet.
  policy_runtime::SystemMonotonicClock clock;
  policy_runtime::LinuxRealtimeSystem realtime_system;
  policy_runtime::RobotIoDaemon daemon{clock, realtime_system, {}, loop_config};
  auto configured = daemon.configure(profile.value());
  if (!configured.has_value()) {
    std::cerr << configured.error().message << '\n';
    return 1;
  }
  // DDS is the external host/daemon boundary. Topic endpoints are created
  // after configuration and before transport workers start.
  auto attached_dds = daemon.attach_dds(profile.value());
  if (!attached_dds.has_value()) {
    std::cerr << attached_dds.error().message << '\n';
    return 1;
  }
  // start() creates the configured Transport instances (including IgH
  // EtherCAT), opens them, and starts their workers/scheduler.
  auto started = daemon.start();
  if (!started.has_value()) {
    std::cerr << started.error().message << '\n';
    return 1;
  }
  // run() owns the daemon lifecycle until shutdown is requested or a safety
  // condition completes the stop sequence.
  // Formatting and terminal I/O stay off the cyclic thread. Cumulative timing
  // maxima expose brief stalls even if WKC has recovered by the next sample.
  std::jthread diagnostics([&daemon](std::stop_token stop) {
    while (!stop.stop_requested()) {
      std::this_thread::sleep_for(std::chrono::seconds{1});
      if (stop.stop_requested()) break;
      const auto h = daemon.health();
      std::cerr << "[io-health] cycle=" << h.cycle_sequence
                << " rt_guarantee=" << h.realtime_guarantee
                << " wkc=" << h.working_counter
                << " data_valid=" << h.process_data_valid
                << " dc_monitor_ns=" << h.dc_deviation_ns
                << " period_ns=" << h.actual_period_ns
                << " wake_max_ns=" << h.maximum_wake_latency_ns
                << " exec_max_ns=" << h.maximum_execution_time_ns
                << " misses=" << h.deadline_misses
                << " skipped=" << h.skipped_releases
                << " safety=" << h.safety_flags << '\n';
    }
  });
  auto ran = daemon.run();
  diagnostics.request_stop();
  if (!ran.has_value()) {
    std::cerr << ran.error().message << '\n';
    return 1;
  }
  return 0;
}
