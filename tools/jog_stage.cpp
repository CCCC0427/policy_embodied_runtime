#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "policy_runtime/devices/cia402/axis.hpp"
#include "policy_runtime/profiles/loader.hpp"
#include "policy_runtime/robot_io/daemon/messages.hpp"
#include "policy_runtime/runtime/dds_robot_io.hpp"
#include "policy_runtime/runtime/runtime_host.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  // Ensure CycloneDDS loads the same XML configuration as the daemon
  if (std::getenv("CYCLONEDDS_URI") == nullptr) {
    setenv("CYCLONEDDS_URI", "file:///etc/policy-runtime/cyclonedds.xml", 1);
  }

  std::string profile_path = "configs/robot_profiles/sea_linear_stage.json";
  if (argc > 1) {
    profile_path = argv[1];
  }

  std::cout << "[JogStage] Loading robot profile: " << profile_path << "...\n";
  auto profile = policy_runtime::profiles::load_robot_profile(profile_path);
  if (!profile.has_value()) {
    std::cerr << "[JogStage] Failed to load profile: " << profile.error().message << "\n";
    return 1;
  }

  bool is_cst = (!profile.value().axes.empty() &&
                 profile.value().axes[0].mode == policy_runtime::profiles::Cia402Mode::cst);

  double param_val = is_cst ? 0.03 : 100.0;
  if (argc > 2) {
    param_val = std::stod(argv[2]);
  }

  std::cout << "[JogStage] Mode detected: " << (is_cst ? "CST (力矩控制模式)" : "CSP (位置控制模式)") << "\n";
  std::cout << "[JogStage] Connecting to DDS robot I/O daemon...\n";
  auto robot_io_res = policy_runtime::make_dds_runtime_robot_io(profile.value());
  if (!robot_io_res.has_value()) {
    std::cerr << "[JogStage] DDS error: " << robot_io_res.error().message << "\n";
    return 1;
  }
  auto& robot_io = *robot_io_res.value();

  std::cout << "[JogStage] Waiting for feedback and handshake with daemon (up to 5s)...\n";
  double initial_pos = 0.0;
  double initial_effort = 0.0;
  bool feedback_received = false;
  for (int i = 0; i < 100; ++i) {
    auto fb = robot_io.read_feedback();
    if (fb.has_value() && fb.value().sequence > 0) {
      initial_pos = fb.value().axes[0].position;
      initial_effort = fb.value().axes[0].effort;
      feedback_received = true;
      std::cout << "[JogStage] Connected! Current position: " << initial_pos
                << ", Current effort: " << initial_effort
                << ", Statusword: 0x" << std::hex << fb.value().axes[0].status_word << std::dec
                << ", ModeDisplay: " << (int)fb.value().axes[0].mode_display << "\n";
      break;
    }
    std::this_thread::sleep_for(50ms);
  }

  if (!feedback_received) {
    std::cerr << "[JogStage] Error: Cannot connect to daemon. Please ensure:\n"
              << "  1. robot-io-daemon is running in Terminal 1\n"
              << "  2. You run jog-stage with 'sudo': sudo ./build/jog-stage ...\n";
    return 1;
  }

  uint64_t base_seq = 1000;
  {
    auto fb = robot_io.read_feedback();
    if (fb.has_value()) {
      base_seq = fb.value().sequence + 1000;
      // 检测是否处于故障报警状态 (Bit 3 == 1)
      if ((fb.value().axes[0].status_word & 0x0008U) != 0U) {
        std::cout << "[JogStage] 检测到驱动器处于故障报警状态 (0x" << std::hex
                  << fb.value().axes[0].status_word << ")！\n" << std::dec;
        std::cout << "[JogStage] 正在自动发送【CiA 402 故障复位脉冲 (Fault Reset)】...\n";

        for (int r = 0; r < 20; ++r) {
          timespec ts{};
          clock_gettime(CLOCK_MONOTONIC, &ts);
          int64_t now_ns = static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;

          policy_runtime::AxisCommand reset_cmd{};
          reset_cmd.sequence = base_seq + r + 1;
          reset_cmd.timestamp_ns = now_ns;
          reset_cmd.flags = policy_runtime::kAxisCommandFaultReset;
          reset_cmd.target = is_cst ? 0.0 : initial_pos;

          std::vector<policy_runtime::AxisCommand> cmds = {reset_cmd};
          std::vector<policy_runtime::St3215ServoCommand> servos{};
          robot_io.publish_commands(cmds, servos, reset_cmd.sequence, now_ns);
          std::this_thread::sleep_for(20ms);
        }

        base_seq += 30;
        std::this_thread::sleep_for(200ms);

        auto fb_after = robot_io.read_feedback();
        if (fb_after.has_value()) {
          uint16_t sw = fb_after.value().axes[0].status_word;
          std::cout << "[JogStage] 复位脉冲已发送，当前状态字: 0x" << std::hex << sw << std::dec << "\n";
          if ((sw & 0x0008U) != 0U) {
            std::cerr << "[JogStage] 警告: 故障位仍未消除 (0x" << std::hex << sw << std::dec
                      << ")，请检查硬件是否有硬急停触发！\n";
            return 1;
          }
          std::cout << "[JogStage] 故障已成功消除，进入就绪状态！\n";
          initial_pos = fb_after.value().axes[0].position;
        }
      }
    }
  }

  if (is_cst) {
    // ----------------------------------------------------
    // CST 力矩控制模式
    // ----------------------------------------------------
    double target_torque = param_val;
    std::cout << "\n======================================================\n";
    std::cout << ">>> CST 力矩控制测试 <<<\n";
    std::cout << ">>> 目标测试力矩: " << target_torque << " (额定力矩的 " << (target_torque * 100.0) << "%) <<<\n";
    std::cout << ">>> 流程: 2秒平滑加力 -> 5秒保持力矩 -> 2秒平滑泄力归零 <<<\n";
    std::cout << "======================================================\n";

    const int ramp_steps = 100;   // 2.0 秒加力
    const int hold_steps = 250;   // 5.0 秒保持
    const int release_steps = 100;// 2.0 秒泄力
    const int total_steps = ramp_steps + hold_steps + release_steps;

    for (int step = 0; step < total_steps; ++step) {
      timespec ts{};
      clock_gettime(CLOCK_MONOTONIC, &ts);
      int64_t now_ns = static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;

      double current_target_torque = 0.0;
      std::string phase_str = "";

      if (step < ramp_steps) {
        double progress = static_cast<double>(step) / ramp_steps;
        double factor = 0.5 * (1.0 - std::cos(progress * M_PI));
        current_target_torque = factor * target_torque;
        phase_str = "平滑加力中";
      } else if (step < ramp_steps + hold_steps) {
        current_target_torque = target_torque;
        phase_str = "力矩保持中";
      } else {
        int rel_step = step - (ramp_steps + hold_steps);
        double progress = static_cast<double>(rel_step) / release_steps;
        double factor = 0.5 * (1.0 + std::cos(progress * M_PI));
        current_target_torque = factor * target_torque;
        phase_str = "平滑泄力中";
      }

      uint64_t seq = base_seq + step + 1;
      policy_runtime::AxisCommand cmd{};
      cmd.sequence = seq;
      cmd.timestamp_ns = now_ns;
      cmd.flags = policy_runtime::kAxisCommandEnable;
      cmd.target = current_target_torque;

      std::vector<policy_runtime::AxisCommand> axis_cmds = {cmd};
      std::vector<policy_runtime::St3215ServoCommand> servo_cmds{};
      auto pub_res = robot_io.publish_commands(axis_cmds, servo_cmds, seq, now_ns);
      if (!pub_res.has_value()) {
        std::cerr << "\n[JogStage] Publish error: " << pub_res.error().message << "\n";
        break;
      }

      auto fb = robot_io.read_feedback();
      if (fb.has_value()) {
        std::cout << "\r[" << phase_str << "]"
                  << " 目标力矩: " << std::fixed << std::setprecision(3) << current_target_torque
                  << " | 实际力矩: " << fb.value().axes[0].effort
                  << " | 实际位置: " << std::setprecision(1) << fb.value().axes[0].position
                  << " | 状态: 0x" << std::hex << fb.value().axes[0].status_word
                  << " | 模式: " << std::dec << (int)fb.value().axes[0].mode_display
                  << std::flush;
      }

      std::this_thread::sleep_for(20ms);
    }

    std::cout << "\n\n[JogStage] 力矩已完全泄放归零，测试安全结束！\n";

  } else {
    // ----------------------------------------------------
    // CSP 位置控制模式 (原逻辑保留)
    // ----------------------------------------------------
    double delta_pos = param_val;
    std::cout << "\n======================================================\n";
    std::cout << ">>> CSP 位置滑行测试: 目标位移 " << delta_pos << " 脉冲 <<<\n";
    std::cout << ">>> 运动将平滑执行 5 秒 <<<\n";
    std::cout << "======================================================\n";

    const int total_steps = 250;  // 5.0 秒
    for (int step = 0; step < total_steps; ++step) {
      timespec ts{};
      clock_gettime(CLOCK_MONOTONIC, &ts);
      int64_t now_ns = static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;

      double progress = static_cast<double>(step) / total_steps;
      double smooth_factor = 0.5 * (1.0 - std::cos(progress * M_PI));
      double current_target = initial_pos + smooth_factor * delta_pos;

      uint64_t seq = base_seq + step + 1;
      policy_runtime::AxisCommand cmd{};
      cmd.sequence = seq;
      cmd.timestamp_ns = now_ns;
      cmd.flags = policy_runtime::kAxisCommandEnable;
      cmd.target = current_target;

      std::vector<policy_runtime::AxisCommand> axis_cmds = {cmd};
      std::vector<policy_runtime::St3215ServoCommand> servo_cmds{};
      auto pub_res = robot_io.publish_commands(axis_cmds, servo_cmds, seq, now_ns);
      if (!pub_res.has_value()) {
        std::cerr << "\n[JogStage] Publish error: " << pub_res.error().message << "\n";
        break;
      }

      auto fb = robot_io.read_feedback();
      if (fb.has_value()) {
        std::cout << "\r[滑行中 " << (int)(progress * 100) << "%]"
                  << " 目标: " << current_target
                  << " | 实际: " << fb.value().axes[0].position
                  << " | 状态: 0x" << std::hex << fb.value().axes[0].status_word
                  << " | 模式: " << std::dec << (int)fb.value().axes[0].mode_display
                  << std::flush;
      }

      std::this_thread::sleep_for(20ms);
    }

    std::cout << "\n[JogStage] 目标位移已下发完毕。\n";
  }

  return 0;
}
