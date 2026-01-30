#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rc_interfaces/msg/drive_cmd.hpp"

using namespace std::chrono_literals;

namespace {

double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

double slew(double prev, double target, double rate_per_s, double dt) {
  if (rate_per_s <= 0.0) return target;
  const double max_d = std::abs(rate_per_s) * dt;
  const double d = target - prev;
  if (d >  max_d) return prev + max_d;
  if (d < -max_d) return prev - max_d;
  return target;
}

} // namespace

class SafetyManagerNode : public rclcpp::Node {
public:
  SafetyManagerNode() : Node("safety_manager_node") {
    // Params
    watchdog_ms_ = declare_parameter<int>("watchdog_ms", 200);
    emergency_timeout_ms_ = declare_parameter<int>("emergency_timeout_ms", 300);

    max_vx_ = declare_parameter<double>("max_vx", 0.6);
    max_vy_ = declare_parameter<double>("max_vy", 0.6);
    max_wz_ = declare_parameter<double>("max_wz", 1.6);

    slew_vx_ = declare_parameter<double>("slew_vx", 2.0);
    slew_vy_ = declare_parameter<double>("slew_vy", 2.0);
    slew_wz_ = declare_parameter<double>("slew_wz", 4.0);

    clamp_emergency_ = declare_parameter<bool>("clamp_emergency", true);

    using rc_interfaces::msg::DriveCmd;

    sub_auto_ = create_subscription<DriveCmd>(
      "/control/drive_cmd", rclcpp::QoS(10),
      std::bind(&SafetyManagerNode::on_auto, this, std::placeholders::_1));

    sub_emg_ = create_subscription<DriveCmd>(
      "/control/drive_cmd_emergency", rclcpp::QoS(10),
      std::bind(&SafetyManagerNode::on_emg, this, std::placeholders::_1));

    pub_safe_ = create_publisher<DriveCmd>("/control/drive_cmd_safe", rclcpp::QoS(10));

    const int pub_hz = declare_parameter<int>("publish_hz", 50);
    const auto period = std::chrono::duration<double>(1.0 / std::max(1, pub_hz));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SafetyManagerNode::on_timer, this));

    last_pub_time_ = now();
    last_auto_rx_ = now();
    last_emg_rx_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    RCLCPP_INFO(get_logger(),
      "SafetyManager ready. watchdog_ms=%d emergency_timeout_ms=%d publish_hz=%d",
      watchdog_ms_, emergency_timeout_ms_, pub_hz);
  }

private:
  using DriveCmd = rc_interfaces::msg::DriveCmd;

  void on_auto(const DriveCmd::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mu_);
    last_auto_ = *msg;
    last_auto_rx_ = now();
  }

  void on_emg(const DriveCmd::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mu_);
    last_emg_ = *msg;
    last_emg_rx_ = now();
  }

  bool emergency_active(const rclcpp::Time &tnow) const {
    if (last_emg_rx_.nanoseconds() == 0) return false;
    const auto age_ms = (tnow - last_emg_rx_).nanoseconds() / 1e6;
    return age_ms <= emergency_timeout_ms_;
  }

  bool auto_alive(const rclcpp::Time &tnow) const {
    const auto age_ms = (tnow - last_auto_rx_).nanoseconds() / 1e6;
    return age_ms <= watchdog_ms_;
  }

  DriveCmd apply_clamp(const DriveCmd &in) const {
    DriveCmd o = in;
    o.vx = static_cast<float>(clamp(o.vx, -max_vx_, max_vx_));
    o.vy = static_cast<float>(clamp(o.vy, -max_vy_, max_vy_));
    o.wz = static_cast<float>(clamp(o.wz, -max_wz_, max_wz_));
    return o;
  }

  DriveCmd apply_slew(const DriveCmd &in, double dt) {
    DriveCmd o = in;
    o.vx = static_cast<float>(slew(prev_out_.vx, o.vx, slew_vx_, dt));
    o.vy = static_cast<float>(slew(prev_out_.vy, o.vy, slew_vy_, dt));
    o.wz = static_cast<float>(slew(prev_out_.wz, o.wz, slew_wz_, dt));
    return o;
  }

  void on_timer() {
    const auto tnow = now();
    const double dt = std::max(1e-3, (tnow - last_pub_time_).seconds());
    last_pub_time_ = tnow;

    DriveCmd auto_cmd;
    DriveCmd emg_cmd;

    {
      std::lock_guard<std::mutex> lk(mu_);
      auto_cmd = last_auto_;
      emg_cmd = last_emg_;
    }

    DriveCmd out;
    const bool emg_active = emergency_active(tnow);

    if (emg_active) {
      // EMERGENCY pass-through
      out = emg_cmd;
      if (clamp_emergency_) out = apply_clamp(out);

      out.source = out.source.empty() ? "EMERGENCY" : out.source;
      prev_out_ = out;

    } else {
      // AUTO safety
      const bool alive = auto_alive(tnow);
      if (!alive) {
        out.enable = false;
        out.vx = 0.0f;
        out.vy = 0.0f;
        out.wz = 0.0f;
        out.source = "SAFETY_WATCHDOG_STOP";
        prev_out_ = out; // 튐 방지용(추천)
      } else {
        out = apply_clamp(auto_cmd);
        out = apply_slew(out, dt);
        if (!out.enable) {
          out.vx = 0.0f;
          out.vy = 0.0f;
          out.wz = 0.0f;
          out.source = "AUTO_DISABLED_STOP";
        } else {
          if (out.source.empty()) out.source = "AUTO_SAFE";
        }
        prev_out_ = out;
      }
    }

    pub_safe_->publish(out);
  }

private:
  int watchdog_ms_{200};
  int emergency_timeout_ms_{300};

  double max_vx_{0.6};
  double max_vy_{0.6};
  double max_wz_{1.6};

  double slew_vx_{2.0};
  double slew_vy_{2.0};
  double slew_wz_{4.0};

  bool clamp_emergency_{true};

  rclcpp::Subscription<DriveCmd>::SharedPtr sub_auto_;
  rclcpp::Subscription<DriveCmd>::SharedPtr sub_emg_;
  rclcpp::Publisher<DriveCmd>::SharedPtr pub_safe_;
  rclcpp::TimerBase::SharedPtr timer_;

  mutable std::mutex mu_;
  DriveCmd last_auto_;
  DriveCmd last_emg_;
  DriveCmd prev_out_;

  rclcpp::Time last_auto_rx_;
  rclcpp::Time last_emg_rx_;
  rclcpp::Time last_pub_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyManagerNode>());
  rclcpp::shutdown();
  return 0;
}
