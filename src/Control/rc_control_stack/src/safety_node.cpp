#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
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

double wrap_pi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// quaternion -> yaw (ROS ENU 기준)
bool quat_to_yaw(const geometry_msgs::msg::Quaternion &q, double &yaw_out) {
  const double norm = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (norm < 1e-6) return false;

  const double w = q.w, x = q.x, y = q.y, z = q.z;
  const double siny_cosp = 2.0 * (w*z + x*y);
  const double cosy_cosp = 1.0 - 2.0 * (y*y + z*z);
  yaw_out = std::atan2(siny_cosp, cosy_cosp);
  return true;
}

} // namespace

class SafetyManagerNode : public rclcpp::Node {
public:
  SafetyManagerNode() : Node("safety_manager_node") {
    using rc_interfaces::msg::DriveCmd;

    // =========================
    // Base Params (기존)
    // =========================
    watchdog_ms_ = declare_parameter<int>("watchdog_ms", 200);
    emergency_timeout_ms_ = declare_parameter<int>("emergency_timeout_ms", 300);

    max_vx_ = declare_parameter<double>("max_vx", 0.6);
    max_vy_ = declare_parameter<double>("max_vy", 0.6);
    max_wz_ = declare_parameter<double>("max_wz", 1.6);

    slew_vx_ = declare_parameter<double>("slew_vx", 2.0);
    slew_vy_ = declare_parameter<double>("slew_vy", 2.0);
    slew_wz_ = declare_parameter<double>("slew_wz", 4.0);

    clamp_emergency_ = declare_parameter<bool>("clamp_emergency", true);

    // =========================
    // IMU Heading Hold Params (NEW)
    // =========================
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
    enable_heading_hold_ = declare_parameter<bool>("enable_heading_hold", true);

    use_orientation_yaw_ = declare_parameter<bool>("use_orientation_yaw", true);
    gyro_bias_z_ = declare_parameter<double>("gyro_bias_z", 0.0);

    // hold engage 조건
    hold_wz_deadband_ = declare_parameter<double>("hold_wz_deadband", 0.10); // |wz_cmd| < 이면 홀드 고려
    hold_engage_speed_ = declare_parameter<double>("hold_engage_speed", 0.08); // |vx| or |vy| >=
    hold_vy_allow_ = declare_parameter<double>("hold_vy_allow", 0.20); // vy 크면 홀드 비활성 (횡이동시)
    hold_min_imu_age_ms_ = declare_parameter<int>("hold_min_imu_age_ms", 0); // 0이면 즉시 사용

    // PID
    hold_kp_ = declare_parameter<double>("hold_kp", 2.5);
    hold_ki_ = declare_parameter<double>("hold_ki", 0.0);
    hold_kd_ = declare_parameter<double>("hold_kd", 0.05);
    hold_i_limit_ = declare_parameter<double>("hold_i_limit", 0.8);
    hold_max_wz_ = declare_parameter<double>("hold_max_wz", 1.0); // DriveCmd wz -1..1에 맞추려면 1.0 추천
    reset_integral_on_release_ = declare_parameter<bool>("reset_integral_on_release", true);

    // =========================
    // Subs/Pubs (기존 + IMU)
    // =========================
    sub_auto_ = create_subscription<DriveCmd>(
      "/control/drive_cmd", rclcpp::QoS(10),
      std::bind(&SafetyManagerNode::on_auto, this, std::placeholders::_1));

    sub_emg_ = create_subscription<DriveCmd>(
      "/control/drive_cmd_emergency", rclcpp::QoS(10),
      std::bind(&SafetyManagerNode::on_emg, this, std::placeholders::_1));

    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SafetyManagerNode::on_imu, this, std::placeholders::_1));

    pub_safe_ = create_publisher<DriveCmd>("/control/drive_cmd_safe", rclcpp::QoS(10));

    const int pub_hz = declare_parameter<int>("publish_hz", 50);
    const auto period = std::chrono::duration<double>(1.0 / std::max(1, pub_hz));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SafetyManagerNode::on_timer, this));

    last_pub_time_ = now();
    last_auto_rx_ = now();
    last_emg_rx_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    // imu state init
    have_imu_time_ = false;
    yaw_ready_ = false;
    holding_ = false;
    yaw_est_ = 0.0;
    yaw_ref_ = 0.0;
    integ_ = 0.0;
    prev_err_ = 0.0;

    RCLCPP_INFO(get_logger(),
      "SafetyManager+IMUHold ready. imu_topic=%s enable_heading_hold=%s",
      imu_topic_.c_str(), enable_heading_hold_ ? "true" : "false");
  }

private:
  using DriveCmd = rc_interfaces::msg::DriveCmd;

  // -------------------------
  // Callbacks
  // -------------------------
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

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    const auto t = rclcpp::Time(msg->header.stamp);

    if (!have_imu_time_) {
      imu_time_ = t;
      have_imu_time_ = true;

      double yaw;
      if (use_orientation_yaw_ && quat_to_yaw(msg->orientation, yaw)) {
        yaw_est_ = yaw;
        yaw_ready_ = true;
      }
      last_imu_rx_ = now();
      return;
    }

    const double dt = (t - imu_time_).seconds();
    imu_time_ = t;
    last_imu_rx_ = now();

    if (dt <= 0.0 || dt > 0.2) {
      return; // 점프/끊김 보호
    }

    // 1) orientation yaw 우선
    double yaw_from_q;
    const bool ok_q = use_orientation_yaw_ && quat_to_yaw(msg->orientation, yaw_from_q);

    if (ok_q) {
      yaw_est_ = yaw_from_q;
      yaw_ready_ = true;
    } else {
      // 2) gyro 적분 (단기 보정용)
      const double gz = msg->angular_velocity.z - gyro_bias_z_;
      yaw_est_ = wrap_pi(yaw_est_ + gz * dt);
      yaw_ready_ = true;
    }
  }

  // -------------------------
  // Helper status
  // -------------------------
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

  // -------------------------
  // IMU heading hold core
  // -------------------------
  DriveCmd apply_heading_hold(const DriveCmd &in, double dt) {
    DriveCmd o = in;

    if (!enable_heading_hold_) {
      release_hold_if_needed();
      return o;
    }
    if (!yaw_ready_) {
      release_hold_if_needed();
      return o;
    }

    // IMU fresh check (선택)
    if (hold_min_imu_age_ms_ > 0) {
      const auto age_ms = (now() - last_imu_rx_).nanoseconds() / 1e6;
      if (age_ms > hold_min_imu_age_ms_) {
        release_hold_if_needed();
        return o;
      }
    }

    const bool enable = o.enable;
    const double vx = o.vx;
    const double vy = o.vy;
    const double wz_cmd = o.wz;

    const bool moving = (std::abs(vx) >= hold_engage_speed_) || (std::abs(vy) >= hold_engage_speed_);
    const bool want_hold = enable
                        && moving
                        && (std::abs(wz_cmd) < hold_wz_deadband_)
                        && (std::abs(vy) < hold_vy_allow_);

    if (!want_hold) {
      release_hold_if_needed();
      return o;
    }

    if (!holding_) {
      yaw_ref_ = yaw_est_;
      holding_ = true;
      integ_ = 0.0;
      prev_err_ = 0.0;
    }

    const double err = wrap_pi(yaw_ref_ - yaw_est_);
    integ_ += err * dt;
    integ_ = clamp(integ_, -hold_i_limit_, +hold_i_limit_);

    const double derr = (err - prev_err_) / std::max(1e-3, dt);
    prev_err_ = err;

    const double wz_corr = hold_kp_ * err + hold_ki_ * integ_ + hold_kd_ * derr;
    double wz_out = wz_cmd + wz_corr;

    // 홀드 보정은 별도 saturate(DriveCmd -1..1 기준이면 1.0 추천)
    wz_out = clamp(wz_out, -hold_max_wz_, +hold_max_wz_);

    o.wz = static_cast<float>(wz_out);
    if (o.source.empty()) o.source = "AUTO_SAFE";
    o.source += "|IMU_HOLD";
    return o;
  }

  void release_hold_if_needed() {
    if (holding_) {
      holding_ = false;
      if (reset_integral_on_release_) integ_ = 0.0;
      prev_err_ = 0.0;
    }
  }

  // -------------------------
  // Timer
  // -------------------------
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
      // emergency면 hold는 무조건 해제
      release_hold_if_needed();

    } else {
      // AUTO safety
      const bool alive = auto_alive(tnow);
      if (!alive) {
        out.enable = false;
        out.vx = 0.0f;
        out.vy = 0.0f;
        out.wz = 0.0f;
        out.source = "SAFETY_WATCHDOG_STOP";
        prev_out_ = out;
        release_hold_if_needed();
      } else {
        out = apply_clamp(auto_cmd);
        out = apply_slew(out, dt);

        if (!out.enable) {
          out.vx = 0.0f;
          out.vy = 0.0f;
          out.wz = 0.0f;
          out.source = "AUTO_DISABLED_STOP";
          prev_out_ = out;
          release_hold_if_needed();
        } else {
          // ✅ 여기서 IMU 보정 적용
          out = apply_heading_hold(out, dt);

          if (out.source.empty()) out.source = "AUTO_SAFE";
          prev_out_ = out;
        }
      }
    }

    pub_safe_->publish(out);
  }

private:
  // =========================
  // Base config (기존)
  // =========================
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

  // =========================
  // IMU hold state (NEW)
  // =========================
  std::string imu_topic_{"/imu/data"};
  bool enable_heading_hold_{true};

  bool use_orientation_yaw_{true};
  double gyro_bias_z_{0.0};

  double hold_wz_deadband_{0.10};
  double hold_engage_speed_{0.08};
  double hold_vy_allow_{0.20};
  int hold_min_imu_age_ms_{0};

  double hold_kp_{2.5};
  double hold_ki_{0.0};
  double hold_kd_{0.05};
  double hold_i_limit_{0.8};
  double hold_max_wz_{1.0};
  bool reset_integral_on_release_{true};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

  bool have_imu_time_{false};
  rclcpp::Time imu_time_{0,0,RCL_ROS_TIME};
  rclcpp::Time last_imu_rx_{0,0,RCL_ROS_TIME};

  bool yaw_ready_{false};
  double yaw_est_{0.0};

  bool holding_{false};
  double yaw_ref_{0.0};
  double integ_{0.0};
  double prev_err_{0.0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyManagerNode>());
  rclcpp::shutdown();
  return 0;
}
