// control_stack_node.cpp (AUTO controller only: publishes /control/drive_cmd)
// Safety + emergency mux is handled by safety_manager_node (src/safety_node.cpp) -> /control/drive_cmd_safe

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <atomic>
#include <cctype>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "rc_interfaces/msg/lane_status.hpp"
#include "rc_interfaces/msg/drive_cmd.hpp"
#include "rc_interfaces/msg/parking_line_status.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

namespace {

double clamp(double v, double lo, double hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

double slew(double prev, double target, double rate_per_s, double dt) {
  const double r = std::abs(rate_per_s);
  const double max_d = r * dt;
  const double d = target - prev;
  if (d >  max_d) return prev + max_d;
  if (d < -max_d) return prev - max_d;
  return target;
}

static inline std::string upper(std::string s) {
  for (auto &c : s) c = (char)std::toupper((unsigned char)c);
  return s;
}

static inline double wrap_pi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static inline double yaw_from_quat(const geometry_msgs::msg::Quaternion &q) {
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
  return yaw;
}

} // namespace

enum class ParkSubState {
  LINE_ALIGN = 0,
  SLOT_MARKER_ALIGN = 1,
  DONE = 2
};

class ControlStackNode : public rclcpp::Node {
public:
  ControlStackNode()
  : Node("control_stack_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_) {

    // ===================== Topics =====================
    declare_parameter<std::string>("lane_topic", "/perception/lane");
    declare_parameter<std::string>("parking_line_topic", "/perception/parking_line");

    // PARK용 slot marker pose
    declare_parameter<std::string>("slot_marker_pose_topic", "/perception/slot_marker_pose");
    // mission ALIGN용 marker pose
    declare_parameter<std::string>("marker_pose_topic", "/perception/marker_pose");

    declare_parameter<std::string>("enable_drive_topic", "/control/enable_drive");
    declare_parameter<std::string>("mission_state_topic", "/mission/state");
    declare_parameter<std::string>("turn_target_topic", "/mission/turn_target_rad");

    // mission -> control handshake
    declare_parameter<std::string>("align_done_topic", "/mission/align_done");

    // outputs (RAW only)
    declare_parameter<bool>("publish_raw_cmd", true);
    declare_parameter<std::string>("out_raw_topic", "/control/drive_cmd");

    // emergency suppress (recommended)
    declare_parameter<std::string>("emergency_topic", "/control/drive_cmd_emergency");
    declare_parameter<int>("emergency_timeout_ms", 300);

    // unload trigger
    declare_parameter<std::string>("unload_topic", "/unload/start");

    // TF frames
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("front_cam_frame", "camera_front");
    declare_parameter<std::string>("side_cam_frame", "camera_side");

    // ===================== Timer =====================
    declare_parameter<double>("timer_hz", 30.0);

    // ===================== Drive (lane follower) =====================
    declare_parameter<double>("linear_speed", 0.20);
    declare_parameter<double>("k_wz", 1.2);
    declare_parameter<double>("max_error_norm", 0.30);

    declare_parameter<int>("lane_timeout_ms", 300);
    declare_parameter<double>("min_quality", 0.2);
    declare_parameter<bool>("stop_on_lane_lost", true);

    declare_parameter<double>("wz_max", 1.0);
    declare_parameter<double>("vx_max", 0.25);
    declare_parameter<double>("vx_min", 0.08);
    declare_parameter<double>("vx_when_turning", 0.12);

    declare_parameter<double>("vx_slew", 1.0);
    declare_parameter<double>("wz_slew", 3.0);

    declare_parameter<bool>("invert_steering", false);

    // ADVANCE_TO_CENTER
    declare_parameter<double>("advance_vx", 0.12);
    declare_parameter<double>("advance_wz", 0.0);
    declare_parameter<double>("advance_vx_slew", 1.2);
    declare_parameter<double>("advance_wz_slew", 6.0);
    declare_parameter<bool>("advance_stop_on_lane_lost", false);
    declare_parameter<bool>("advance_require_lane_fresh", false);

    // TURNING
    declare_parameter<double>("wz_turn", 0.8);
    declare_parameter<double>("wz_turn_slew", 6.0);
    declare_parameter<double>("turn_vx", 0.0);
    declare_parameter<bool>("turn_invert", false);

    // global knobs
    declare_parameter<double>("speed_scale", 1.0);
    declare_parameter<double>("wz_scale", 1.0);
    declare_parameter<bool>("enforce_vx_min", false);

    declare_parameter<bool>("stop_on_low_quality", false);
    declare_parameter<double>("low_quality_speed_scale", 0.3);
    declare_parameter<double>("turn_threshold", 0.3);
    
    // ===== NEW: angle + offset steering (robust) =====
    declare_parameter<double>("k_psi", 2.0);            // heading gain (lane.angle)
    declare_parameter<double>("k_y", 2.5);              // lateral gain (lane.offset_norm)
    declare_parameter<double>("v0", 0.05);              // avoid div0
    declare_parameter<double>("err_lpf_alpha", 0.25);   // error LPF alpha (0~1)
    declare_parameter<double>("angle_max_rad", 0.7);    // clamp lane.angle (rad) ~40deg
    declare_parameter<double>("deadband_offset", 0.02); // ignore tiny offset jitter
    declare_parameter<double>("deadband_angle", 0.03);  // ignore tiny angle jitter (rad)
    declare_parameter<double>("lowq_steer_gain", 0.35); // q<min_q => steering soften (0~1)

    // ===================== Parking Controller =====================
    declare_parameter<std::string>("parking_active_state_name", "PARK");
    declare_parameter<bool>("parking_publish_disable_once_on_deactivate", true);

    // LINE_ALIGN
    declare_parameter<double>("park_k_offset", 1.2);
    declare_parameter<double>("park_k_angle", 0.8);
    declare_parameter<double>("park_max_wz", 1.2);
    declare_parameter<double>("park_vx_cmd", 0.10);
    declare_parameter<double>("park_vy_cmd", 0.0);

    declare_parameter<bool>("park_require_valid", true);
    declare_parameter<double>("park_min_quality", 0.20);

    declare_parameter<double>("park_align_offset_eps", 0.03);
    declare_parameter<double>("park_align_angle_eps", 0.10);
    declare_parameter<bool>("park_auto_hold_when_aligned", true);

    // SLOT_MARKER_ALIGN
    declare_parameter<double>("park_k_x", 0.8);
    declare_parameter<double>("park_k_y", 1.2);
    declare_parameter<double>("park_k_yaw", 1.0);

    declare_parameter<double>("park_vx_max", 0.12);
    declare_parameter<double>("park_vy_max", 0.18);
    declare_parameter<double>("park_wz_max2", 1.2);

    declare_parameter<double>("park_desired_standoff_x", 0.20);

    declare_parameter<double>("park_eps_x", 0.02);
    declare_parameter<double>("park_eps_y", 0.02);
    declare_parameter<double>("park_eps_yaw", 0.10);
    declare_parameter<bool>("park_use_marker_yaw", false);

    // ===================== ALIGN_TO_MARKER =====================
    declare_parameter<double>("align_k_x", 0.8);
    declare_parameter<double>("align_k_y", 1.2);
    declare_parameter<double>("align_k_yaw", 1.0);

    declare_parameter<double>("align_vx_max", 0.10);
    declare_parameter<double>("align_vy_max", 0.12);
    declare_parameter<double>("align_wz_max", 0.9);

    declare_parameter<double>("align_desired_x", 0.20);
    declare_parameter<double>("align_desired_y", 0.0);

    declare_parameter<double>("align_eps_x", 0.02);
    declare_parameter<double>("align_eps_y", 0.02);
    declare_parameter<double>("align_eps_yaw", 0.10);

    declare_parameter<bool>("align_use_marker_yaw", false);
    declare_parameter<int>("marker_pose_timeout_ms", 250);

    // ===================== Read IO =====================
    lane_topic_ = get_parameter("lane_topic").as_string();
    parking_line_topic_ = get_parameter("parking_line_topic").as_string();
    slot_marker_pose_topic_ = get_parameter("slot_marker_pose_topic").as_string();
    marker_pose_topic_ = get_parameter("marker_pose_topic").as_string();

    enable_drive_topic_ = get_parameter("enable_drive_topic").as_string();
    mission_state_topic_ = get_parameter("mission_state_topic").as_string();
    turn_target_topic_ = get_parameter("turn_target_topic").as_string();
    align_done_topic_ = get_parameter("align_done_topic").as_string();

    publish_raw_cmd_ = get_parameter("publish_raw_cmd").as_bool();
    out_raw_topic_ = get_parameter("out_raw_topic").as_string();

    emergency_topic_ = get_parameter("emergency_topic").as_string();

    unload_topic_ = get_parameter("unload_topic").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    front_cam_frame_ = get_parameter("front_cam_frame").as_string();
    side_cam_frame_ = get_parameter("side_cam_frame").as_string();

    parking_active_state_name_ = upper(get_parameter("parking_active_state_name").as_string());

    // ===================== Pub/Sub =====================
    if (publish_raw_cmd_) {
      pub_raw_ = create_publisher<rc_interfaces::msg::DriveCmd>(out_raw_topic_, 10);
    }
    pub_unload_ = create_publisher<std_msgs::msg::Bool>(unload_topic_, 10);
    pub_align_done_ = create_publisher<std_msgs::msg::Bool>(align_done_topic_, 10);

    sub_lane_ = create_subscription<rc_interfaces::msg::LaneStatus>(
      lane_topic_, 10,
      [this](rc_interfaces::msg::LaneStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mu_lane_);
        last_lane_ = *msg;
        last_lane_rx_ = now();
        has_lane_ = true;
      });

    sub_pline_ = create_subscription<rc_interfaces::msg::ParkingLineStatus>(
      parking_line_topic_, 10,
      [this](rc_interfaces::msg::ParkingLineStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mu_pline_);
        last_pline_ = *msg;
        has_pline_ = true;
      });

    sub_slot_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      slot_marker_pose_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mu_slot_);
        last_slot_pose_cam_ = *msg;
        last_slot_rx_ = now();
        has_slot_pose_ = true;
      });

    sub_marker_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      marker_pose_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mu_marker_pose_);
        last_marker_pose_cam_ = *msg;
        last_marker_pose_rx_ = now();
        has_marker_pose_ = true;
      });

    sub_enable_ = create_subscription<std_msgs::msg::Bool>(
      enable_drive_topic_, 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        enable_drive_.store(msg->data);
      });

    sub_state_ = create_subscription<std_msgs::msg::String>(
      mission_state_topic_, 10,
      [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mu_state_);
        mission_state_ = upper(msg->data);
      });

    sub_turn_target_ = create_subscription<std_msgs::msg::Float32>(
      turn_target_topic_, 10,
      [this](std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mu_state_);
        turn_target_rad_ = msg->data;
      });

    // emergency activity detector (for suppress/reset)
    sub_emg_ = create_subscription<rc_interfaces::msg::DriveCmd>(
      emergency_topic_, 10,
      [this](rc_interfaces::msg::DriveCmd::SharedPtr) {
        std::lock_guard<std::mutex> lk(mu_emg_);
        last_emg_rx_ = now();
        has_emg_ = true;
      });

    // IMU (optional)
    declare_parameter<bool>("use_imu", true);
    declare_parameter<std::string>("imu_topic", "/imu/data");
    if (get_parameter("use_imu").as_bool()) {
      imu_topic_ = get_parameter("imu_topic").as_string();
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(mu_imu_);
          gyro_z_ = (double)msg->angular_velocity.z;
          last_imu_time_ = now();
          has_imu_ = true;
        });
    }

    double hz = get_parameter("timer_hz").as_double();
    if (hz < 1.0) hz = 10.0;
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / hz),
                               std::bind(&ControlStackNode::on_timer_, this));
    last_loop_t_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(get_logger(),
      "control_stack_node (AUTO only) start.\n"
      " in: lane=%s parking_line=%s slot_pose=%s marker_pose=%s enable=%s state=%s turn_target=%s emergency=%s\n"
      " out: raw=%s (publish_raw=%s) unload=%s align_done=%s\n"
      " frames: base=%s front_cam=%s side_cam=%s parking_active_state=%s",
      lane_topic_.c_str(), parking_line_topic_.c_str(),
      slot_marker_pose_topic_.c_str(), marker_pose_topic_.c_str(),
      enable_drive_topic_.c_str(), mission_state_topic_.c_str(), turn_target_topic_.c_str(),
      emergency_topic_.c_str(),
      out_raw_topic_.c_str(), publish_raw_cmd_ ? "true" : "false",
      unload_topic_.c_str(), align_done_topic_.c_str(),
      base_frame_.c_str(), front_cam_frame_.c_str(), side_cam_frame_.c_str(),
      parking_active_state_name_.c_str());
  }

private:
  // ---------- helpers ----------
  double speed_scale_() { return clamp(get_parameter("speed_scale").as_double(), 0.0, 1.0); }
  double wz_scale_()    { return clamp(get_parameter("wz_scale").as_double(), 0.0, 1.0); }

  std::string get_mission_state_() {
    std::lock_guard<std::mutex> lk(mu_state_);
    return mission_state_;
  }

  float get_turn_target_() {
    std::lock_guard<std::mutex> lk(mu_state_);
    return turn_target_rad_;
  }

  bool emergency_active_() {
    std::lock_guard<std::mutex> lk(mu_emg_);
    if (!has_emg_) return false;
    const int to_ms = get_parameter("emergency_timeout_ms").as_int();
    const double age_ms = (now() - last_emg_rx_).seconds() * 1000.0;
    return age_ms <= (double)to_ms;
  }

  // ---------- publish RAW ----------
  void publish_raw_(bool enable, double vx, double vy, double wz, const std::string &src) {
    rc_interfaces::msg::DriveCmd cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = base_frame_;
    cmd.enable = enable;
    cmd.vx = (float)vx;
    cmd.vy = (float)vy;
    cmd.wz = (float)wz;
    cmd.source = src;

    last_raw_ = cmd;
    last_raw_rx_ = now();
    has_raw_ = true;

    if (publish_raw_cmd_ && pub_raw_) pub_raw_->publish(cmd);
  }

  void publish_stop_raw_(const std::string &why) {
    publish_raw_(false, 0.0, 0.0, 0.0, "ctrl_raw(stop):" + why);
    prev_vx_ = 0.0;
    prev_wz_ = 0.0;
  }

  void publish_align_done_once_(const std::string &why) {
    if (align_done_sent_) return;
    std_msgs::msg::Bool b;
    b.data = true;
    pub_align_done_->publish(b);
    align_done_sent_ = true;
    RCLCPP_INFO(get_logger(), "ALIGN_DONE published (%s) -> %s", why.c_str(), align_done_topic_.c_str());
  }

  // ===================== DRIVE =====================
  void drive_turning_(double dt) {
    const bool invert = get_parameter("turn_invert").as_bool();
    const double wz_turn = std::max(0.1, get_parameter("wz_turn").as_double());
    const double wz_turn_slew = std::max(0.1, get_parameter("wz_turn_slew").as_double());
    const double turn_vx = get_parameter("turn_vx").as_double();

    const float target_rad = get_turn_target_();

    double wz_target = 0.0;
    if (std::fabs((double)target_rad) > 1e-6) {
      wz_target = (target_rad > 0.0f) ? +wz_turn : -wz_turn;
    }
    if (invert) wz_target = -wz_target;

    wz_target *= wz_scale_();
    const double vx_target = turn_vx * speed_scale_();

    const double vx_slew = std::max(0.01, get_parameter("vx_slew").as_double());
    const double wz_cmd = slew(prev_wz_, wz_target, wz_turn_slew, dt);
    const double vx_cmd = slew(prev_vx_, vx_target, vx_slew, dt);
    prev_wz_ = wz_cmd;
    prev_vx_ = vx_cmd;

    publish_raw_(true, vx_cmd, 0.0, wz_cmd, "ctrl_raw(turning)");
  }

  void drive_advance_(double dt) {
    rc_interfaces::msg::LaneStatus lane;
    rclcpp::Time last_rx;
    bool has_lane = false;
    {
      std::lock_guard<std::mutex> lk(mu_lane_);
      has_lane = has_lane_;
      if (has_lane) { lane = last_lane_; last_rx = last_lane_rx_; }
    }

    const bool require_fresh = get_parameter("advance_require_lane_fresh").as_bool();
    const bool stop_on_lost = get_parameter("advance_stop_on_lane_lost").as_bool();

    if (require_fresh) {
      if (!has_lane) { publish_stop_raw_("advance:no_lane(require_fresh)"); return; }
      const int timeout_ms = get_parameter("lane_timeout_ms").as_int();
      const double age_ms = (now() - last_rx).seconds() * 1000.0;
      if (age_ms > timeout_ms) { publish_stop_raw_("advance:lane_stale(require_fresh)"); return; }
      if (stop_on_lost && !lane.in_lane) { publish_stop_raw_("advance:lane_lost(stop_on_lost)"); return; }
    } else {
      if (stop_on_lost && has_lane && !lane.in_lane) { publish_stop_raw_("advance:lane_lost(stop_on_lost)"); return; }
    }

    const double vx_max = std::max(0.0, get_parameter("vx_max").as_double());
    const double vx_target = clamp(get_parameter("advance_vx").as_double() * speed_scale_(), 0.0, vx_max);
    const double wz_target = get_parameter("advance_wz").as_double() * wz_scale_();

    const double vx_slew = std::max(0.01, get_parameter("advance_vx_slew").as_double());
    const double wz_slew = std::max(0.01, get_parameter("advance_wz_slew").as_double());

    const double vx_cmd = slew(prev_vx_, vx_target, vx_slew, dt);
    const double wz_cmd = slew(prev_wz_, wz_target, wz_slew, dt);

    prev_vx_ = vx_cmd;
    prev_wz_ = wz_cmd;

    publish_raw_(true, vx_cmd, 0.0, wz_cmd, "ctrl_raw(advance_to_center)");
  }

  void drive_lane_(double dt) {
    rc_interfaces::msg::LaneStatus lane;
    rclcpp::Time last_rx;
    bool has_lane = false;
    {
      std::lock_guard<std::mutex> lk(mu_lane_);
      has_lane = has_lane_;
      if (has_lane) { lane = last_lane_; last_rx = last_lane_rx_; }
    }
    if (!has_lane) { publish_stop_raw_("no_lane_yet"); return; }

    const int timeout_ms = get_parameter("lane_timeout_ms").as_int();
    const double age_ms = (now() - last_rx).seconds() * 1000.0;
    if (age_ms > timeout_ms) { publish_stop_raw_("lane_stale"); return; }

    const bool stop_on_lost = get_parameter("stop_on_lane_lost").as_bool();
    if (stop_on_lost && !lane.in_lane) { publish_stop_raw_("lane_lost"); return; }

    const double min_q = clamp(get_parameter("min_quality").as_double(), 0.0, 1.0);
    const double q = clamp((double)lane.quality, 0.0, 1.0);

    // ================== NEW steering (angle + offset, robust) ==================
    // raw errors
    double e_y   = (double)lane.offset_norm; // lateral
    double e_psi = (double)lane.angle;       // heading rad (lane_node에서 채워줘야 함)

    // deadband (tiny jitter -> do nothing)
    const double deadband_off = std::max(0.0, get_parameter("deadband_offset").as_double());
    const double deadband_ang = std::max(0.0, get_parameter("deadband_angle").as_double());
    if (std::fabs(e_y)   < deadband_off) e_y = 0.0;
    if (std::fabs(e_psi) < deadband_ang) e_psi = 0.0;

    // clamp errors
    const double max_err = std::max(1e-6, get_parameter("max_error_norm").as_double());
    e_y = clamp(e_y, -max_err, +max_err);

    const double ang_max = std::max(1e-6, get_parameter("angle_max_rad").as_double());
    e_psi = clamp(e_psi, -ang_max, +ang_max);

    // low quality => soften steering (NOT amplify)
    double quality_gain = 1.0;
    if (q < min_q) {
      quality_gain = clamp(get_parameter("lowq_steer_gain").as_double(), 0.0, 1.0);
    }

    // LPF on errors
    const double a = clamp(get_parameter("err_lpf_alpha").as_double(), 0.01, 1.0);
    if (!have_lane_f_) {
      e_y_f_ = e_y;
      e_psi_f_ = e_psi;
      have_lane_f_ = true;
    } else {
      e_y_f_   = (1.0 - a) * e_y_f_   + a * e_y;
      e_psi_f_ = (1.0 - a) * e_psi_f_ + a * e_psi;
    }

    // Stanley-lite: wz = k_psi*e_psi + atan2(k_y*e_y, v+v0)
    const double k_psi = get_parameter("k_psi").as_double();
    const double k_y   = get_parameter("k_y").as_double();
    const double v0    = std::max(1e-3, get_parameter("v0").as_double());

    // speed proxy (previous cmd is OK, robust under camera delay)
    const double v_for_steer = std::max(0.0, std::fabs(prev_vx_));

    double wz_target = quality_gain * (k_psi * e_psi_f_ + std::atan2(k_y * e_y_f_, v_for_steer + v0));

    const bool invert = get_parameter("invert_steering").as_bool();
    if (invert) wz_target = -wz_target;

    const double wz_max = std::max(1e-6, get_parameter("wz_max").as_double());
    wz_target = clamp(wz_target * wz_scale_(), -wz_max, +wz_max);

    // ================== speed (keep your logic) ==================
    const double vx_max = std::max(0.0, get_parameter("vx_max").as_double());
    const double vx_min = std::max(0.0, get_parameter("vx_min").as_double());
    const double vx_turn = std::max(0.0, get_parameter("vx_when_turning").as_double());
    const double linear_speed = std::max(0.0, get_parameter("linear_speed").as_double());
    const bool enforce_min = get_parameter("enforce_vx_min").as_bool();

    double vx_target = clamp(linear_speed, 0.0, vx_max) * speed_scale_();
    vx_target *= q;

    if (enforce_min) vx_target = clamp(vx_target, vx_min, vx_max);
    else             vx_target = clamp(vx_target, 0.0, vx_max);

    const double turn_threshold = clamp(get_parameter("turn_threshold").as_double(), 0.0, 1.0);
    const double turn_factor = std::abs(wz_target) / wz_max;
    if (turn_factor > turn_threshold) {
      vx_target = std::min(vx_target, vx_turn * speed_scale_());
    }

    if (q < min_q) {
      const bool stop_lowq = get_parameter("stop_on_low_quality").as_bool();
      if (stop_lowq) { publish_stop_raw_("low_quality"); return; }
      const double lq_scale = clamp(get_parameter("low_quality_speed_scale").as_double(), 0.0, 1.0);
      vx_target *= lq_scale;
    }

    // ================== slew + publish ==================
    const double vx_slew = std::max(0.01, get_parameter("vx_slew").as_double());
    const double wz_slew = std::max(0.01, get_parameter("wz_slew").as_double());

    const double vx_cmd = slew(prev_vx_, vx_target, vx_slew, dt);
    const double wz_cmd = slew(prev_wz_, wz_target, wz_slew, dt);
    prev_vx_ = vx_cmd;
    prev_wz_ = wz_cmd;

    publish_raw_(true, vx_cmd, 0.0, wz_cmd, "ctrl_raw(lane)");
  }

  // ===================== TF helper =====================
  bool lookup_base_from_(const std::string &cam_frame, geometry_msgs::msg::TransformStamped &out_tf) {
    try {
      out_tf = tf_buffer_.lookupTransform(base_frame_, cam_frame, tf2::TimePointZero);
      return true;
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "TF lookup failed: %s <- %s : %s",
        base_frame_.c_str(), cam_frame.c_str(), e.what());
      return false;
    }
  }

  // ===================== ALIGN_TO_MARKER =====================
  void tick_align_to_marker_() {
    geometry_msgs::msg::PoseStamped pose_cam;
    rclcpp::Time rx;
    {
      std::lock_guard<std::mutex> lk(mu_marker_pose_);
      if (!has_marker_pose_) { publish_raw_(true, 0.0, 0.0, 0.0, "align:no_pose"); return; }
      pose_cam = last_marker_pose_cam_;
      rx = last_marker_pose_rx_;
    }

    const int timeout_ms = get_parameter("marker_pose_timeout_ms").as_int();
    const double age_ms = (now() - rx).seconds() * 1000.0;
    if (age_ms > timeout_ms) { publish_raw_(true, 0.0, 0.0, 0.0, "align:pose_stale"); return; }

    const std::string cam_frame = pose_cam.header.frame_id.empty() ? front_cam_frame_ : pose_cam.header.frame_id;

    geometry_msgs::msg::TransformStamped tf_base_from_cam;
    if (!lookup_base_from_(cam_frame, tf_base_from_cam)) {
      publish_raw_(true, 0.0, 0.0, 0.0, "align:no_tf");
      return;
    }

    geometry_msgs::msg::PoseStamped marker_in_base;
    try {
      tf2::doTransform(pose_cam, marker_in_base, tf_base_from_cam);
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "align doTransform failed: %s", e.what());
      publish_raw_(true, 0.0, 0.0, 0.0, "align:transform_fail");
      return;
    }

    const double desired_x = get_parameter("align_desired_x").as_double();
    const double desired_y = get_parameter("align_desired_y").as_double();

    const double ex = (marker_in_base.pose.position.x - desired_x);
    const double ey = (marker_in_base.pose.position.y - desired_y);

    const bool use_yaw = get_parameter("align_use_marker_yaw").as_bool();
    double eyaw = 0.0;
    if (use_yaw) eyaw = wrap_pi(yaw_from_quat(marker_in_base.pose.orientation));

    const double kx = get_parameter("align_k_x").as_double();
    const double ky = get_parameter("align_k_y").as_double();
    const double kyaw = get_parameter("align_k_yaw").as_double();

    const double vx_max = std::max(0.0, get_parameter("align_vx_max").as_double());
    const double vy_max = std::max(0.0, get_parameter("align_vy_max").as_double());
    const double wz_max = std::max(0.0, get_parameter("align_wz_max").as_double());

    double vx = clamp(-kx * ex, -vx_max, +vx_max);
    double vy = clamp(-ky * ey, -vy_max, +vy_max);

    double wz = 0.0;
    if (use_yaw) wz = clamp(-kyaw * eyaw, -wz_max, +wz_max);

    const double eps_x = std::max(0.0, get_parameter("align_eps_x").as_double());
    const double eps_y = std::max(0.0, get_parameter("align_eps_y").as_double());
    const double eps_yaw = std::max(0.0, get_parameter("align_eps_yaw").as_double());

    const bool ok_xy = (std::fabs(ex) <= eps_x) && (std::fabs(ey) <= eps_y);
    const bool ok_yaw = (!use_yaw) || (std::fabs(eyaw) <= eps_yaw);

    if (ok_xy && ok_yaw) {
      publish_raw_(true, 0.0, 0.0, 0.0, "align:done_hold");
      publish_align_done_once_("aligned");
      return;
    }

    publish_raw_(true, vx, vy, wz, "align:tracking");
  }

  // ===================== PARKING =====================
  void park_reset_for_activation_() {
    park_substate_ = ParkSubState::LINE_ALIGN;
    unload_sent_ = false;
    has_pline_ = false;
    has_slot_pose_ = false;
  }

  void park_tick_line_align_() {
    rc_interfaces::msg::ParkingLineStatus pl;
    bool have = false;
    {
      std::lock_guard<std::mutex> lk(mu_pline_);
      have = has_pline_;
      if (have) pl = last_pline_;
    }

    if (!have) { publish_raw_(true, 0.0, 0.0, 0.0, "parking:no_line"); return; }

    const bool require_valid = get_parameter("park_require_valid").as_bool();
    const double min_q = get_parameter("park_min_quality").as_double();

    const bool ok_valid = (!require_valid) || pl.valid;
    const bool ok_quality = (pl.quality >= min_q);
    if (!(ok_valid && ok_quality)) { publish_raw_(true, 0.0, 0.0, 0.0, "parking:line_bad"); return; }

    const double k_off = get_parameter("park_k_offset").as_double();
    const double k_ang = get_parameter("park_k_angle").as_double();
    const double max_wz = get_parameter("park_max_wz").as_double();

    double wz = -(k_off * pl.offset_norm + k_ang * pl.angle);
    wz = clamp(wz, -max_wz, +max_wz);

    double vx = get_parameter("park_vx_cmd").as_double();
    double vy = get_parameter("park_vy_cmd").as_double();

    const double off_eps = get_parameter("park_align_offset_eps").as_double();
    const double ang_eps = get_parameter("park_align_angle_eps").as_double();
    const bool hold = get_parameter("park_auto_hold_when_aligned").as_bool();

    const bool aligned = (std::fabs(pl.offset_norm) <= off_eps) && (std::fabs(pl.angle) <= ang_eps);

    if (hold && aligned) {
      publish_raw_(true, 0.0, 0.0, 0.0, "parking:line_aligned_hold");

      bool have_slot = false;
      {
        std::lock_guard<std::mutex> lk(mu_slot_);
        have_slot = has_slot_pose_;
      }
      if (have_slot) {
        park_substate_ = ParkSubState::SLOT_MARKER_ALIGN;
        RCLCPP_INFO(get_logger(), "PARK LINE_ALIGN done -> SLOT_MARKER_ALIGN");
      }
      return;
    }

    publish_raw_(true, vx, vy, wz, "parking:line_align");
  }

  void park_tick_slot_align_() {
    geometry_msgs::msg::PoseStamped slot_pose_cam;
    {
      std::lock_guard<std::mutex> lk(mu_slot_);
      if (!has_slot_pose_) { publish_raw_(true, 0.0, 0.0, 0.0, "parking:no_slot_pose"); return; }
      slot_pose_cam = last_slot_pose_cam_;
    }

    geometry_msgs::msg::TransformStamped tf_base_from_side;
    if (!lookup_base_from_(side_cam_frame_, tf_base_from_side)) {
      publish_raw_(true, 0.0, 0.0, 0.0, "parking:no_tf");
      return;
    }

    geometry_msgs::msg::PoseStamped marker_in_base;
    try {
      tf2::doTransform(slot_pose_cam, marker_in_base, tf_base_from_side);
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "doTransform failed: %s", e.what());
      publish_raw_(true, 0.0, 0.0, 0.0, "parking:transform_fail");
      return;
    }

    const double standoff = get_parameter("park_desired_standoff_x").as_double();
    const double ex = (marker_in_base.pose.position.x - standoff);
    const double ey = (marker_in_base.pose.position.y);

    const bool use_yaw = get_parameter("park_use_marker_yaw").as_bool();
    double eyaw = 0.0;
    if (use_yaw) eyaw = wrap_pi(yaw_from_quat(marker_in_base.pose.orientation));

    const double kx = get_parameter("park_k_x").as_double();
    const double ky = get_parameter("park_k_y").as_double();
    const double kyaw = get_parameter("park_k_yaw").as_double();

    const double vx_max = get_parameter("park_vx_max").as_double();
    const double vy_max = get_parameter("park_vy_max").as_double();
    const double wz_max2 = get_parameter("park_wz_max2").as_double();

    double vx = clamp(-kx * ex, -vx_max, +vx_max);
    double vy = clamp(-ky * ey, -vy_max, +vy_max);

    double wz = 0.0;
    if (use_yaw) wz = clamp(-kyaw * eyaw, -wz_max2, +wz_max2);

    const double eps_x = get_parameter("park_eps_x").as_double();
    const double eps_y = get_parameter("park_eps_y").as_double();
    const double eps_yaw = get_parameter("park_eps_yaw").as_double();

    const bool ok_xy = (std::fabs(ex) <= eps_x) && (std::fabs(ey) <= eps_y);
    const bool ok_yaw = (!use_yaw) || (std::fabs(eyaw) <= eps_yaw);

    if (ok_xy && ok_yaw) {
      publish_raw_(true, 0.0, 0.0, 0.0, "parking:slot_aligned");
      park_substate_ = ParkSubState::DONE;
      RCLCPP_INFO(get_logger(), "PARK SLOT_ALIGN done -> DONE (ex=%.3f ey=%.3f eyaw=%.3f)", ex, ey, eyaw);
      return;
    }

    publish_raw_(true, vx, vy, wz, "parking:slot_align");
  }

  void park_tick_done_() {
    publish_raw_(true, 0.0, 0.0, 0.0, "parking:done_hold");

    if (!unload_sent_) {
      std_msgs::msg::Bool msg;
      msg.data = true;
      pub_unload_->publish(msg);
      unload_sent_ = true;
      RCLCPP_INFO(get_logger(), "UNLOAD trigger published: %s", unload_topic_.c_str());
    }
  }

  void run_parking_fsm_() {
    switch (park_substate_) {
      case ParkSubState::LINE_ALIGN:        park_tick_line_align_(); break;
      case ParkSubState::SLOT_MARKER_ALIGN: park_tick_slot_align_(); break;
      case ParkSubState::DONE:             park_tick_done_(); break;
    }
  }

  // ===================== main loop =====================
  void on_timer_() {
    auto now_st = std::chrono::steady_clock::now();
    const double dt = std::max(1e-3, std::chrono::duration<double>(now_st - last_loop_t_).count());
    last_loop_t_ = now_st;

    // emergency active => suppress AUTO output + reset slew memory
    if (emergency_active_()) {
      prev_vx_ = 0.0;
      prev_wz_ = 0.0;
      publish_stop_raw_("suppressed_by_emergency");
      align_done_sent_ = false;
      return;
    }

    if (!enable_drive_.load()) {
      publish_stop_raw_("enable_drive=false");
      align_done_sent_ = false;
      return;
    }

    const std::string st = get_mission_state_();

    // ALIGN edge
    if (st == "ALIGN_TO_MARKER") {
      if (!align_active_) {
        align_active_ = true;
        align_done_sent_ = false;
        RCLCPP_INFO(get_logger(), "ALIGN activated");
      }
    } else {
      if (align_active_) {
        align_active_ = false;
        align_done_sent_ = false;
        RCLCPP_INFO(get_logger(), "ALIGN deactivated");
      }
    }

    // PARK activation edge
    const bool parking_active = (st == parking_active_state_name_);
    if (parking_active && !parking_active_) {
      park_reset_for_activation_();
      RCLCPP_INFO(get_logger(), "PARK Activated -> substate=LINE_ALIGN");
    }
    if (!parking_active && parking_active_) {
      const bool pub_once = get_parameter("parking_publish_disable_once_on_deactivate").as_bool();
      if (pub_once) publish_raw_(false, 0.0, 0.0, 0.0, "parking:deactivated_once");
      RCLCPP_INFO(get_logger(), "PARK Deactivated");
    }
    parking_active_ = parking_active;

    // routing
    if (align_active_) {
      tick_align_to_marker_();
    } else if (parking_active_) {
      run_parking_fsm_();
    } else if (st == "TURNING") {
      drive_turning_(dt);
    } else if (st == "ADVANCE_TO_CENTER") {
      drive_advance_(dt);
    } else if (st == "STOP_AT_MARKER" || st == "STOP_BUMP" || st == "IDLE") {
      publish_stop_raw_("mission_state=" + st);
    } else {
      drive_lane_(dt);
    }

    // NOTE: no /control/drive_cmd_safe here. safety_manager_node will handle it.
  }

private:
  // topics/frames
  std::string lane_topic_;
  std::string parking_line_topic_;
  std::string slot_marker_pose_topic_;
  std::string marker_pose_topic_;

  std::string enable_drive_topic_;
  std::string mission_state_topic_;
  std::string turn_target_topic_;
  std::string align_done_topic_;

  bool publish_raw_cmd_{true};
  std::string out_raw_topic_;
  std::string imu_topic_{"/imu/data"};
  std::string unload_topic_;

  std::string base_frame_;
  std::string front_cam_frame_;
  std::string side_cam_frame_;
  std::string parking_active_state_name_{"PARK"};

  // emergency
  std::string emergency_topic_;
  rclcpp::Subscription<rc_interfaces::msg::DriveCmd>::SharedPtr sub_emg_;
  std::mutex mu_emg_;
  rclcpp::Time last_emg_rx_{0, 0, RCL_ROS_TIME};
  bool has_emg_{false};

  // pubs/subs
  rclcpp::Publisher<rc_interfaces::msg::DriveCmd>::SharedPtr pub_raw_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_unload_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_align_done_;

  rclcpp::Subscription<rc_interfaces::msg::LaneStatus>::SharedPtr sub_lane_;
  rclcpp::Subscription<rc_interfaces::msg::ParkingLineStatus>::SharedPtr sub_pline_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_slot_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_marker_pose_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_state_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_turn_target_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // states
  std::atomic<bool> enable_drive_{false};

  std::mutex mu_lane_;
  rc_interfaces::msg::LaneStatus last_lane_;
  rclcpp::Time last_lane_rx_{0, 0, RCL_ROS_TIME};
  bool has_lane_{false};

  std::mutex mu_pline_;
  rc_interfaces::msg::ParkingLineStatus last_pline_;
  bool has_pline_{false};

  std::mutex mu_slot_;
  geometry_msgs::msg::PoseStamped last_slot_pose_cam_;
  rclcpp::Time last_slot_rx_{0, 0, RCL_ROS_TIME};
  bool has_slot_pose_{false};

  std::mutex mu_marker_pose_;
  geometry_msgs::msg::PoseStamped last_marker_pose_cam_;
  rclcpp::Time last_marker_pose_rx_{0, 0, RCL_ROS_TIME};
  bool has_marker_pose_{false};

  std::mutex mu_state_;
  std::string mission_state_{"IDLE"};
  float turn_target_rad_{0.0f};

  std::mutex mu_imu_;
  double gyro_z_{0.0};
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  bool has_imu_{false};

  // raw cmd
  rc_interfaces::msg::DriveCmd last_raw_;
  rclcpp::Time last_raw_rx_{0, 0, RCL_ROS_TIME};
  bool has_raw_{false};

  // slew memory
  double prev_vx_{0.0};
  double prev_wz_{0.0};
  std::chrono::steady_clock::time_point last_loop_t_;

  // filtered errors for robustness (latency/noise tolerant)
  bool have_lane_f_{false};
  double e_y_f_{0.0};
  double e_psi_f_{0.0};

  // parking fsm
  bool parking_active_{false};
  ParkSubState park_substate_{ParkSubState::LINE_ALIGN};
  bool unload_sent_{false};

  // align handshake
  bool align_active_{false};
  bool align_done_sent_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlStackNode>());
  rclcpp::shutdown();
  return 0;
}
