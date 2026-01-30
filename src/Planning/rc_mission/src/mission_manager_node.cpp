#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "rc_interfaces/msg/task_cmd.hpp"
#include "rc_interfaces/msg/task_status.hpp"
#include "rc_interfaces/msg/marker_status.hpp"

#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

// ---------------- helpers ----------------
static inline std::string upper(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c){ return (unsigned char)std::toupper(c); });
  return s;
}

static inline double wrap_pi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static inline bool contains(const std::vector<int32_t>& v, int32_t x) {
  return std::find(v.begin(), v.end(), x) != v.end();
}

struct XY { double x{0.0}; double y{0.0}; };

static inline double bearing_xy(const XY& a, const XY& b) {
  return std::atan2(b.y - a.y, b.x - a.x);
}
static inline double dist_xy(const XY& a, const XY& b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  return std::sqrt(dx*dx + dy*dy);
}

static inline std::string action_from_turn(double turn, double straight_th, double uturn_th) {
  const double abs_t = std::fabs(turn);
  if (abs_t < straight_th) return "STRAIGHT";
  if (abs_t > uturn_th) return "UTURN";
  return (turn > 0.0) ? "LEFT" : "RIGHT";
}

static inline double quantize_turn_rad(double turn_rad) {
  const double a = std::fabs(turn_rad);
  if (a < 0.35) return 0.0;
  if (a < 1.80) return (turn_rad > 0.0 ? +M_PI_2 : -M_PI_2);
  return (turn_rad > 0.0 ? +M_PI : -M_PI);
}

// ================= MissionManager (with Planner + ALIGN) =================
class MissionManagerNode : public rclcpp::Node {
public:
  MissionManagerNode() : Node("mission_manager_node") {
    // ===== Topics =====
    declare_parameter<std::string>("task_cmd_topic", "/server/task_cmd");
    declare_parameter<std::string>("task_status_topic", "/server/task_status");
    declare_parameter<std::string>("mission_state_topic", "/mission/state");
    declare_parameter<std::string>("marker_topic", "/perception/marker");

    // ===== Planner outputs (debug/for other nodes) =====
    declare_parameter<std::string>("turn_rad_topic", "/planning/turn_rad");
    declare_parameter<std::string>("turn_action_topic", "/planning/turn_action");
    declare_parameter<std::string>("next_distance_topic", "/planning/next_distance");
    declare_parameter<std::string>("next_marker_id_topic", "/planning/next_marker_id");

    // ===== Control gating =====
    declare_parameter<std::string>("enable_drive_topic", "/control/enable_drive");
    declare_parameter<std::string>("turn_target_topic", "/mission/turn_target_rad");

    // ===== Rates =====
    declare_parameter<double>("publish_rate_hz", 10.0);

    // ===== Marker gate =====
    declare_parameter<double>("min_marker_quality", 0.3);
    declare_parameter<double>("marker_timeout_sec", 1.0);
    declare_parameter<bool>("require_marker_in_route", true);
    declare_parameter<double>("marker_debounce_sec", 0.25);

    // ===== Stop/Turn behavior =====
    declare_parameter<double>("hold_sec", 0.2);
    declare_parameter<double>("wz_turn", 0.8);
    declare_parameter<double>("min_turn_rad", 0.35);

    // ===== ADVANCE_TO_CENTER =====
    declare_parameter<bool>("use_advance_to_center", true);
    declare_parameter<double>("center_offset_m", 0.16);
    declare_parameter<double>("advance_vx", 0.12);
    declare_parameter<double>("advance_min_sec", 0.2);
    declare_parameter<double>("advance_max_sec", 3.0);
    declare_parameter<bool>("advance_then_stop", true);

    // ===== Task behavior =====
    declare_parameter<bool>("accept_route_restart", true);

    // ===== IMU-based TURN 종료 =====
    declare_parameter<bool>("use_imu_turn", true);
    declare_parameter<std::string>("imu_topic", "/imu/data");
    declare_parameter<int>("imu_timeout_ms", 200);
    declare_parameter<bool>("turn_fallback_to_time", true);
    declare_parameter<double>("yaw_tol_rad", 0.06);
    declare_parameter<double>("settle_gyro_z", 0.12);
    declare_parameter<double>("max_turn_sec", 4.0);

    // ===== Planner params (marker map) =====
    declare_parameter<std::string>("marker_map_yaml", "");
    declare_parameter<int>("nav_id_min", 0);
    declare_parameter<int>("nav_id_max", 15);
    declare_parameter<double>("straight_threshold_rad", 0.35);
    declare_parameter<double>("uturn_threshold_rad", 2.60);

    // ===== ALIGN_TO_MARKER (NEW) =====
    declare_parameter<bool>("use_align_to_marker", true);
    declare_parameter<double>("align_timeout_sec", 1.8);
    declare_parameter<std::string>("align_done_topic", "/mission/align_done");

    // ===== 안정화 패치: STOP_BUMP before TURN (NEW) =====
    declare_parameter<bool>("use_stop_bump_before_turn", true);
    declare_parameter<double>("stop_bump_sec", 0.12); // align done 후 관성/슬루 안정화

    // ===== Load params =====
    task_cmd_topic_ = get_parameter("task_cmd_topic").as_string();
    task_status_topic_ = get_parameter("task_status_topic").as_string();
    mission_state_topic_ = get_parameter("mission_state_topic").as_string();
    marker_topic_ = get_parameter("marker_topic").as_string();

    turn_rad_topic_ = get_parameter("turn_rad_topic").as_string();
    turn_action_topic_ = get_parameter("turn_action_topic").as_string();
    next_distance_topic_ = get_parameter("next_distance_topic").as_string();
    next_marker_id_topic_ = get_parameter("next_marker_id_topic").as_string();

    enable_drive_topic_ = get_parameter("enable_drive_topic").as_string();
    turn_target_topic_ = get_parameter("turn_target_topic").as_string();

    use_imu_turn_ = get_parameter("use_imu_turn").as_bool();
    imu_topic_ = get_parameter("imu_topic").as_string();
    imu_timeout_ms_ = get_parameter("imu_timeout_ms").as_int();
    turn_fallback_to_time_ = get_parameter("turn_fallback_to_time").as_bool();
    yaw_tol_rad_ = get_parameter("yaw_tol_rad").as_double();
    settle_gyro_z_ = get_parameter("settle_gyro_z").as_double();
    max_turn_sec_ = get_parameter("max_turn_sec").as_double();

    use_align_to_marker_ = get_parameter("use_align_to_marker").as_bool();
    align_timeout_sec_   = get_parameter("align_timeout_sec").as_double();
    align_done_topic_    = get_parameter("align_done_topic").as_string();

    use_stop_bump_before_turn_ = get_parameter("use_stop_bump_before_turn").as_bool();
    stop_bump_sec_             = get_parameter("stop_bump_sec").as_double();

    load_marker_map_();

    // ===== Pubs =====
    pub_state_  = create_publisher<std_msgs::msg::String>(mission_state_topic_, 10);
    pub_status_ = create_publisher<rc_interfaces::msg::TaskStatus>(task_status_topic_, 10);
    pub_enable_ = create_publisher<std_msgs::msg::Bool>(enable_drive_topic_, 10);
    pub_turn_target_ = create_publisher<std_msgs::msg::Float32>(turn_target_topic_, 10);

    // planner debug pubs
    pub_plan_action_ = create_publisher<std_msgs::msg::String>(turn_action_topic_, 10);
    pub_plan_turn_   = create_publisher<std_msgs::msg::Float32>(turn_rad_topic_, 10);
    pub_plan_dist_   = create_publisher<std_msgs::msg::Float32>(next_distance_topic_, 10);
    pub_plan_nextid_ = create_publisher<std_msgs::msg::Int32>(next_marker_id_topic_, 10);

    // ===== Subs =====
    sub_task_ = create_subscription<rc_interfaces::msg::TaskCmd>(
      task_cmd_topic_, 10,
      std::bind(&MissionManagerNode::on_task_cmd_, this, std::placeholders::_1));

    sub_marker_ = create_subscription<rc_interfaces::msg::MarkerStatus>(
      marker_topic_, 10,
      std::bind(&MissionManagerNode::on_marker_, this, std::placeholders::_1));

    // align_done: ControlStack -> MissionManager
    sub_align_done_ = create_subscription<std_msgs::msg::Bool>(
      align_done_topic_, 10,
      [this](std_msgs::msg::Bool::SharedPtr msg){
        align_done_.store(msg->data);
      });

    if (use_imu_turn_) {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::SharedPtr msg){
          last_gyro_z_ = static_cast<double>(msg->angular_velocity.z);
          last_imu_time_ = now();
          has_imu_ = true;

          if (state_ == "TURNING") {
            const auto t = now();
            if (last_imu_integ_time_.nanoseconds() == 0) {
              last_imu_integ_time_ = t;
              return;
            }
            const double dt = (t - last_imu_integ_time_).seconds();
            last_imu_integ_time_ = t;
            if (dt <= 0.0 || dt > 0.2) return;
            yaw_accum_rad_ += last_gyro_z_ * dt;
          }
        });
    }

    const double hz = std::max(1.0, get_parameter("publish_rate_hz").as_double());
    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / hz)),
      std::bind(&MissionManagerNode::on_timer_, this));

    set_state_("IDLE", "boot");
    publish_enable_(false, "boot");
    publish_turn_target_(0.0f);

    RCLCPP_INFO(get_logger(),
      "mission_manager ready. task_cmd=%s marker=%s align_done=%s imu_turn=%s imu_topic=%s map=%zu",
      task_cmd_topic_.c_str(), marker_topic_.c_str(), align_done_topic_.c_str(),
      use_imu_turn_ ? "true" : "false", imu_topic_.c_str(),
      marker_map_.size());
  }

private:
  // ================= Planner(Map) =================
  void load_marker_map_() {
    const std::string path = get_parameter("marker_map_yaml").as_string();
    if (path.empty()) {
      RCLCPP_WARN(get_logger(), "marker_map_yaml empty. planner will publish END/unknown turns.");
      return;
    }

    try {
      YAML::Node root = YAML::LoadFile(path);
      auto markers = root["markers"];
      if (!markers || !markers.IsSequence()) {
        RCLCPP_ERROR(get_logger(), "Invalid YAML: 'markers' sequence not found: %s", path.c_str());
        return;
      }

      marker_map_.clear();
      for (const auto& m : markers) {
        if (!m["id"] || !m["x"] || !m["y"]) continue;
        const int id = m["id"].as<int>();
        marker_map_[id] = XY{m["x"].as<double>(), m["y"].as<double>()};
      }

      RCLCPP_INFO(get_logger(), "Loaded marker_map: %s (count=%zu)", path.c_str(), marker_map_.size());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to load marker_map YAML: %s err=%s", path.c_str(), e.what());
    }
  }

  bool get_xy_(int id, XY& out) const {
    auto it = marker_map_.find(id);
    if (it == marker_map_.end()) return false;
    out = it->second;
    return true;
  }

  bool is_nav_id_(int id) const {
    const int nav_min = get_parameter("nav_id_min").as_int();
    const int nav_max = get_parameter("nav_id_max").as_int();
    return (id >= nav_min && id <= nav_max);
  }

  // route_index_는 "다음에 도달해야 하는 marker index"
  // current_marker_id_는 "최근 관측된 marker"
  void maybe_align_route_to_current_() {
    if (!task_active_) return;
    if (current_marker_id_ < 0) return;
    if (!contains(route_ids_, current_marker_id_)) return;

    auto it = std::find(route_ids_.begin(), route_ids_.end(), current_marker_id_);
    const int cur_idx = static_cast<int>(std::distance(route_ids_.begin(), it));
    const int new_next_idx = std::min(cur_idx + 1, (int)route_ids_.size());

    if (new_next_idx > route_index_) {
      route_index_ = new_next_idx;
      RCLCPP_INFO(get_logger(), "Aligned route_index_ to %d based on current_marker=%d", route_index_, current_marker_id_);
    }
  }

  void publish_plan_end_() {
    std_msgs::msg::Int32 n; n.data = -1;
    pub_plan_nextid_->publish(n);

    std_msgs::msg::String a; a.data = "END";
    std_msgs::msg::Float32 t; t.data = 0.0f;
    std_msgs::msg::Float32 d; d.data = 0.0f;
    pub_plan_action_->publish(a);
    pub_plan_turn_->publish(t);
    pub_plan_dist_->publish(d);
  }

  void publish_plan_() {
    if (!task_active_ || route_ids_.empty()) { publish_plan_end_(); return; }
    if (route_index_ < 0 || route_index_ >= (int)route_ids_.size()) { publish_plan_end_(); return; }

    const int next = (int)route_ids_[route_index_];

    if (current_marker_id_ < 0) {
      std_msgs::msg::Int32 n; n.data = next;
      pub_plan_nextid_->publish(n);
      return;
    }

    if (!contains(route_ids_, current_marker_id_)) {
      std_msgs::msg::Int32 n; n.data = next;
      pub_plan_nextid_->publish(n);
      return;
    }

    auto itc = std::find(route_ids_.begin(), route_ids_.end(), current_marker_id_);
    const int cur_idx = static_cast<int>(std::distance(route_ids_.begin(), itc));
    const int cur = current_marker_id_;

    if (next == cur) {
      std_msgs::msg::Int32 n; n.data = next;
      pub_plan_nextid_->publish(n);

      std_msgs::msg::String a; a.data = "STRAIGHT";
      std_msgs::msg::Float32 t; t.data = 0.0f;
      std_msgs::msg::Float32 d; d.data = 0.0f;
      pub_plan_action_->publish(a);
      pub_plan_turn_->publish(t);
      pub_plan_dist_->publish(d);
      return;
    }

    const int prev = (cur_idx - 1 >= 0) ? (int)route_ids_[cur_idx - 1] : cur;

    XY p_prev, p_cur, p_next;
    if (!get_xy_(prev, p_prev) || !get_xy_(cur, p_cur) || !get_xy_(next, p_next)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Map missing for prev=%d cur=%d next=%d. planner publish next only.", prev, cur, next);
      std_msgs::msg::Int32 n; n.data = next;
      pub_plan_nextid_->publish(n);
      return;
    }

    double heading = bearing_xy(p_prev, p_cur);
    if (prev == cur) heading = bearing_xy(p_cur, p_next);
    const double bearing_next = bearing_xy(p_cur, p_next);
    const double turn = wrap_pi(bearing_next - heading);
    const double dist = dist_xy(p_cur, p_next);

    const double straight_th = get_parameter("straight_threshold_rad").as_double();
    const double uturn_th = get_parameter("uturn_threshold_rad").as_double();
    const std::string action = action_from_turn(turn, straight_th, uturn_th);

    std_msgs::msg::Int32 n; n.data = next;
    pub_plan_nextid_->publish(n);

    std_msgs::msg::String a; a.data = action;
    std_msgs::msg::Float32 t; t.data = (float)turn;
    std_msgs::msg::Float32 d; d.data = (float)dist;
    pub_plan_action_->publish(a);
    pub_plan_turn_->publish(t);
    pub_plan_dist_->publish(d);

    // latch
    last_turn_rad_ = (float)turn;
    last_turn_action_ = action;
    last_turn_rad_stamp_ = now();
    last_turn_action_stamp_ = now();
  }

  // ================= Mission callbacks =================
  void on_task_cmd_(const rc_interfaces::msg::TaskCmd::SharedPtr msg) {
    const std::string incoming_task_id = msg->task_id;
    const auto incoming_route = msg->route_ids;

    if (incoming_route.empty()) {
      RCLCPP_WARN(get_logger(), "TaskCmd received but route_ids empty. Ignore.");
      return;
    }

    const bool accept_restart = get_parameter("accept_route_restart").as_bool();
    if (task_active_ && !accept_restart) {
      RCLCPP_WARN(get_logger(), "TaskCmd received but task already active and accept_route_restart=false. Ignore.");
      return;
    }

    route_ids_.clear();
    route_ids_.reserve(incoming_route.size());
    for (auto id32 : incoming_route) {
      const int id = (int)id32;
      if (is_nav_id_(id)) route_ids_.push_back((int32_t)id);
    }

    if (route_ids_.empty()) {
      RCLCPP_WARN(get_logger(), "TaskCmd set but nav-filtered route empty. task_id=%s", incoming_task_id.c_str());
      task_active_ = false;
      set_state_("IDLE", "route_empty");
      publish_enable_(false, "route_empty");
      publish_plan_end_();
      return;
    }

    task_active_ = true;
    task_id_ = incoming_task_id;
    plate_target_ = msg->plate_target;
    goal_ = msg->goal;

    route_index_ = 0;

    // phase timers clear
    advance_until_ = rclcpp::Time(0,0,RCL_ROS_TIME);
    stop_until_    = rclcpp::Time(0,0,RCL_ROS_TIME);
    align_until_   = rclcpp::Time(0,0,RCL_ROS_TIME);
    bump_until_    = rclcpp::Time(0,0,RCL_ROS_TIME);
    turn_until_    = rclcpp::Time(0,0,RCL_ROS_TIME);

    // flags reset
    align_done_.store(false);

    // imu turn reset
    yaw_accum_rad_ = 0.0;
    target_turn_rad_ = 0.0;
    last_imu_integ_time_ = rclcpp::Time(0,0,RCL_ROS_TIME);
    turn_start_time_ = rclcpp::Time(0,0,RCL_ROS_TIME);

    maybe_align_route_to_current_();

    set_state_("DRIVE", "task_cmd");
    publish_enable_(true, "task_cmd");
    publish_turn_target_(0.0f);

    publish_plan_();
    publish_status_("task_applied");

    RCLCPP_INFO(get_logger(),
      "New task applied. task_id=%s route_len=%zu next=%d current=%d route_index=%d",
      task_id_.c_str(), route_ids_.size(),
      (route_index_ < (int)route_ids_.size() ? (int)route_ids_[route_index_] : -1),
      (int)current_marker_id_,
      route_index_);
  }

  void on_marker_(const rc_interfaces::msg::MarkerStatus::SharedPtr msg) {
    const double min_q = get_parameter("min_marker_quality").as_double();
    if (!msg->valid) return;
    if (msg->quality < min_q) return;

    const int id = (int)msg->id;
    if (!is_nav_id_(id)) return;

    // debounce (steady clock)
    const double debounce = std::max(0.0, get_parameter("marker_debounce_sec").as_double());
    const auto t = std::chrono::steady_clock::now();
    if (id == last_seen_marker_id_) {
      const double dt = std::chrono::duration<double>(t - last_seen_tp_).count();
      if (dt < debounce) return;
    }
    last_seen_marker_id_ = id;
    last_seen_tp_ = t;

    current_marker_id_ = (int32_t)id;
    last_marker_stamp_ = msg->header.stamp;

    if (!task_active_) { publish_plan_(); return; }

    const bool require_in_route = get_parameter("require_marker_in_route").as_bool();
    if (require_in_route && !contains(route_ids_, (int32_t)id)) {
      publish_plan_();
      return;
    }

    maybe_align_route_to_current_();

    if (route_index_ < (int)route_ids_.size()) {
      const int32_t next_id = route_ids_[route_index_];
      if ((int32_t)id == next_id) {
        route_index_++;

        if (route_index_ >= (int)route_ids_.size()) {
          set_state_("PARK", "route_done");
          publish_enable_(false, "park");
          publish_turn_target_(0.0f);
          publish_plan_();
          publish_status_("marker_done_last");
          return;
        }

        const bool use_adv = get_parameter("use_advance_to_center").as_bool();
        if (use_adv) begin_advance_to_center_("marker_reached");
        else begin_stop_at_marker_("marker_reached(no_adv)");

        publish_plan_();
        publish_status_("marker_advance");
        return;
      }
    }

    publish_plan_();
  }

  // ================= phases =================
  void begin_advance_to_center_(const std::string& reason) {
    const double offset_m = std::max(0.0, get_parameter("center_offset_m").as_double());
    const double vx = std::max(0.05, get_parameter("advance_vx").as_double());

    double sec = (vx > 1e-6) ? (offset_m / vx) : 0.0;
    const double min_sec = std::max(0.0, get_parameter("advance_min_sec").as_double());
    const double max_sec = std::max(min_sec, get_parameter("advance_max_sec").as_double());
    sec = std::max(min_sec, std::min(max_sec, sec));

    advance_until_ = now() + rclcpp::Duration::from_seconds(sec);

    set_state_("ADVANCE_TO_CENTER", reason);
    publish_enable_(true, "advance_to_center");
    publish_turn_target_(0.0f);
  }

  void begin_stop_at_marker_(const std::string& reason) {
    const double hold = std::max(0.0, get_parameter("hold_sec").as_double());
    stop_until_ = now() + rclcpp::Duration::from_seconds(hold);

    set_state_("STOP_AT_MARKER", reason);
    publish_enable_(false, "stop_at_marker");
    publish_turn_target_(0.0f);

    // 안전: 상태 진입 시 align_done 항상 리셋
    align_done_.store(false);
  }

  void begin_align_to_marker_(const std::string& reason) {
    if (!use_align_to_marker_) {
      begin_stop_bump_before_turn_("align_disabled->bump");
      return;
    }

    align_done_.store(false);
    const double sec = std::max(0.2, align_timeout_sec_);
    align_until_ = now() + rclcpp::Duration::from_seconds(sec);

    set_state_("ALIGN_TO_MARKER", reason);
    publish_enable_(true, "align_to_marker");
    publish_turn_target_(0.0f);
  }

  void begin_stop_bump_before_turn_(const std::string& reason) {
    if (!use_stop_bump_before_turn_) {
      begin_turning_("bump_disabled->turn");
      return;
    }
    const double sec = std::max(0.05, stop_bump_sec_);
    bump_until_ = now() + rclcpp::Duration::from_seconds(sec);

    set_state_("STOP_BUMP", reason);
    publish_enable_(false, "stop_bump");
    publish_turn_target_(0.0f);
  }

  void begin_turning_(const std::string& reason) {
    // 최신 plan 반영
    publish_plan_();

    double tr = wrap_pi((double)last_turn_rad_);
    const double tr_q = quantize_turn_rad(tr);

    const double min_turn = std::max(0.0, get_parameter("min_turn_rad").as_double());
    const double wz_turn  = std::max(0.1, get_parameter("wz_turn").as_double());

    if (std::fabs(tr_q) < min_turn) {
      publish_turn_target_(0.0f);
      set_state_("DRIVE", "turn_skip(straight)");
      publish_enable_(true, "drive");
      return;
    }

    // TURN 초기화
    yaw_accum_rad_ = 0.0;
    target_turn_rad_ = tr_q;
    turn_start_time_ = now();
    last_imu_integ_time_ = rclcpp::Time(0,0,RCL_ROS_TIME);

    // fallback timer
    const double turn_sec = std::fabs(tr_q) / wz_turn;
    turn_until_ = now() + rclcpp::Duration::from_seconds(turn_sec);

    current_turn_target_rad_ = (float)tr_q;

    // 안정화: TURNING state 먼저 올리고, target은 다음 tick에서 1번 더 내보내도록 유지
    set_state_("TURNING", reason);
    publish_enable_(true, "turning");
    publish_turn_target_(current_turn_target_rad_);

    RCLCPP_INFO(get_logger(), "TURNING start: target=%.3frad (%.1fdeg), fallback_sec=%.2f (%s)",
                tr_q, tr_q * 180.0 / M_PI, turn_sec, reason.c_str());
  }

  void finish_turn_(const std::string& why) {
    publish_turn_target_(0.0f);
    set_state_("DRIVE", why);
    publish_enable_(true, "drive_after_turn");
    align_done_.store(false);
  }

  // ================= timer =================
  void on_timer_() {
    // marker stale monitor
    double age = 1e9;
    if (last_marker_stamp_.nanoseconds() > 0) {
      age = (now() - last_marker_stamp_).seconds();
    }
    const double timeout = get_parameter("marker_timeout_sec").as_double();
    marker_stale_ = (age > timeout);

    const auto t = now();

    if (state_ == "ADVANCE_TO_CENTER") {
      if (advance_until_.nanoseconds() > 0 && t >= advance_until_) {
        const bool adv_then_stop = get_parameter("advance_then_stop").as_bool();
        if (adv_then_stop) begin_stop_at_marker_("advance_done");
        else begin_align_to_marker_("advance_done(no_stop)");
      } else {
        publish_enable_(true, "advance_hold");
      }
    }
    else if (state_ == "STOP_AT_MARKER") {
      if (t >= stop_until_) {
        begin_align_to_marker_("stop_done");
      }
    }
    else if (state_ == "ALIGN_TO_MARKER") {
      if (align_done_.load()) {
        begin_stop_bump_before_turn_("align_done");
      } else if (align_until_.nanoseconds() > 0 && t >= align_until_) {
        RCLCPP_WARN(get_logger(), "ALIGN timeout -> STOP_BUMP");
        begin_stop_bump_before_turn_("align_timeout");
      } else {
        publish_enable_(true, "align_hold");
        publish_turn_target_(0.0f);
      }
    }
    else if (state_ == "STOP_BUMP") {
      if (t >= bump_until_) {
        begin_turning_("bump_done");
      } else {
        publish_enable_(false, "bump_hold");
        publish_turn_target_(0.0f);
      }
    }
    else if (state_ == "TURNING") {
      // ===== IMU 기반 종료 =====
      if (use_imu_turn_) {
        bool imu_ok = true;
        double imu_age_ms = 1e9;
        if (!has_imu_) imu_ok = false;
        else {
          imu_age_ms = (now() - last_imu_time_).seconds() * 1000.0;
          if (imu_age_ms > (double)imu_timeout_ms_) imu_ok = false;
        }

        const double max_turn_sec = std::max(0.5, max_turn_sec_);
        const double elapsed = (turn_start_time_.nanoseconds() > 0) ? (now() - turn_start_time_).seconds() : 0.0;

        if (elapsed > max_turn_sec) {
          RCLCPP_WARN(get_logger(), "TURN timeout (%.2fs > %.2fs). forcing finish.", elapsed, max_turn_sec);
          finish_turn_("turn_timeout");
        } else if (imu_ok) {
          const double target_abs = std::fabs(target_turn_rad_);
          const double yaw_abs = std::fabs(yaw_accum_rad_);
          const double tol = std::max(0.01, yaw_tol_rad_);
          const double settle = std::max(0.01, settle_gyro_z_);

          if (yaw_abs >= (target_abs - tol) && std::fabs(last_gyro_z_) < settle) {
            RCLCPP_INFO(get_logger(),
              "TURN done by IMU: yaw=%.3f/%.3f(rad), gyro_z=%.3f age=%.0fms",
              yaw_accum_rad_, target_turn_rad_, last_gyro_z_, imu_age_ms);
            finish_turn_("turn_done_imu");
          }
        } else {
          if (!turn_fallback_to_time_) {
            RCLCPP_WARN(get_logger(), "IMU timeout during TURN. forcing finish (no fallback).");
            finish_turn_("turn_done_no_imu");
          }
        }
      }

      // ===== 시간 기반 종료 (fallback) =====
      if (state_ == "TURNING") {
        if (t >= turn_until_) {
          finish_turn_("turn_done_time");
        }
      }
    }
    else if (state_ == "PARK") {
      publish_enable_(false, "park_hold");
    }
    else if (state_ == "IDLE") {
      publish_enable_(false, "idle_hold");
    }
    else if (state_ == "DRIVE") {
      if (task_active_) publish_enable_(true, "drive_hold");
      else publish_enable_(false, "no_task");
    }

    publish_plan_();
    publish_state_();
    publish_status_("tick");
  }

  // ================= publish helpers =================
  void set_state_(const std::string& s, const std::string& reason) {
    state_ = upper(s);
    RCLCPP_INFO(get_logger(), "state=%s (%s)", state_.c_str(), reason.c_str());
    publish_state_();
  }

  void publish_state_() {
    std_msgs::msg::String out;
    out.data = state_;
    pub_state_->publish(out);
  }

  void publish_enable_(bool en, const std::string& why) {
    std_msgs::msg::Bool b;
    b.data = en;
    pub_enable_->publish(b);
    (void)why;
  }

  void publish_turn_target_(float rad) {
    std_msgs::msg::Float32 m;
    m.data = rad;
    pub_turn_target_->publish(m);
  }

  void publish_status_(const std::string& detail) {
    rc_interfaces::msg::TaskStatus st;
    st.header.stamp = now();
    st.task_id = task_id_;
    st.state = state_;
    st.current_marker_id = current_marker_id_;
    st.next_marker_id = get_next_marker_id_();
    st.progress_0_1 = calc_progress_();
    st.detail = detail;
    pub_status_->publish(st);
  }

  int32_t get_next_marker_id_() const {
    if (!task_active_) return -1;
    if (route_index_ < 0 || route_index_ >= (int)route_ids_.size()) return -1;
    return route_ids_[route_index_];
  }

  float calc_progress_() const {
    if (!task_active_) return 0.0f;
    if (route_ids_.empty()) return 0.0f;
    const double p = (double)route_index_ / (double)route_ids_.size();
    return (float)std::max(0.0, std::min(1.0, p));
  }

private:
  // topics
  std::string task_cmd_topic_;
  std::string task_status_topic_;
  std::string mission_state_topic_;
  std::string marker_topic_;

  // planner debug topics
  std::string turn_rad_topic_;
  std::string turn_action_topic_;
  std::string next_distance_topic_;
  std::string next_marker_id_topic_;

  // control topics
  std::string enable_drive_topic_;
  std::string turn_target_topic_;

  // pubs/subs
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
  rclcpp::Publisher<rc_interfaces::msg::TaskStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_enable_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_turn_target_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  pub_plan_action_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_plan_turn_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_plan_dist_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr   pub_plan_nextid_;

  rclcpp::Subscription<rc_interfaces::msg::TaskCmd>::SharedPtr sub_task_;
  rclcpp::Subscription<rc_interfaces::msg::MarkerStatus>::SharedPtr sub_marker_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_align_done_;

  rclcpp::TimerBase::SharedPtr timer_;

  // mission data
  bool task_active_{false};
  std::string task_id_{""};
  std::string plate_target_{""};
  std::string goal_{""};

  // route
  std::vector<int32_t> route_ids_;
  int route_index_{0};

  // state
  std::string state_{"IDLE"};

  // marker context
  int32_t current_marker_id_{-1};
  rclcpp::Time last_marker_stamp_{0, 0, RCL_ROS_TIME};
  bool marker_stale_{true};

  // debounce
  int last_seen_marker_id_{-1};
  std::chrono::steady_clock::time_point last_seen_tp_{};

  // phase timing
  rclcpp::Time advance_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time stop_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time align_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time bump_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time turn_until_{0, 0, RCL_ROS_TIME};
  float current_turn_target_rad_{0.0f};

  // planning latch
  float last_turn_rad_{0.0f};
  rclcpp::Time last_turn_rad_stamp_{0, 0, RCL_ROS_TIME};
  std::string last_turn_action_{""};
  rclcpp::Time last_turn_action_stamp_{0, 0, RCL_ROS_TIME};

  // ===== IMU turn =====
  bool use_imu_turn_{true};
  std::string imu_topic_{"/imu/data"};
  int imu_timeout_ms_{200};
  bool turn_fallback_to_time_{true};
  double yaw_tol_rad_{0.06};
  double settle_gyro_z_{0.12};
  double max_turn_sec_{4.0};

  bool has_imu_{false};
  rclcpp::Time last_imu_time_{0,0,RCL_ROS_TIME};
  double last_gyro_z_{0.0};

  double yaw_accum_rad_{0.0};
  double target_turn_rad_{0.0};
  rclcpp::Time last_imu_integ_time_{0,0,RCL_ROS_TIME};
  rclcpp::Time turn_start_time_{0,0,RCL_ROS_TIME};

  // ===== ALIGN =====
  bool use_align_to_marker_{true};
  double align_timeout_sec_{1.8};
  std::string align_done_topic_{"/mission/align_done"};
  std::atomic<bool> align_done_{false};

  // ===== STOP_BUMP =====
  bool use_stop_bump_before_turn_{true};
  double stop_bump_sec_{0.12};

  // marker map
  std::unordered_map<int, XY> marker_map_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionManagerNode>());
  rclcpp::shutdown();
  return 0;
}
