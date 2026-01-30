#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <algorithm>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rc_interfaces/msg/drive_cmd.hpp"

using namespace std::chrono_literals;

static inline double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}
static inline std::string upper(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c){ return (unsigned char)std::toupper(c); });
  return s;
}

class KeyboardTeleopNode : public rclcpp::Node {
public:
  KeyboardTeleopNode() : Node("keyboard_teleop_node") {
    // ===== Params =====
    declare_parameter<std::string>("mission_state_topic", "/mission/state");           // ✅ NEW
    declare_parameter<std::string>("enable_drive_topic", "/control/enable_drive");    // ✅ NEW

    // emergency override cmd (goes to safety_manager mux)
    declare_parameter<std::string>("emergency_drive_cmd_topic", "/control/drive_cmd_emergency");

    declare_parameter<bool>("publish_emergency", true);
    declare_parameter<double>("publish_rate_hz", 30.0);

    declare_parameter<double>("vx_cmd", 0.15);
    declare_parameter<double>("vy_cmd", 0.15);
    declare_parameter<double>("wz_cmd", 0.30);

    declare_parameter<int>("key_active_ms", 140);

    // deadman
    declare_parameter<bool>("require_deadman", true);
    declare_parameter<bool>("deadman_disable_enable", true);

    // on state change: whether to also set enable_drive automatically
    declare_parameter<bool>("auto_enable_on_drive_state", true);
    declare_parameter<bool>("auto_disable_on_idle_or_park", true);

    mission_state_topic_ = get_parameter("mission_state_topic").as_string();
    enable_drive_topic_  = get_parameter("enable_drive_topic").as_string();
    emergency_topic_     = get_parameter("emergency_drive_cmd_topic").as_string();
    publish_emergency_   = get_parameter("publish_emergency").as_bool();

    pub_state_  = create_publisher<std_msgs::msg::String>(mission_state_topic_, 10);
    pub_enable_ = create_publisher<std_msgs::msg::Bool>(enable_drive_topic_, 10);
    pub_emg_    = create_publisher<rc_interfaces::msg::DriveCmd>(emergency_topic_, 10);

    // also subscribe mission/state if something else publishes it (optional safety)
    sub_state_ = create_subscription<std_msgs::msg::String>(
      mission_state_topic_, 10,
      [this](std_msgs::msg::String::SharedPtr msg){
        // keep local state in sync
        current_state_ = upper(msg->data);
      });

    setup_terminal();
    print_help();

    const double hz = std::max(1.0, get_parameter("publish_rate_hz").as_double());
    timer_ = create_wall_timer(
      std::chrono::milliseconds((int)(1000.0 / hz)),
      std::bind(&KeyboardTeleopNode::on_timer, this));

    auto t0 = std::chrono::steady_clock::now();
    last_w_tp_ = last_a_tp_ = last_s_tp_ = last_d_tp_ = t0 - 1s;
    last_j_tp_ = last_l_tp_ = t0 - 1s;

    // start safe
    drive_enable_local_ = false;
    current_state_ = "IDLE";
    publish_enable_(false);
    publish_state_("IDLE");

    RCLCPP_INFO(get_logger(),
      "KeyboardTeleop ready. state_topic=%s enable_topic=%s emg_topic=%s publish_emergency=%s",
      mission_state_topic_.c_str(), enable_drive_topic_.c_str(), emergency_topic_.c_str(),
      publish_emergency_ ? "true" : "false");
  }

  ~KeyboardTeleopNode() override {
    restore_terminal();
  }

private:
  void print_help() {
    RCLCPP_INFO(get_logger(),
      "\n=== Keyboard Teleop (MissionManager v2 + EMERGENCY) ===\n"
      "[Mission STATE publish -> /mission/state]\n"
      "  1: IDLE   2: DRIVE   3: PARK\n"
      "  t: TOGGLE (IDLE<->DRIVE)\n"
      "  p: PARK_TOGGLE (DRIVE<->PARK, IDLE->PARK)\n"
      "  space: HARD STOP (enable_drive=false + emergency Z)\n"
      "  q: quit\n"
      "\n[Enable gate -> /control/enable_drive]\n"
      "  E: enable_drive=true\n"
      "  C: enable_drive=false\n"
      "\n[Emergency DriveCmd -> /control/drive_cmd_emergency]\n"
      "  w/s : vx +/- (hold)\n"
      "  a/d : wz +/- (hold)\n"
      "  j/l : vy +/- (hold)\n"
      "  x   : zero(vx,vy,wz=0) (enable 유지)\n"
    );
  }

  void setup_terminal() {
    tcgetattr(STDIN_FILENO, &orig_term_);
    termios raw = orig_term_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    terminal_ready_ = true;
  }

  void restore_terminal() {
    if (terminal_ready_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &orig_term_);
      terminal_ready_ = false;
    }
  }

  int read_key() {
    unsigned char c = 0;
    const int n = ::read(STDIN_FILENO, &c, 1);
    if (n == 1) return (int)c;
    return -1;
  }

  void publish_state_(const std::string& s) {
    std_msgs::msg::String m;
    m.data = upper(s);
    pub_state_->publish(m);
  }

  void publish_enable_(bool en) {
    std_msgs::msg::Bool b;
    b.data = en;
    pub_enable_->publish(b);
  }

  void publish_emg_(bool enable, double vx, double vy, double wz, const std::string& src) {
    if (!publish_emergency_) return;

    rc_interfaces::msg::DriveCmd cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link";
    cmd.enable = enable;
    cmd.vx = (float)vx;
    cmd.vy = (float)vy;
    cmd.wz = (float)wz;
    cmd.source = src;
    pub_emg_->publish(cmd);
  }

  void hard_stop_(const char* reason) {
    vx_out_ = vy_out_ = wz_out_ = 0.0;

    auto t = std::chrono::steady_clock::now();
    last_w_tp_ = last_a_tp_ = last_s_tp_ = last_d_tp_ = t - 1s;
    last_j_tp_ = last_l_tp_ = t - 1s;

    // ✅ gate OFF
    drive_enable_local_ = false;
    publish_enable_(false);

    // ✅ emergency Z (enable=false)
    publish_emg_(false, 0.0, 0.0, 0.0, std::string("keyboard_hard_stop:") + reason);

    RCLCPP_WARN(get_logger(), "HARD STOP: %s", reason);
  }

  void set_state_local_(const std::string& s, const char* why) {
    current_state_ = upper(s);
    publish_state_(current_state_);

    const bool auto_en = get_parameter("auto_enable_on_drive_state").as_bool();
    const bool auto_dis = get_parameter("auto_disable_on_idle_or_park").as_bool();

    if (auto_en && current_state_ == "DRIVE") {
      drive_enable_local_ = true;
      publish_enable_(true);
    }
    if (auto_dis && (current_state_ == "IDLE" || current_state_ == "PARK")) {
      drive_enable_local_ = false;
      publish_enable_(false);
      // park/idle 들어갈 때 emergency도 Z로 한번 박아두면 안전
      publish_emg_(false, 0.0, 0.0, 0.0, std::string("keyboard_state_z:") + why);
    }

    RCLCPP_INFO(get_logger(), "state=%s (%s)", current_state_.c_str(), why);
  }

  void toggle_drive_() {
    if (current_state_ == "DRIVE") set_state_local_("IDLE", "toggle->idle");
    else set_state_local_("DRIVE", "toggle->drive");
  }

  void park_toggle_() {
    if (current_state_ == "PARK") set_state_local_("DRIVE", "park_toggle->drive");
    else set_state_local_("PARK", "park_toggle->park");
  }

  void handle_key_(int k) {
    if (k == 'q' || k == 'Q') {
      RCLCPP_INFO(get_logger(), "quit.");
      rclcpp::shutdown();
      return;
    }

    // ===== Mission state keys (NEW) =====
    if (k == '1') { set_state_local_("IDLE", "key1"); return; }
    if (k == '2') { set_state_local_("DRIVE", "key2"); return; }
    if (k == '3') { set_state_local_("PARK", "key3"); return; }
    if (k == 't' || k == 'T') { toggle_drive_(); return; }
    if (k == 'p' || k == 'P') { park_toggle_(); return; }

    if (k == ' ') {
      // strongest stop
      hard_stop_("space");
      return;
    }

    // ===== Enable gate keys (NEW) =====
    if (k == 'e' || k == 'E') {
      drive_enable_local_ = true;
      publish_enable_(true);
      RCLCPP_INFO(get_logger(), "enable_drive=true");
      return;
    }
    if (k == 'c' || k == 'C') {
      drive_enable_local_ = false;
      publish_enable_(false);
      // also force emergency Z
      publish_emg_(false, 0.0, 0.0, 0.0, "keyboard_enable_false_z");
      RCLCPP_INFO(get_logger(), "enable_drive=false (Z)");
      return;
    }

    // ===== Emergency drive keys =====
    if (!publish_emergency_) return;

    if (k == 'x' || k == 'X') {
      vx_out_ = vy_out_ = wz_out_ = 0.0;
      publish_emg_(drive_enable_local_, 0.0, 0.0, 0.0, "keyboard_zero");
      return;
    }

    auto t = std::chrono::steady_clock::now();
    if (k == 'w' || k == 'W') { last_w_tp_ = t; return; }
    if (k == 's' || k == 'S') { last_s_tp_ = t; return; }
    if (k == 'a' || k == 'A') { last_a_tp_ = t; return; }
    if (k == 'd' || k == 'D') { last_d_tp_ = t; return; }
    if (k == 'j' || k == 'J') { last_j_tp_ = t; return; }
    if (k == 'l' || k == 'L') { last_l_tp_ = t; return; }
  }

  static bool is_active_(const std::chrono::steady_clock::time_point& tp, int active_ms) {
    const auto now_tp = std::chrono::steady_clock::now();
    const auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_tp - tp).count();
    return dt_ms <= active_ms;
  }

  void on_timer() {
    // drain keys
    while (true) {
      const int k = read_key();
      if (k < 0) break;
      handle_key_(k);
    }

    if (!publish_emergency_) return;

    const int active_ms = get_parameter("key_active_ms").as_int();
    const bool require_deadman = get_parameter("require_deadman").as_bool();
    const bool deadman_disable_enable = get_parameter("deadman_disable_enable").as_bool();

    const bool w = is_active_(last_w_tp_, active_ms);
    const bool s = is_active_(last_s_tp_, active_ms);
    const bool a = is_active_(last_a_tp_, active_ms);
    const bool d = is_active_(last_d_tp_, active_ms);
    const bool j = is_active_(last_j_tp_, active_ms);
    const bool l = is_active_(last_l_tp_, active_ms);

    const double vx_cmd = clamp(get_parameter("vx_cmd").as_double(), 0.0, 1.0);
    const double vy_cmd = clamp(get_parameter("vy_cmd").as_double(), 0.0, 1.0);
    const double wz_cmd = clamp(get_parameter("wz_cmd").as_double(), 0.0, 1.0);

    double vx = 0.0, vy = 0.0, wz = 0.0;

    if (w && !s) vx = +vx_cmd;
    else if (s && !w) vx = -vx_cmd;

    if (j && !l) vy = +vy_cmd;
    else if (l && !j) vy = -vy_cmd;

    if (a && !d) wz = +wz_cmd;
    else if (d && !a) wz = -wz_cmd;

    const bool any_motion = (w || s || a || d || j || l);

    // ✅ deadman => no motion keys -> Z (enable=false) is best for your Arduino protocol
    if (require_deadman && !any_motion) {
      if (deadman_disable_enable) {
        publish_emg_(false, 0.0, 0.0, 0.0, "keyboard_deadman_z");
        vx_out_ = vy_out_ = wz_out_ = 0.0;
        return;
      } else {
        publish_emg_(drive_enable_local_, 0.0, 0.0, 0.0, "keyboard_deadman_zero");
        vx_out_ = vy_out_ = wz_out_ = 0.0;
        return;
      }
    }

    vx_out_ = vx; vy_out_ = vy; wz_out_ = wz;
    publish_emg_(drive_enable_local_, vx_out_, vy_out_, wz_out_, "keyboard_hold_emg");
  }

private:
  // topics
  std::string mission_state_topic_;
  std::string enable_drive_topic_;
  std::string emergency_topic_;
  bool publish_emergency_{true};

  // pubs/subs
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   pub_enable_;
  rclcpp::Publisher<rc_interfaces::msg::DriveCmd>::SharedPtr pub_emg_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_state_;

  rclcpp::TimerBase::SharedPtr timer_;

  // terminal
  termios orig_term_{};
  bool terminal_ready_{false};

  // local state
  std::string current_state_{"IDLE"};
  bool drive_enable_local_{false};

  // outputs
  double vx_out_{0.0}, vy_out_{0.0}, wz_out_{0.0};

  // key timing
  std::chrono::steady_clock::time_point last_w_tp_, last_s_tp_, last_a_tp_, last_d_tp_;
  std::chrono::steady_clock::time_point last_j_tp_, last_l_tp_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardTeleopNode>());
  rclcpp::shutdown();
  return 0;
}
