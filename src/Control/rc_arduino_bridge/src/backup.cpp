#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <optional>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rc_interfaces/msg/drive_cmd.hpp"

using namespace std::chrono_literals;

namespace {

double clamp(double v, double lo, double hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

speed_t to_speed_t(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return B115200;
  }
}

class SerialPort {
public:
  SerialPort() = default;
  ~SerialPort() { close_port(); }

  bool open_port(const std::string& dev, int baud) {
    close_port();

    fd_ = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      last_err_ = "open() failed: " + std::string(std::strerror(errno));
      return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      last_err_ = "tcgetattr() failed: " + std::string(std::strerror(errno));
      close_port();
      return false;
    }

    cfmakeraw(&tty);

    // 8N1
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;        // no HW flow control
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // no SW flow control

    // Non-blocking read with small timeout (not essential here)
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1; // 0.1s

    speed_t spd = to_speed_t(baud);
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      last_err_ = "tcsetattr() failed: " + std::string(std::strerror(errno));
      close_port();
      return false;
    }

    // flush
    tcflush(fd_, TCIOFLUSH);

    dev_ = dev;
    baud_ = baud;
    return true;
  }

  void close_port() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    dev_.clear();
    baud_ = 0;
  }

  bool is_open() const { return fd_ >= 0; }

  const std::string& last_error() const { return last_err_; }

  bool write_all(const std::string& s) {
    if (fd_ < 0) {
      last_err_ = "serial not open";
      return false;
    }
    const char* p = s.data();
    size_t n = s.size();

    while (n > 0) {
      ssize_t w = ::write(fd_, p, n);
      if (w < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          // let it breathe; this is rare for tiny lines
          ::usleep(1000);
          continue;
        }
        last_err_ = "write() failed: " + std::string(std::strerror(errno));
        return false;
      }
      p += w;
      n -= static_cast<size_t>(w);
    }
    return true;
  }

private:
  int fd_{-1};
  std::string dev_;
  int baud_{0};
  std::string last_err_;
};

} // namespace

class ArduinoBridgeNode : public rclcpp::Node {
public:
  ArduinoBridgeNode() : Node("arduino_bridge_node") {
    // ---- parameters
    declare_parameter<std::string>("port", "/dev/ttyACM0");
    declare_parameter<int>("baud", 115200);
    declare_parameter<double>("tx_hz", 50.0);
    declare_parameter<int>("watchdog_ms", 200);
    declare_parameter<double>("max_vx", 0.4); // m/s
    declare_parameter<double>("max_vy", 0.4); // m/s
    declare_parameter<double>("max_wz", 1.2); // rad/s
    declare_parameter<std::string>("input", "cmd_vel"); // "cmd_vel" or "drive_cmd"
    declare_parameter<bool>("debug_log", false);

    port_ = get_parameter("port").as_string();
    baud_ = get_parameter("baud").as_int();
    tx_hz_ = get_parameter("tx_hz").as_double();
    watchdog_ms_ = get_parameter("watchdog_ms").as_int();
    max_vx_ = get_parameter("max_vx").as_double();
    max_vy_ = get_parameter("max_vy").as_double();
    max_wz_ = get_parameter("max_wz").as_double();
    input_ = get_parameter("input").as_string();
    debug_log_ = get_parameter("debug_log").as_bool();

    // ---- open serial
    if (!serial_.open_port(port_, baud_)) {
      RCLCPP_ERROR(get_logger(), "Serial open failed (%s): %s",
                   port_.c_str(), serial_.last_error().c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Serial opened: %s @ %d", port_.c_str(), baud_);
    }

    // ---- subscriptions
    if (input_ == "drive_cmd") {
      sub_drive_ = create_subscription<rc_interfaces::msg::DriveCmd>(
        "/drive/cmd", 10,
        [this](rc_interfaces::msg::DriveCmd::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(mu_);
          last_rx_ = now();
          has_cmd_ = msg->enable;
          // Here we assume msg->vx/vy/wz are already in m/s, m/s, rad/s.
          // We'll normalize by max_* params.
          vx_ = msg->vx;
          vy_ = msg->vy;
          wz_ = msg->wz;
        }
      );
      RCLCPP_INFO(get_logger(), "Input: /drive/cmd (rc_interfaces/DriveCmd)");
    } else {
      sub_twist_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        [this](geometry_msgs::msg::Twist::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(mu_);
          last_rx_ = now();
          has_cmd_ = true;
          vx_ = msg->linear.x;
          vy_ = msg->linear.y;
          wz_ = msg->angular.z;
        }
      );
      RCLCPP_INFO(get_logger(), "Input: /cmd_vel (geometry_msgs/Twist)");
    }

    // ---- tx timer
    if (tx_hz_ < 1.0) tx_hz_ = 1.0;
    auto period = std::chrono::duration<double>(1.0 / tx_hz_);
    timer_ = create_wall_timer(period, std::bind(&ArduinoBridgeNode::on_timer, this));

    last_rx_ = now();
    last_sent_stop_ = false;

    RCLCPP_INFO(get_logger(),
      "arduino_bridge ready (tx_hz=%.1f, watchdog_ms=%d, max_vx=%.2f, max_vy=%.2f, max_wz=%.2f)",
      tx_hz_, watchdog_ms_, max_vx_, max_vy_, max_wz_);
  }

private:
  void on_timer() {
    // attempt reopen if needed
    if (!serial_.is_open()) {
      static int cnt = 0;
      if ((cnt++ % 50) == 0) { // ~1s at 50Hz
        RCLCPP_WARN(get_logger(), "Serial not open, retrying: %s", port_.c_str());
      }
      serial_.open_port(port_, baud_);
      return;
    }

    // snapshot command
    float vx, vy, wz;
    bool enable;
    rclcpp::Time last_rx;
    {
      std::lock_guard<std::mutex> lk(mu_);
      vx = vx_;
      vy = vy_;
      wz = wz_;
      enable = has_cmd_;
      last_rx = last_rx_;
    }

    const auto t = now();
    const double age_ms = (t - last_rx).seconds() * 1000.0;

    // watchdog: no cmd => STOP
    if (!enable || age_ms > watchdog_ms_) {
      if (!last_sent_stop_) {
        // send stop once, Arduino watchdog will keep it safe anyway
        if (!serial_.write_all("Z\n")) {
          RCLCPP_ERROR(get_logger(), "UART write failed: %s", serial_.last_error().c_str());
          serial_.close_port();
          return;
        }
        last_sent_stop_ = true;
        if (debug_log_) {
          RCLCPP_WARN(get_logger(), "TX: Z (watchdog %.0fms)", age_ms);
        }
      }
      return;
    }

    // normalize to -1..+1 for Arduino firmware
    const double vx_n = clamp(vx / max_vx_, -1.0, 1.0);
    const double vy_n = clamp(vy / max_vy_, -1.0, 1.0);
    const double wz_n = clamp(wz / max_wz_, -1.0, 1.0);

    char buf[96];
    std::snprintf(buf, sizeof(buf), "D %.3f %.3f %.3f\n", vx_n, vy_n, wz_n);
    std::string line(buf);

    if (!serial_.write_all(line)) {
      RCLCPP_ERROR(get_logger(), "UART write failed: %s", serial_.last_error().c_str());
      serial_.close_port();
      return;
    }

    last_sent_stop_ = false;
    if (debug_log_) {
      RCLCPP_INFO(get_logger(), "TX: %s", line.c_str());
    }
  }

private:
  // params
  std::string port_;
  int baud_{115200};
  double tx_hz_{50.0};
  int watchdog_ms_{200};
  double max_vx_{0.4}, max_vy_{0.4}, max_wz_{1.2};
  std::string input_{"cmd_vel"};
  bool debug_log_{false};

  // state
  std::mutex mu_;
  float vx_{0.0f}, vy_{0.0f}, wz_{0.0f};
  bool has_cmd_{false};
  rclcpp::Time last_rx_{0,0,RCL_ROS_TIME};
  bool last_sent_stop_{false};

  // ROS
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Subscription<rc_interfaces::msg::DriveCmd>::SharedPtr sub_drive_;

  // serial
  SerialPort serial_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArduinoBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
