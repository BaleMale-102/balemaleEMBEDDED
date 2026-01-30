#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

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

  bool open_port(const std::string& dev, int baud, bool disable_dtr_rts) {
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
    tty.c_cflag &= ~CRTSCTS;               // no HW flow control
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);// no SW flow control

    // non-blocking read with short timeout
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

    // ✅ IMPORTANT:
    // 기존 tcflush(TCIOFLUSH)가 Arduino가 부팅 직후 1회 뿌리는 "READY"를 날려먹는 케이스가 많음.
    // 따라서 OS 레벨 입력 flush는 하지 않음. (우리 rx_buf_는 close/open 시 이미 비움)
    // tcflush(fd_, TCIOFLUSH);  // ❌ 제거

    // ★ DTR/RTS 내려서 UNO reset 방지 (드라이버에 따라 효과 있음)
    if (disable_dtr_rts) {
      int status = 0;
      if (ioctl(fd_, TIOCMGET, &status) == 0) {
        status &= ~TIOCM_DTR;
        status &= ~TIOCM_RTS;
        (void)ioctl(fd_, TIOCMSET, &status);
      }
    }

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
    rx_buf_.clear();
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

  // non-blocking line read (\n terminated). returns true if a line extracted.
  bool read_line(std::string &out_line) {
    out_line.clear();
    if (fd_ < 0) return false;

    char buf[256];
    while (true) {
      ssize_t r = ::read(fd_, buf, sizeof(buf));
      if (r < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) break;
        last_err_ = "read() failed: " + std::string(std::strerror(errno));
        return false;
      }
      if (r == 0) break;
      rx_buf_.append(buf, buf + r);
      if (rx_buf_.size() > 4096) rx_buf_.erase(0, rx_buf_.size() - 4096);
    }

    auto pos = rx_buf_.find('\n');
    if (pos == std::string::npos) return false;

    std::string line = rx_buf_.substr(0, pos);
    rx_buf_.erase(0, pos + 1);

    // trim CR
    if (!line.empty() && line.back() == '\r') line.pop_back();
    out_line = line;
    return true;
  }

private:
  int fd_{-1};
  std::string dev_;
  int baud_{0};
  std::string last_err_;
  std::string rx_buf_;
};

} // namespace

class ArduinoBridgeNode : public rclcpp::Node {
public:
  ArduinoBridgeNode() : Node("arduino_bridge_node") {
    declare_parameter<std::string>("drive_cmd_topic", "/control/drive_cmd_safe");
    
    declare_parameter<std::string>("port", "/dev/ttyUSB0");
    declare_parameter<int>("baud", 115200);
    declare_parameter<double>("tx_hz", 50.0);
    declare_parameter<int>("watchdog_ms", 200);

    declare_parameter<double>("max_vx", 0.4);
    declare_parameter<double>("max_vy", 0.4);
    declare_parameter<double>("max_wz", 1.2);

    declare_parameter<std::string>("input", "drive_cmd"); // cmd_vel or drive_cmd
    declare_parameter<bool>("debug_log", true);

    // Arduino protocol
    declare_parameter<int>("arduino_max_pwm", 1900);
    declare_parameter<int>("handshake_timeout_ms", 4000);
    declare_parameter<bool>("disable_dtr_rts", true);

    drive_cmd_topic_ = get_parameter("drive_cmd_topic").as_string();

    port_ = get_parameter("port").as_string();
    baud_ = get_parameter("baud").as_int();
    tx_hz_ = get_parameter("tx_hz").as_double();
    watchdog_ms_ = get_parameter("watchdog_ms").as_int();

    max_vx_ = get_parameter("max_vx").as_double();
    max_vy_ = get_parameter("max_vy").as_double();
    max_wz_ = get_parameter("max_wz").as_double();

    input_ = get_parameter("input").as_string();
    debug_log_ = get_parameter("debug_log").as_bool();

    arduino_max_pwm_ = get_parameter("arduino_max_pwm").as_int();
    handshake_timeout_ms_ = get_parameter("handshake_timeout_ms").as_int();
    disable_dtr_rts_ = get_parameter("disable_dtr_rts").as_bool();

    open_serial_();

    if (input_ == "drive_cmd") {
      sub_drive_ = create_subscription<rc_interfaces::msg::DriveCmd>(
        drive_cmd_topic_, 10,
        [this](rc_interfaces::msg::DriveCmd::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(mu_);
          last_rx_ = now();

          has_cmd_ = msg->enable;
          vx_ = msg->vx;
          vy_ = msg->vy;
          wz_ = msg->wz;

          if (debug_log_) {
            RCLCPP_INFO(
              get_logger(),
              "DriveCmd RX (source=%s enable=%d) vx=%.3f vy=%.3f wz=%.3f",
              msg->source.c_str(),
              msg->enable ? 1 : 0,
              msg->vx, msg->vy, msg->wz
            );
          }
        }
      );
      RCLCPP_INFO(get_logger(), "Input: %s (rc_interfaces/DriveCmd)", drive_cmd_topic_.c_str());
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

    if (tx_hz_ < 1.0) tx_hz_ = 1.0;
    auto period = std::chrono::duration<double>(1.0 / tx_hz_);
    timer_ = create_wall_timer(period, std::bind(&ArduinoBridgeNode::on_timer, this));

    last_rx_ = now();
    last_sent_stop_ = false;

    RCLCPP_INFO(get_logger(),
      "arduino_bridge ready (port=%s baud=%d tx_hz=%.1f watchdog_ms=%d)",
      port_.c_str(), baud_, tx_hz_, watchdog_ms_);
  }

private:
  void open_serial_() {
    if (!serial_.open_port(port_, baud_, disable_dtr_rts_)) {
      RCLCPP_ERROR(get_logger(), "Serial open failed (%s): %s",
                   port_.c_str(), serial_.last_error().c_str());
      state_ = State::DISCONNECTED;
      return;
    }
    RCLCPP_INFO(get_logger(), "Serial opened: %s @ %d", port_.c_str(), baud_);

    // handshake: wait READY then send P
    state_ = State::HANDSHAKE;
    ready_ = false;
    sent_pwm_ = false;
    handshake_start_ms_ = now().nanoseconds() / 1000000LL;
  }

  void handle_rx_() {
    std::string line;
    while (serial_.read_line(line)) {
      if (debug_log_) {
        RCLCPP_INFO(get_logger(), "UART RX: %s", line.c_str());
      }

      // ✅ 1) 진짜 READY면 당연히 ready
      if (line.find("READY") != std::string::npos) {
        ready_ = true;
      }

      // ✅ 2) HANDSHAKE 중에는 "ACK ..." / "RX: ..." 같은 어떤 정상 응답도
      //    "보드가 살아있다"로 판단해서 ready 처리
      if (state_ == State::HANDSHAKE) {
        if (line.rfind("ACK", 0) == 0 || line.rfind("RX:", 0) == 0) {
          ready_ = true;
        }
      }

      // P ACK 체크
      if (line.rfind("ACK P", 0) == 0) {
        sent_pwm_ = true;
      }
    }
  }

  void on_timer() {
    // try reopen if needed
    if (!serial_.is_open()) {
      static int cnt = 0;
      if ((cnt++ % 50) == 0) {
        RCLCPP_WARN(get_logger(), "Serial not open, retrying: %s", port_.c_str());
      }
      open_serial_();
      return;
    }

    // always pump RX (ACK/READY)
    handle_rx_();

    const int64_t now_ms = now().nanoseconds() / 1000000LL;

    // handshake state
    if (state_ == State::HANDSHAKE) {
      if (!ready_) {
        if ((now_ms - handshake_start_ms_) > handshake_timeout_ms_) {
          RCLCPP_WARN(get_logger(), "Handshake timeout (no READY). Reopen port.");
          serial_.close_port();
          return;
        }
        // don't send drive while not ready, but keep safe stop
        (void)serial_.write_all("Z\n");
        return;
      }

      // READY received -> send PWM once
      if (!sent_pwm_) {
        char b[32];
        std::snprintf(b, sizeof(b), "P %d\n", arduino_max_pwm_);
        if (!serial_.write_all(std::string(b))) {
          RCLCPP_ERROR(get_logger(), "UART write failed: %s", serial_.last_error().c_str());
          serial_.close_port();
          return;
        }
        if (debug_log_) RCLCPP_INFO(get_logger(), "UART TX: %s", b);
        // wait ACK P (sent_pwm_=true)
        return;
      }

      // all good
      state_ = State::RUN;
      RCLCPP_INFO(get_logger(), "Handshake done. Start RUN.");
    }

    // RUN: snapshot command
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
        if (!serial_.write_all("Z\n")) {
          RCLCPP_ERROR(get_logger(), "UART write failed: %s", serial_.last_error().c_str());
          serial_.close_port();
          return;
        }
        last_sent_stop_ = true;
        if (debug_log_) {
          RCLCPP_WARN(get_logger(), "UART TX: Z (watchdog %.0fms)", age_ms);
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

    if (!serial_.write_all(std::string(buf))) {
      RCLCPP_ERROR(get_logger(), "UART write failed: %s", serial_.last_error().c_str());
      serial_.close_port();
      return;
    }

    last_sent_stop_ = false;
    if (debug_log_) {
      RCLCPP_INFO(get_logger(), "UART TX: %s", buf);
    }
  }

private:
  enum class State { DISCONNECTED, HANDSHAKE, RUN };

  // params
  std::string drive_cmd_topic_{"/control/drive_cmd_safe"};
  std::string port_;
  int baud_{115200};
  double tx_hz_{50.0};
  int watchdog_ms_{200};
  double max_vx_{0.4}, max_vy_{0.4}, max_wz_{1.2};
  std::string input_{"cmd_vel"};
  bool debug_log_{true};

  int arduino_max_pwm_{1900};
  int handshake_timeout_ms_{4000};
  bool disable_dtr_rts_{true};

  // state
  std::mutex mu_;
  float vx_{0.0f}, vy_{0.0f}, wz_{0.0f};
  bool has_cmd_{false};
  rclcpp::Time last_rx_{0,0,RCL_ROS_TIME};
  bool last_sent_stop_{false};

  // handshake
  State state_{State::DISCONNECTED};
  bool ready_{false};
  bool sent_pwm_{false};
  int64_t handshake_start_ms_{0};

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
