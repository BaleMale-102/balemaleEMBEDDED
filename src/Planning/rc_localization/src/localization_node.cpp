/**
 * localization_node.cpp
 * 
 * MarkerStatus를 구독하여 marker_map에서 map 좌표를 조회하고
 * 로봇의 map 기준 위치를 발행
 * 
 * 입력:
 *   - /perception/marker_status (MarkerStatus) - marker_pose_node에서 발행
 *   - /imu/data (Imu) - optional, yaw fusion용
 * 
 * 출력:
 *   - /localization/pose (PoseStamped) - map 기준 로봇 위치
 *   - /localization/fix_valid (Bool) - 유효한 fix 여부
 *   - TF: map -> base_link
 */

#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rc_interfaces/msg/marker_status.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include <yaml-cpp/yaml.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

struct MarkerMapEntry {
  double x{0.0};      // map 좌표 (단위: 파라미터로 조정)
  double y{0.0};
  double yaw{0.0};    // 마커가 향하는 방향 (rad)
};

// ============ Utility Functions ============

static geometry_msgs::msg::Quaternion yaw_to_quat(double yaw_rad) {
  geometry_msgs::msg::Quaternion q;
  const double half = 0.5 * yaw_rad;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half);
  q.w = std::cos(half);
  return q;
}

static inline double wrap_pi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static inline double quat_to_yaw(const geometry_msgs::msg::Quaternion& qmsg) {
  tf2::Quaternion q(qmsg.x, qmsg.y, qmsg.z, qmsg.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return wrap_pi(yaw);
}


class LocalizationNode : public rclcpp::Node {
public:
  LocalizationNode() : Node("localization_node") {
    // ============ Parameters ============
    declare_parameter<std::string>("marker_status_topic", "/perception/marker_status");
    declare_parameter<std::string>("pose_topic", "/localization/pose");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "base_link");

    declare_parameter<double>("min_quality", 0.25);
    declare_parameter<double>("marker_timeout_sec", 1.0);
    declare_parameter<bool>("hold_last_on_stale", true);
    declare_parameter<double>("publish_hz", 20.0);

    // marker_map YAML 경로
    declare_parameter<std::string>("marker_map_yaml", "");

    // 좌표 단위 변환 (marker_map이 cm이면 0.01, m이면 1.0)
    declare_parameter<double>("map_unit_scale", 0.01);  // cm -> m

    // 카메라 오프셋 (base_link -> camera_front)
    declare_parameter<bool>("apply_cam_offset", true);
    declare_parameter<double>("cam_offset_x", 0.06);   // 카메라가 base 앞쪽 6cm
    declare_parameter<double>("cam_offset_y", 0.0);

    // IMU 관련
    declare_parameter<bool>("use_imu", false);
    declare_parameter<std::string>("imu_topic", "/imu/data");
    declare_parameter<double>("imu_timeout_sec", 0.3);

    // yaw 소스: "marker" | "imu" | "fuse"
    declare_parameter<std::string>("yaw_source", "marker");
    declare_parameter<double>("yaw_fuse_imu_weight", 0.7);

    // ============ Load marker map ============
    load_marker_map();

    // ============ Pub/Sub ============
    pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      get_parameter("pose_topic").as_string(), 10);
    pub_fix_valid_ = create_publisher<std_msgs::msg::Bool>(
      "/localization/fix_valid", 10);

    sub_marker_status_ = create_subscription<rc_interfaces::msg::MarkerStatus>(
      get_parameter("marker_status_topic").as_string(), 10,
      std::bind(&LocalizationNode::on_marker_status, this, std::placeholders::_1));

    if (get_parameter("use_imu").as_bool()) {
      sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        get_parameter("imu_topic").as_string(),
        rclcpp::SensorDataQoS(),
        std::bind(&LocalizationNode::on_imu, this, std::placeholders::_1));
    }

    tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Timer
    const double hz = std::max(1.0, get_parameter("publish_hz").as_double());
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / hz));
    timer_ = create_wall_timer(period, std::bind(&LocalizationNode::on_timer, this));

    RCLCPP_INFO(get_logger(),
      "localization_node started\n"
      "  marker_status: %s\n"
      "  pose: %s\n"
      "  map_frame: %s\n"
      "  base_frame: %s\n"
      "  marker_map entries: %zu\n"
      "  map_unit_scale: %.4f",
      get_parameter("marker_status_topic").as_string().c_str(),
      get_parameter("pose_topic").as_string().c_str(),
      get_parameter("map_frame").as_string().c_str(),
      get_parameter("base_frame").as_string().c_str(),
      marker_map_.size(),
      get_parameter("map_unit_scale").as_double()
    );
  }

private:
  // ============ Load marker map from YAML ============
  void load_marker_map() {
    const std::string path = get_parameter("marker_map_yaml").as_string();
    if (path.empty()) {
      RCLCPP_WARN(get_logger(), "marker_map_yaml is empty!");
      return;
    }

    try {
      YAML::Node root = YAML::LoadFile(path);
      auto markers = root["markers"];
      if (!markers || !markers.IsSequence()) {
        RCLCPP_ERROR(get_logger(), "Invalid YAML: 'markers' not found: %s", path.c_str());
        return;
      }

      marker_map_.clear();
      for (const auto& m : markers) {
        if (!m["id"]) continue;

        const int id = m["id"].as<int>();
        MarkerMapEntry entry;
        
        // x, y가 없거나 빈 값이면 스킵
        if (m["x"] && !m["x"].IsNull()) {
          entry.x = m["x"].as<double>();
        } else {
          continue;  // 좌표 없는 마커는 무시
        }
        
        if (m["y"] && !m["y"].IsNull()) {
          entry.y = m["y"].as<double>();
        } else {
          continue;
        }
        
        entry.yaw = (m["yaw"] && !m["yaw"].IsNull()) ? m["yaw"].as<double>() : 0.0;
        
        marker_map_[id] = entry;
      }

      RCLCPP_INFO(get_logger(), "Loaded marker_map: %s (%zu valid entries)",
        path.c_str(), marker_map_.size());
        
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to load marker_map: %s", e.what());
    }
  }

  // ============ Callbacks ============
  void on_marker_status(const rc_interfaces::msg::MarkerStatus::SharedPtr msg) {
    if (!msg->valid) return;

    const double min_q = get_parameter("min_quality").as_double();
    if (msg->quality < min_q) return;

    auto it = marker_map_.find(msg->id);
    if (it == marker_map_.end()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Marker id=%d not in marker_map", msg->id);
      return;
    }

    std::lock_guard<std::mutex> lk(mtx_);
    
    last_marker_id_ = msg->id;
    last_marker_map_entry_ = it->second;
    last_rel_x_ = msg->rel_x;
    last_rel_y_ = msg->rel_y;
    last_rel_z_ = msg->rel_z;
    last_rel_yaw_ = msg->rel_yaw;
    last_marker_stamp_ = (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
      ? now() : rclcpp::Time(msg->header.stamp);
    have_marker_ = true;
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    const auto& q = msg->orientation;
    if (q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 0.0) return;

    std::lock_guard<std::mutex> lk(mtx_);
    imu_yaw_ = quat_to_yaw(q);
    imu_stamp_ = (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
      ? now() : rclcpp::Time(msg->header.stamp);
    have_imu_ = true;
  }

  // ============ Timer callback ============
  void on_timer() {
    const std::string map_frame = get_parameter("map_frame").as_string();
    const std::string base_frame = get_parameter("base_frame").as_string();
    const double timeout = get_parameter("marker_timeout_sec").as_double();
    const bool hold_last = get_parameter("hold_last_on_stale").as_bool();
    const double scale = get_parameter("map_unit_scale").as_double();

    double marker_age = 1e9;
    bool fix_valid = false;

    double robot_x = 0.0, robot_y = 0.0, robot_yaw = 0.0;

    {
      std::lock_guard<std::mutex> lk(mtx_);

      if (!have_marker_) {
        // 초기 상태 - 원점 발행
        publish_pose_and_tf(0.0, 0.0, 0.0, map_frame, base_frame, false);
        return;
      }

      marker_age = (now() - last_marker_stamp_).seconds();
      fix_valid = (marker_age <= timeout);

      if (!fix_valid && !hold_last) {
        std_msgs::msg::Bool valid_msg;
        valid_msg.data = false;
        pub_fix_valid_->publish(valid_msg);
        return;
      }

      // ============ 로봇 위치 계산 ============
      // marker_map에서 마커의 map 좌표 가져오기
      double marker_map_x = last_marker_map_entry_.x * scale;
      double marker_map_y = last_marker_map_entry_.y * scale;
      double marker_map_yaw = last_marker_map_entry_.yaw;

      // 카메라에서 본 상대 위치 (rel_z가 전방 거리, rel_x가 좌우)
      // 카메라 좌표계: Z=전방, X=오른쪽, Y=아래
      // Map 좌표계: X=전방, Y=왼쪽 (일반적인 ROS 규약)
      
      // 로봇이 마커를 보고 있을 때:
      // - 마커까지 거리: rel_z
      // - 마커 방향으로부터 로봇의 yaw 계산
      
      // 단순화: 마커 위치 = 로봇 위치 (마커 바로 앞에 있다고 가정)
      // 실제로는 rel_x, rel_z로 오프셋 계산 필요
      
      // 로봇의 yaw 결정
      robot_yaw = compute_robot_yaw(marker_map_yaw);
      
      // 카메라 오프셋 적용
      const bool apply_offset = get_parameter("apply_cam_offset").as_bool();
      const double cam_x = get_parameter("cam_offset_x").as_double();
      const double cam_y = get_parameter("cam_offset_y").as_double();

      // 로봇 위치 = 마커 위치 - 로봇->마커 벡터 (map 좌표계)
      // rel_z: 마커까지 전방 거리, rel_x: 마커의 좌우 오프셋
      const double c = std::cos(robot_yaw);
      const double s = std::sin(robot_yaw);
      
      // 카메라에서 마커까지의 벡터를 map 좌표계로 변환
      // 카메라 기준: 마커가 (rel_x, rel_y, rel_z)에 있음
      // rel_z = 전방거리, rel_x = 우측오프셋
      double dx_map = c * last_rel_z_ - s * last_rel_x_;
      double dy_map = s * last_rel_z_ + c * last_rel_x_;
      
      robot_x = marker_map_x - dx_map;
      robot_y = marker_map_y - dy_map;

      // base_link -> camera 오프셋 보정
      if (apply_offset) {
        robot_x -= c * cam_x - s * cam_y;
        robot_y -= s * cam_x + c * cam_y;
      }
    }

    if (!fix_valid) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Marker stale (age=%.2fs), holding last pose", marker_age);
    }

    publish_pose_and_tf(robot_x, robot_y, robot_yaw, map_frame, base_frame, fix_valid);

    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 500,
      "Localization: x=%.3f y=%.3f yaw=%.2f fix=%s",
      robot_x, robot_y, robot_yaw, fix_valid ? "true" : "false");
  }

  double compute_robot_yaw(double marker_map_yaw) {
    const std::string yaw_source = get_parameter("yaw_source").as_string();

    // marker_map의 yaw는 마커가 바라보는 방향
    // 로봇이 마커를 정면으로 보고 있다면, 로봇 yaw = marker_yaw + PI
    double marker_based_yaw = wrap_pi(marker_map_yaw + M_PI - last_rel_yaw_);

    if (yaw_source == "marker") {
      return marker_based_yaw;
    }

    if (yaw_source == "imu") {
      if (have_imu_) {
        double imu_age = (now() - imu_stamp_).seconds();
        if (imu_age < get_parameter("imu_timeout_sec").as_double()) {
          return imu_yaw_;
        }
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "IMU stale, fallback to marker yaw");
      return marker_based_yaw;
    }

    // fuse
    if (have_imu_) {
      double imu_age = (now() - imu_stamp_).seconds();
      if (imu_age < get_parameter("imu_timeout_sec").as_double()) {
        double w = get_parameter("yaw_fuse_imu_weight").as_double();
        w = std::clamp(w, 0.0, 1.0);
        // angle lerp
        double diff = wrap_pi(imu_yaw_ - marker_based_yaw);
        return wrap_pi(marker_based_yaw + w * diff);
      }
    }

    return marker_based_yaw;
  }

  void publish_pose_and_tf(double x, double y, double yaw,
                           const std::string& map_frame,
                           const std::string& base_frame,
                           bool fix_valid) {
    auto stamp = now();

    // PoseStamped
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = yaw_to_quat(yaw);
    pub_pose_->publish(pose);

    // TF
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = map_frame;
    tf.child_frame_id = base_frame;
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = pose.pose.orientation;
    tf_br_->sendTransform(tf);

    // fix_valid
    std_msgs::msg::Bool valid_msg;
    valid_msg.data = fix_valid;
    pub_fix_valid_->publish(valid_msg);
  }

private:
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_fix_valid_;

  // Subscribers
  rclcpp::Subscription<rc_interfaces::msg::MarkerStatus>::SharedPtr sub_marker_status_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Marker map
  std::unordered_map<int, MarkerMapEntry> marker_map_;

  // State
  std::mutex mtx_;
  
  bool have_marker_{false};
  int last_marker_id_{-1};
  MarkerMapEntry last_marker_map_entry_;
  float last_rel_x_{0.0f};
  float last_rel_y_{0.0f};
  float last_rel_z_{0.0f};
  float last_rel_yaw_{0.0f};
  rclcpp::Time last_marker_stamp_{0, 0, RCL_ROS_TIME};

  bool have_imu_{false};
  double imu_yaw_{0.0};
  rclcpp::Time imu_stamp_{0, 0, RCL_ROS_TIME};
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}