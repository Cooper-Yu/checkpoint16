#include <chrono>
#include <array>
#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

struct Pose2D { double x; double y; double yaw; };
// Relative waypoint changes: yaw in radians; x/y in the odometry frame, in meters.
struct SegmentDelta { double dphi; double dx; double dy; };

std::optional<Pose2D> make_segment_target(
  const Pose2D & start, const SegmentDelta & delta)
{

  return Pose2D {
    start.x + delta.dx,
    start.y + delta.dy,
    start.yaw + delta.dphi
  };
}

struct PositionError { double ex; double ey; };
std::optional<PositionError> compute_position_error(
  const Pose2D & target, const Pose2D & current)
{

  return PositionError {
    target.x - current.x,
    target.y - current.y
  };
}

struct WorldVelocity { double vx; double vy; };
std::optional<WorldVelocity> compute_world_velocity(const PositionError & error)
{
  const double position_tolerance = 0.02;  // m
  const double max_speed = 0.4;           // m/s
  const double distance_gain = 0.8;       // 1/s
  const double rho = std::hypot(error.ex, error.ey);
  const double v = std::min(distance_gain * rho, max_speed);
  return rho <= position_tolerance ? WorldVelocity {0, 0} : WorldVelocity {v * error.ex / rho, v * error.ey / rho};
}

// Limit the vector change in world-frame linear velocity.
std::optional<WorldVelocity> limit_world_velocity_change(
  const WorldVelocity & previous, const WorldVelocity & desired, double max_delta)
{
  if (!std::isfinite(previous.vx) || !std::isfinite(previous.vy) ||
      !std::isfinite(desired.vx) || !std::isfinite(desired.vy) ||
      !std::isfinite(max_delta) || max_delta < 0.0) {
    return std::nullopt;
  }
  const double dx = desired.vx - previous.vx;
  const double dy = desired.vy - previous.vy;
  const double change = std::hypot(dx, dy);
  if (!std::isfinite(change)) {
    return std::nullopt;
  }
  if (change <= max_delta) {
    return desired;
  }
  return WorldVelocity {previous.vx + max_delta * dx / change, previous.vy + max_delta * dy / change};
}

struct BodyLinearVelocity { double vx; double vy; };
// Rotate the desired world velocity into the current body frame.
std::optional<BodyLinearVelocity> world_to_body(
  const WorldVelocity & world, double current_yaw)
{
  const double vx = world.vx * std::cos(current_yaw) + world.vy * std::sin(current_yaw);
  const double vy = -world.vx * std::sin(current_yaw) + world.vy * std::cos(current_yaw);

  return BodyLinearVelocity{vx, vy};
}

std::optional<double> compute_heading_error(double target_yaw, double current_yaw)
{
  const double pi = std::acos(-1.0);
  return std::remainder(target_yaw - current_yaw, 2*pi);
}

std::optional<double> compute_angular_velocity(double heading_error)
{
  const double heading_tolerance = 0.05;  // rad
  const double heading_gain = 1.0;        // 1/s
  const double max_angular_speed = 0.5;   // rad/s
  const double angular_speed = heading_gain * heading_error;
  return std::abs(heading_error) <= heading_tolerance ? 0 : std::clamp(angular_speed, -max_angular_speed, max_angular_speed);
}

// Coordinate the remaining turn with the remaining translation time.
std::optional<double> compute_coordinated_angular_velocity(
  double rho, double speed, double heading_error)
{
  const double position_tolerance = 0.02;  // m

  // Once position is reached, finish any remaining heading correction.
  if (rho <= position_tolerance) {
    return compute_angular_velocity(heading_error);
  }

  const double heading_tolerance = 0.05;  // rad
  const double max_angular_speed = 0.5;   // rad/s
  if (std::abs(heading_error) <= heading_tolerance) {
    return 0.0;
  }

  // Match the remaining turn to the translation time, then limit angular speed.
  return std::clamp(heading_error * speed / rho, -max_angular_speed, max_angular_speed);
}

// Wheel order: front left, front right, rear left, rear right; rad/s.
using WheelSpeeds = std::array<double, 4>;
std::optional<WheelSpeeds> body_to_wheels(
  const BodyLinearVelocity & body, double omega)
{
  const double wheel_radius = 0.05;  // m
  const double k = 0.085 + 0.135;
  const double vx = body.vx;
  const double vy = body.vy;
  return WheelSpeeds {
    (vx - vy - k * omega) / wheel_radius,
    (vx + vy + k * omega) / wheel_radius,
    (vx + vy - k * omega) / wheel_radius,
    (vx - vy + k * omega) / wheel_radius,
  };
}

std::optional<WheelSpeeds> compute_wheel_command(
  const Pose2D & target, const Pose2D & current,
  WorldVelocity & previous_world_velocity, double max_delta)
{
  if (!std::isfinite(target.x) || !std::isfinite(target.y) ||
      !std::isfinite(target.yaw) || !std::isfinite(current.x) ||
      !std::isfinite(current.y) || !std::isfinite(current.yaw)) {
    return std::nullopt;
  }
  const auto position_error_result = compute_position_error(target, current);
  if (!position_error_result) {
    return std::nullopt;
  }
  const auto position_error = position_error_result.value();
  const auto world_velocity = compute_world_velocity(position_error);
  if (!world_velocity) {
    return std::nullopt;
  }
  const double rho = std::hypot(position_error.ex, position_error.ey);
  // Position reached: retain the existing immediate translation stop.
  const auto limited_world_velocity = rho <= 0.02 ? world_velocity :
    limit_world_velocity_change(previous_world_velocity, world_velocity.value(), max_delta);
  if (!limited_world_velocity) {
    return std::nullopt;
  }
  const auto world_velocity_vxy = limited_world_velocity.value();
  const auto linear_velocity_result = world_to_body(world_velocity_vxy, current.yaw);
  if (!linear_velocity_result) {
    return std::nullopt;
  }
  const auto linear_velocity = linear_velocity_result.value();
  const auto heading_error_result = compute_heading_error(target.yaw, current.yaw);
  if (!heading_error_result) {
    return std::nullopt;
  }
  const auto heading_error = heading_error_result.value();
  const double speed = std::hypot(world_velocity_vxy.vx, world_velocity_vxy.vy);
  const auto angular_velocity_result = compute_coordinated_angular_velocity(rho, speed, heading_error);
  if (!angular_velocity_result) {
    return std::nullopt;
  }
  const auto angular_velocity = angular_velocity_result.value();

  const auto wheels_result = body_to_wheels(linear_velocity, angular_velocity);
  if (!wheels_result) {
    return std::nullopt;
  }
  previous_world_velocity = world_velocity_vxy;

  return wheels_result;
}

std::optional<bool> segment_reached(const Pose2D & target, const Pose2D & current)
{
  const double position_tolerance = 0.02;
  const double heading_tolerance = 0.05;
  const auto position_error_result = compute_position_error(target, current);
  if (!position_error_result) {
    return std::nullopt;
  }
  const auto position_error = position_error_result.value();
  const auto heading_error_result = compute_heading_error(target.yaw, current.yaw);
  if (!heading_error_result) {
    return std::nullopt;
  }
  const auto heading_error = heading_error_result.value();
  return  std::hypot(position_error.ex, position_error.ey) <= position_tolerance &&  std::abs(heading_error) <= heading_tolerance;
}

class EightTrajectory : public rclcpp::Node
{
public:
  EightTrajectory() : Node("eight_trajectory")
  {
    publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_speed", 10);
    // Stop on missing/stale feedback or completion; timeout uses steady wall time.
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      if (finished_) {
        publish_stop();
        if (completion_started_at_) {
          const double elapsed_seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - completion_started_at_.value()).count();
          if (elapsed_seconds >= 1) {
            RCLCPP_INFO(get_logger(), "Trajectory complete; final stop wait finished.");
            timer_->cancel();
            rclcpp::shutdown();
          }
        }
        return;
      }
      if (!current_ || !target_ || !last_pose_at_ ||
          std::chrono::steady_clock::now() - *last_pose_at_ > std::chrono::milliseconds(500)) {
        publish_stop();
        return;
      }
      update_motion();
    });
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {

        const auto & p = msg->pose.pose.position;
        const auto & q = msg->pose.pose.orientation;
        const double norm2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
            !std::isfinite(norm2) || norm2 < 1e-12) return;
        const double yaw = std::atan2(2.0*(q.w*q.z + q.x*q.y),
                                      norm2 - 2.0*(q.y*q.y + q.z*q.z));
        accept_pose(Pose2D{p.x, p.y, yaw});
        last_pose_at_ = std::chrono::steady_clock::now();
        if (current_ && target_) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "Current: %.3f %.3f %.3f; Target: %.3f %.3f %.3f",
            current_->x, current_->y, current_->yaw,
            target_->x, target_->y, target_->yaw);
        }
      });
  }
private:
  void accept_pose(const Pose2D & pose)
  {
    const SegmentDelta first_segment{0.0, 1.0, -1.0};
    current_ = pose;
    // Fix the initial target once; later feedback only updates the current pose.
    if (!target_) {
      target_ = make_segment_target(pose, first_segment);
    }
  }
  void update_motion()
  {
    const auto finished_result = segment_reached(target_.value(), current_.value());
    if (!finished_result) {
      publish_stop();
      return;
    }
    // Pause this cycle after arrival; the next cycle tracks the next planned target.
    if (finished_result.value()) {
      advance_segment();
      publish_stop();
      return;
    }

    if (!finished_) {
      // Command change limit: 0.02 m/s vector change per 100 ms wall-timer callback.
      const auto wheels_result = compute_wheel_command(
        target_.value(), current_.value(), previous_world_velocity_, 0.02);
      if (!wheels_result) {
        publish_stop();
        return;
      }
      const auto wheels = wheels_result.value();
      publish_wheels(wheels);
      return;
    }

    publish_stop();
    return;

  }

  void publish_wheels(const WheelSpeeds & wheels)
  {
    std_msgs::msg::Float32MultiArray msg;
    for (double w : wheels) {
      const float value = static_cast<float>(w);
      if (!std::isfinite(value)) { publish_stop(); return; }
      msg.data.push_back(value);
    }
    publisher_->publish(msg);
  }
  void publish_stop()
  {
    previous_world_velocity_ = WorldVelocity{0.0, 0.0};
    std_msgs::msg::Float32MultiArray msg;
    msg.data = {0.0F, 0.0F, 0.0F, 0.0F};
    publisher_->publish(msg);
  }

  const std::array<SegmentDelta, 8> segments{{
    {0.0, 1.0, -1.0}, {0.0, 1.0, 1.0},
    {0.0, 1.0, 1.0}, {-1.5708, 1.0, -1.0},
    {-1.5708, -1.0, -1.0}, {0.0, -1.0, 1.0},
    {0.0, -1.0, 1.0}, {0.0, -1.0, -1.0}
  }};
  std::size_t segment_index_{0};

  // Called only after arrival, with a valid target and segment index.
  // Accumulate from planned targets so stopping tolerance does not shift later waypoints.
  void advance_segment()
  {
    segment_index_  < segments.size() - 1 ? finished_ = false : finished_ = true;
    if (!finished_) {
      target_ = make_segment_target(target_.value(), segments[++segment_index_]);
    } else if (!completion_started_at_) {
      completion_started_at_ = std::chrono::steady_clock::now();

    }
  }

  WorldVelocity previous_world_velocity_{0.0, 0.0};
  bool finished_{false};
  std::optional<std::chrono::steady_clock::time_point> completion_started_at_;
  std::optional<std::chrono::steady_clock::time_point> last_pose_at_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<Pose2D> current_;
  std::optional<Pose2D> target_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EightTrajectory>());
  rclcpp::shutdown();
  return 0;
}
