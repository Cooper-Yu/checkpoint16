#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

// Timed wheel-speed sequence using FL, FR, RL, RR order (rad/s).
class WheelVelocitiesPublisher : public rclcpp::Node
{
public:
  WheelVelocitiesPublisher() : Node("wheel_velocities_publisher")
  {
    publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_speed", 10);
    segment_started_at_ = std::chrono::steady_clock::now();
    log_current_mode();
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
      [this]() {
        const double elapsed_seconds = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - segment_started_at_).count();
        update_motion(elapsed_seconds);
      });
  }

private:
  enum class MotionMode {
    Forward,
    Backward,
    Left,
    Right,
    Clockwise,
    Counterclockwise,
    Stop
  };

  void log_current_mode()
  {
    const char * name = "Unknown";
    switch (mode_) {
      case MotionMode::Forward: name = "Forward"; break;
      case MotionMode::Backward: name = "Backward"; break;
      case MotionMode::Left: name = "Left"; break;
      case MotionMode::Right: name = "Right"; break;
      case MotionMode::Clockwise: name = "Clockwise"; break;
      case MotionMode::Counterclockwise: name = "Counterclockwise"; break;
      case MotionMode::Stop: name = "Stop"; break;
    }
    RCLCPP_INFO(get_logger(), "Starting motion: %s", name);
  }

  MotionMode next_mode(MotionMode current)
  {
    switch (current) {
      case MotionMode::Forward:
        return MotionMode::Backward;
      case MotionMode::Backward:
        return MotionMode::Left;
      case MotionMode::Left:
        return MotionMode::Right;
      case MotionMode::Right:
        return MotionMode::Clockwise;
      case MotionMode::Clockwise:
        return MotionMode::Counterclockwise;
      case MotionMode::Counterclockwise:
        return MotionMode::Stop;
      case MotionMode::Stop:
        return MotionMode::Stop;
    }
    return MotionMode::Stop;
  }

  void publish_current_mode()
  {
    switch (mode_) {
      case MotionMode::Forward:
        publish_forward();
        return;
      case MotionMode::Backward:
        publish_backward();
        return;
      case MotionMode::Left:
        publish_left();
        return;
      case MotionMode::Right:
        publish_right();
        return;
      case MotionMode::Clockwise:
        publish_clockwise();
        return;
      case MotionMode::Counterclockwise:
        publish_counterclockwise();
        return;
      case MotionMode::Stop:
        publish_stop();
        return;
    }
  }

  void begin_backward()
  {
    mode_ = MotionMode::Backward;
    segment_started_at_ = std::chrono::steady_clock::now();

  }

  // Advance timed stages without blocking; keep publishing zero after the last stage.
  void update_motion(double elapsed_seconds)
  {

    if (mode_ == MotionMode::Stop) {
      publish_current_mode();
      return;
    }

    if (elapsed_seconds < 3.0) {
      publish_current_mode();
      return;
    }

    mode_ = next_mode(mode_);
    log_current_mode();
    segment_started_at_ = std::chrono::steady_clock::now();
    publish_current_mode();
    return;
  }

  void publish_stop()
  {
    std_msgs::msg::Float32MultiArray stop_msg;
    stop_msg.data = {0, 0, 0, 0};
    publisher_->publish(stop_msg);

  }

  void publish_forward()
  {
    const float forward_speed_mps = 0.10f;
    const float wheel_radius_m = 0.05f;
    std_msgs::msg::Float32MultiArray forward_msg;
    float wheel_speed_radps = forward_speed_mps / wheel_radius_m;
    forward_msg.data = {wheel_speed_radps, wheel_speed_radps, wheel_speed_radps, wheel_speed_radps};
    publisher_->publish(forward_msg);
  }
  void publish_backward()
  {
    const float speed_magnitude_mps = 0.10f;
    const float wheel_radius_m = 0.05f;
    std_msgs::msg::Float32MultiArray backward_msg;
    float wheel_speed_radps = -speed_magnitude_mps / wheel_radius_m;
    backward_msg.data = {wheel_speed_radps, wheel_speed_radps, wheel_speed_radps, wheel_speed_radps};
    publisher_->publish(backward_msg);
  }
  void publish_left()
  {
    const float left_speed_mps = 0.10f;
    const float wheel_radius_m = 0.05f;
    std_msgs::msg::Float32MultiArray left_msg;
    float wheel_speed_radps = left_speed_mps / wheel_radius_m;
    left_msg.data = {-wheel_speed_radps, wheel_speed_radps, wheel_speed_radps, -wheel_speed_radps};
    publisher_->publish(left_msg);
  }

  void publish_right()
  {
    const float right_speed_magnitude_mps = 0.10f;
    const float wheel_radius_m = 0.05f;
    std_msgs::msg::Float32MultiArray right_msg;
    float wheel_speed_radps = right_speed_magnitude_mps / wheel_radius_m;
    right_msg.data = {wheel_speed_radps, -wheel_speed_radps, -wheel_speed_radps, wheel_speed_radps};
    publisher_->publish(right_msg);
  }
  void publish_clockwise()
  {
    const float body_angular_velocity_radps = -0.50f;
    const float half_wheelbase_m = 0.085f;
    const float half_track_m = 0.135f;
    const float wheel_radius_m = 0.05f;
    std_msgs::msg::Float32MultiArray clockwise_msg;
    float wheel_speed_radps = (half_wheelbase_m + half_track_m) * body_angular_velocity_radps / wheel_radius_m;
    clockwise_msg.data = {-wheel_speed_radps, wheel_speed_radps, -wheel_speed_radps, wheel_speed_radps};
    publisher_->publish(clockwise_msg);
  }

  void publish_counterclockwise()
  {
    const float body_angular_velocity_radps = 0.50f;
    const float half_wheelbase_m = 0.085f;
    const float half_track_m = 0.135f;
    const float wheel_radius_m = 0.05f;
    std_msgs::msg::Float32MultiArray counterclockwise_msg;
    float wheel_speed_radps = (half_wheelbase_m + half_track_m) * body_angular_velocity_radps / wheel_radius_m;
    counterclockwise_msg.data = {-wheel_speed_radps, wheel_speed_radps, -wheel_speed_radps, wheel_speed_radps};
    publisher_->publish(counterclockwise_msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  MotionMode mode_ = MotionMode::Forward;
  std::chrono::steady_clock::time_point segment_started_at_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelVelocitiesPublisher>());
  rclcpp::shutdown();
  return 0;
}
